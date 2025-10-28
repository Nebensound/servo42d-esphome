# C++ Interface Specification

Status: 🔵 SPECIFICATION – Defines the C++ architecture and public API for the servoxxd_modbus component implementation

Audience: This document is for developers implementing the ESPHome C++ component. It is binding to the user-facing API defined in [01-yaml-api.md](./01-yaml-api.md) and [README.md](../../README.md).

Purpose: Provide a clear, cohesive design for the C++ classes, methods, responsibilities, data flow, and contracts that implement the YAML API and behavior. The existing code can inform the design, but the implementation may be rewritten following this spec.

## Design goals

- Honor [README.md](../../README.md) and [01-yaml-api.md](./01-yaml-api.md) exactly (names, types, behavior, notes)
- Clean separation of concerns: device, control, positioning, command queue, conversions
- Deterministic, serial Modbus command execution with timeouts and deduplication
- Stable public API for actions/configs; internal details are free to evolve
- Explicit units for all public methods; centralized conversions internally
- Safe defaults and clamps to hardware limits; protect mechanics on stop

## High-level architecture

- [ServoXxdModbus](../../components/servoxxd_modbus/stepper/servo42d.h): Main component class (Facade)
  - Inherits: [stepper::Stepper](https://esphome.io/components/stepper/), [modbus::ModbusDevice](https://esphome.io/components/modbus.html), esphome::Component
  - Responsibilities: lifecycle, configuration, API, Modbus callbacks, synchronization with ESPHome
  - Delegates all movement, homing, and stop APIs to StepperEngine

- [StepperEngine](../../components/servoxxd_modbus/stepper/stepper_engine.h): Core movement and state machine logic
  - Encapsulates all movement, homing, stop, and error states as state machine
  - Manages CommandQueue and processes Modbus callbacks
  - Executes all movement actions, homing, stop, run_continuous, error handling, etc.
  - Communicates with ServoXxdModbus for configuration, status, helper functions

  - [CommandQueue](../../components/servoxxd_modbus/stepper/servo42d_command_queue.h): Serializes Modbus commands (managed by StepperEngine)
  - Modbus Commands: [ReadCommand, WriteCommand, MultiWriteCommand](../../components/servoxxd_modbus/stepper/servo42d_modbus_commands.h) (function codes 0x04, 0x06, 0x10)

### class StepperEngine

#### Responsibilities

- Encapsulates all movement, homing, and stop logic as state machine
- Processes all movement commands (move_to, stop, home, run_continuous)
- Monitors and controls internal states (Idle, Moving, Homing, Error, Disabled...)
- Manages CommandQueue and processes Modbus callbacks (response, error, timeout)
- Communicates with ServoXxdModbus for configuration, status, helper functions
- Manages and regularly updates values like encoder position (current_position) using hybrid strategy (polling + event)

#### Update Strategy for Status Values

StepperEngine manages regular polling and updates of multiple status values from the motor controller using a hybrid strategy (polling + event-triggered queries).

**Polled Status Values:**

The following values are polled at configurable intervals (default: 100–500ms):

1. **Encoder Position** (Command 0x30)
   - Split format: `carry` (int32_t) + `value` (uint16_t, 0-16383)
   - Updated continuously, even when motor is disabled
   - Primary source for `current_position` tracking
  
2. **Motor Speed** (Command 0x32)
   - Real-time speed in RPM (int16_t)
   - Positive = CCW, Negative = CW
   - Used for state transitions (e.g., Stopping → Idle when speed = 0)
  
3. **Motor Status** (Command 0x3A)
   - Motor enabled/disabled state (uint8_t: 0 = disabled, 1 = enabled)
   - Monitors motor_auto_disabled state for sleep_when_done
  
4. **Protection Status** (Command 0x3E)
   - Locked-rotor protection state (uint8_t: 0 = OK, 1 = protected)
   - Triggers automatic transition to Error state
   - Requires `release_protection()` to recover

**Optionally Polled Values:**

The following values may be polled less frequently or on-demand:

1. **IO Ports Status** (Command 0x34)
   - IN_1, IN_2, OUT_1, OUT_2 states (uint8_t bitmask)
   - Useful for limit switch monitoring or external sensors
2. **Angle Error** (Command 0x39)
   - Tracking error in encoder ticks (int32_t, 0-51200 = 0-360°)
   - Useful for tuning and diagnostics
3. **Pulse Count** (Command 0x33)
   - Number of pulses received in pulse mode (int32_t)
   - Only relevant for pulse interface mode
4. **Homing Status** (Command 0x3B)
   - Go-to-zero status when power on (uint8_t: 0 = in progress, 1 = success, 2 = fail)
   - Only polled during Homing state

**Event-Triggered Queries:**

After completion of movement commands, immediate status updates are triggered:

- After `move_to()`: Query encoder position and speed
- After `stop()`: Query speed and motor status
- After `home()`: Query homing status, then encoder position
- After `enable()`/`disable()`: Query motor status
- After error recovery: Query protection status

**Polling Strategy Requirements:**

The StepperEngine shall implement periodic status polling with the following requirements:

1. **Polling Interval:** Configurable poll interval (default: 200ms) for status value updates
2. **Polled Values:** Each poll cycle shall query:
   - Encoder position (Command 0x30)
   - Motor speed (Command 0x32)
   - Motor status (Command 0x3A)
   - Protection status (Command 0x3E)
3. **Update Cycle:** The `update()` method shall:
   - Check if poll interval elapsed since last poll
   - Execute state machine transition processing
   - Process command queue timeouts
   - Trigger next command execution if queue not empty
4. **Timing Accuracy:** Poll timing shall use millisecond precision and handle timer overflow correctly

**Status Callbacks:**

Optional callbacks can be registered for status change notifications:

- Position change callback (significant delta, e.g., >10 steps)
- Speed change callback
- Protection triggered callback
- Motor enabled/disabled callback

**Error Detection:**

- Timeouts: No response within timeout period → transition to Error state
- Protection: Protection `status != 0` → transition to Error state, log cause
- Communication errors: Modbus errors → retry with exponential backoff, transition to Error after max retries

#### Public Methods

```cpp
void move_to(Position target, std::optional<Speed> speed, std::optional<Acceleration> accel);
void stop(std::optional<Acceleration> decel);
void emergency_stop();
void home();  // Uses homing configuration from ServoXxdModbus parent (mode, direction, speed, etc.)
void run_continuous(Speed speed, Acceleration accel);
void update(); // called cyclically, processes state machine, CommandQueue, and polling
Position get_current_position() const;
void set_position_update_callback(std::function<void(Position)> cb); // optional
void handle_error(...);
void on_modbus_response(const std::vector<uint8_t>& data);
void on_modbus_error(uint8_t function_code, uint8_t exception_code);
```

#### State Machine

**States:**

```cpp
enum class State {
  Disabled,      // Motor disabled, no motion possible
  Idle,          // Motor ready, waiting for commands
  Moving,        // Position movement in progress (Position Mode only)
  Running,       // Continuous rotation in progress (Speed Mode only)
  Homing,        // Homing process in progress
  Stopping,      // Controlled stop in progress (with deceleration)
  Error          // Error occurred (e.g. Protection, Modbus error, Timeout)
};
```

**State Transitions:**

| From → To | Trigger | Condition | Action |
|-----------|---------|-----------|--------|
| Disabled → Idle | enable() | - | Enable motor, query status |
| Idle → Disabled | disable() | - | Disable motor |
| Idle → Moving | move_to() | Position Mode | Send move command, set target position |
| Idle → Running | run_continuous() | Speed Mode | Send speed command |
| Idle → Homing | home() | Position Mode | Start homing sequence |
| Moving → Idle | Target reached | Encoder position = target position | - |
| Moving → Stopping | stop() | - | Send stop command with deceleration |
| Running → Stopping | stop() | - | Send stop command with deceleration |
| Homing → Idle | Homing completed | Homing status OK | Set position offset |
| Stopping → Idle | Standstill reached | Speed = 0 | - |
| * → Error | Protection triggered | Protection register != 0 | Log error, stop motor |
| * → Error | Modbus timeout | No response | Log error, retry/abort |
| * → Error | emergency_stop() | - | Immediate halt, disable motor, set emergency flag |
| Error → Idle | release_protection() | - | Reset error/emergency flag, check status |

**Events and Processing:**

- **Commands from ServoXxdModbus:** move_to(), home(), stop(), run_continuous(), enable(), disable(), emergency_stop()
- **Modbus Responses:** Position, speed, status, protection are processed and may trigger state transitions
- **Polling Events:** Regular encoder queries, status checks trigger transitions (e.g. Moving → Idle when target reached)
- **Error Events:** Protection, timeout, Modbus errors trigger Error state

**State Machine Responsibilities:**

- Validate commands based on current state (e.g. move_to() only in Idle, not in Moving)
- Automatic transitions based on encoder feedback and status updates
- Error handling and recovery (Error → Idle after release_protection)
- Coordinate Modbus commands via CommandQueue (only valid commands in appropriate state)

**Implementation Notes:**

- update() processes state transitions based on internal events (e.g. polling results, timeouts)
- Log state transitions for debugging
- Monitor state-specific timeouts (e.g. maximum movement duration)
- For invalid commands (e.g. move_to() in Speed Mode): log error, ignore command

#### Command Validation Matrix

This table defines which commands are allowed in each state. The StepperEngine validates incoming commands and rejects/buffers them based on the current state.

| Command | Disabled | Idle | Moving | Running | Homing | Stopping | Error |
|---------|----------|------|--------|---------|--------|----------|-------|
| enable() | ✅ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ |
| disable() | ❌ | ✅ | ⏸️¹ | ⏸️¹ | ⏸️¹ | ⏸️¹ | ✅ |
| emergency_stop() | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| release_protection() | ✅ | ✅ | ❌ | ❌ | ❌ | ❌ | ✅ |
| restart() | ✅ | ✅ | ❌ | ❌ | ❌ | ❌ | ✅ |
| move_to() | ❌ | ✅² | 🔄⁷ | ❌ | ❌ | 🔄⁷ | ❌ |
| run_continuous() | ❌ | ✅³ | ❌ | ✅³ | ❌ | ❌ | ❌ |
| stop() | ❌ | ✅⁴ | ✅ | ✅ | ✅ | ✅ | ❌ |
| home() | ❌ | ✅² | ❌ | ❌ | ❌ | ❌ | ❌ |
| set_zero() | ❌ | ✅ | ❌ | ❌ | ❌ | ❌ | ❌ |
| calibrate() | ❌ | ✅ | ❌ | ❌ | ❌ | ❌ | ❌ |
| set_work_mode() | ❌ | ✅ | ❌ | ❌ | ❌ | ❌ | ❌ |
| set_working_current() | ✅ | ✅ | ⏸️⁵ | ⏸️⁵ | ❌ | ⏸️⁵ | ✅ |
| set_holding_current_percent() | ✅ | ✅ | ⏸️⁵ | ⏸️⁵ | ❌ | ⏸️⁵ | ✅ |
| set_microstepping() | ❌ | ✅ | ❌ | ❌ | ❌ | ❌ | ❌ |
| set_speed() | ✅ | ✅ | ⏸️⁶ | ✅³ | ❌ | ⏸️⁶ | ✅ |
| set_acceleration() | ✅ | ✅ | ⏸️⁶ | ✅³ | ❌ | ⏸️⁶ | ✅ |
| key_lock() / key_unlock() | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |

**Legend:**

- ✅ = Allowed (command is executed immediately)
- ❌ = Rejected (command is ignored, error logged)
- ⏸️ = Buffered (command queued, executed when state allows)
- 🔄 = Replace (override current operation with new command)

**Notes:**

1. **disable() during motion:** Stop motor first (transition to Stopping), then disable when idle
2. **Position Mode only** (checked at runtime)
3. **Speed Mode only** (checked at runtime)
4. **stop() in Idle/Stopping:** No-op (motor already stopped)
5. **Current changes during motion:** Buffer and apply after reaching Idle (optional: apply immediately if hardware supports it)
6. **Speed/acceleration changes during motion:** Buffer and apply to next movement (optional: apply immediately in Running state)
7. **move_to() during motion (Moving/Stopping) - Target Override Behavior:**
   When `move_to()` is called while the motor is already moving (state = Moving) or decelerating (state = Stopping), the component shall update the target position using one of two strategies:

   **Primary Strategy - Immediate Target Update:**
   - Immediately send new target position to hardware
   - Motor controller interrupts current movement and redirects to new target
   - Provides real-time responsiveness essential for:
     - Position tracking applications
     - Joystick/manual control interfaces
     - Reactive motion control systems
   - Compatible with standard ESPHome stepper behavior

   **Fallback Strategy - Coalescing Buffer:**
   - If hardware does not support mid-movement target updates, buffer the new target position
   - Replace any previously buffered move command (keep only the most recent target)
   - Execute buffered command when current movement completes (motor reaches Idle state)
   - Advantages:
     - Mechanically safer (no abrupt direction changes)
     - Discards outdated intermediate targets automatically
     - Ensures smooth motion profile

   **Implementation Decision:**
   - Developer must test hardware capability during development to determine which strategy the motor controller supports
   - Implement **one** of the two strategies based on test results:
     - If hardware supports mid-movement target updates → implement immediate update strategy
     - If hardware rejects mid-movement commands → implement coalescing buffer strategy
   - Document the chosen strategy and test results in code comments
   - No runtime detection or automatic fallback required - implementation choice is permanent for this hardware

**Implementation Requirements:**

The chosen implementation must satisfy the following requirements:

1. **Immediate Target Update Strategy** (if hardware supports it):
   - When `move_to()` is called during Moving or Stopping state:
     - Send new target position command immediately
     - Motor shall interrupt current movement and redirect to new target
     - State remains Moving (no transition to Idle)
     - `target_pos_` is updated to new target

2. **Coalescing Buffer Strategy** (if hardware does not support mid-movement updates):
   - When `move_to()` is called during Moving or Stopping state:
     - Buffer the new target position internally
     - Replace any previously buffered target (keep only newest)
     - Current movement continues uninterrupted
     - When motor reaches Idle state, automatically execute buffered move command
     - `target_pos_` reflects the buffered target

3. **Hardware Testing Procedure:**
   - Configure motor in Position Mode
   - Start a long-distance move (e.g., 10 full rotations at moderate speed)
   - While motor is moving, send a new target position command
   - **Observation:**
     - Hardware supports: Motor smoothly redirects to new target
     - Hardware does not support: Command fails/ignored, or motor stops unexpectedly
   - Document test results in implementation comments

#### Interface to ServoXxdModbus

- Constructor: StepperEngine(ServoXxdModbus* parent, ...);
- Access to configuration, status, helper functions via parent
- Optional callbacks for status changes

#### Example Flow

- ServoXxdModbus::set_target() → engine_->move_to(...)
- ServoXxdModbus::home() → engine_->home(...)
- ServoXxdModbus::stop() → engine_->stop(...)
- ServoXxdModbus::run_continuous() → engine_->run_continuous(...)
- ServoXxdModbus::loop() → engine_->update() (includes polling and state machine)

#### CommandQueue Integration

- StepperEngine holds and manages the CommandQueue
- update() calls queue->execute_next(), checks timeouts, processes responses/errors
- Modbus callbacks are delegated to engine->on_modbus_response()/on_modbus_error()

#### Benefits

- Improved testability and extensibility
- Facade remains clean and simple
- Movement logic is clearly encapsulated and can be developed independently
- Current position is always up-to-date and robust against communication errors

----

## Public C++ API (binding)

These methods implement the YAML actions and configuration setters. Names, signatures, and semantics must match [01-yaml-api.md](./01-yaml-api.md).

### Configuration setters (called by ESPHome during setup)

ESPHome's generated `main.cpp` calls these setters during component initialization to apply YAML configuration values.

#### Basic configuration

```cpp
void set_steps_per_revolution(float steps);  // YAML: steps_per_revolution
void set_microsteps(uint16_t subdivision);  // 1–256, YAML: microsteps
void set_speed(Speed speed);  // YAML: speed (alias: max_speed)
void set_acceleration(Acceleration acceleration);  // YAML: acceleration (affects both accel and decel)
void set_sleep_when_done(uint32_t timeout_ms);  // UINT32_MAX = disabled, 0 = immediate, 1+ = delay in ms, YAML: sleep_when_done
void set_servo_type(ServoType type);  // SERVO28D, SERVO35D, SERVO42D, SERVO57D, YAML: servo_type
void set_control_mode(WorkMode mode);  // SR_OPEN, SR_CLOSE, SR_VFOC, YAML: control_mode
void set_working_current(float current_milliamps);  // YAML: working_current
void set_holding_current_percent(uint8_t percent);  // YAML: holding_current_percent
void set_en_pin_active(EnPinActive polarity);  // LOW, HIGH, ALWAYS, YAML: en_pin_active
void set_auto_screen_off(bool enable);  // YAML: auto_screen_off
void set_lock_keys_at_startup(bool lock);  // YAML: lock_keys_at_startup
void set_mode(OperatingMode mode);  // POSITION, SPEED, YAML: mode
```

[YAML reference](./01-yaml-api.md#basic-configuration)

#### Position mode configuration

```cpp
void set_homing_mode(HomingMode mode);  // ENDSTOP, SENSORLESS, VIRTUAL, YAML: homing.mode
void set_homing_at_startup(bool enable);  // YAML: homing.at_startup
void set_homing_direction(HomingDirection dir);  // CW, CCW, NEAREST, YAML: homing.direction
void set_homing_speed(Speed speed);  // for ENDSTOP/SENSORLESS, YAML: homing.speed
void set_homing_speed_level(uint8_t level);  // 0–4 for VIRTUAL, YAML: homing.speed (zeroing_speed type)
void set_homing_endstop_trigger(EndstopTrigger trigger);  // HIGH, LOW, YAML: homing.endstop_trigger
void set_homing_current(float current_milliamps);  // YAML: homing.current
```

[YAML reference](./01-yaml-api.md#position-mode-configuration)

#### Speed mode configuration

No additional methods required (uses initial_speed and initial_acceleration).

[YAML reference](./01-yaml-api.md#speed-mode-configuration)

### Actions

#### Both modes

```cpp
void enable();  // YAML: stepper.enable
void disable();  // YAML: stepper.disable
void emergency_stop();  // YAML: stepper.emergency_stop
void stop(std::optional<Acceleration> deceleration);  // YAML: stepper.stop
void calibrate();  // YAML: stepper.calibrate
void release_protection();  // YAML: stepper.release_protection
void restart();  // YAML: stepper.restart
void key_lock();  // YAML: stepper.key_lock
void key_unlock();  // YAML: stepper.key_unlock
```

[YAML reference](./01-yaml-api.md#basic-actions)

#### Position mode only

```cpp
// Overloaded set_target methods:
void set_target(Position target);  // YAML: stepper.set_target - our implementation with Position type
// Note: Base class void set_target(int32_t steps) is also available (called by ESPHome actions)
// Both overloads coexist: Position version for our API, int32_t version for ESPHome compatibility

void report_position(Position position);  // YAML: stepper.report_position
// Note: Base class void report_position(int32_t steps) is also available (called by ESPHome actions)
void home(bool no_restart = false);  // YAML: stepper.home
void set_zero();  // YAML: stepper.set_zero
```

> [!NOTE]
> Base class void set_target(int32_t steps) is also available (called by ESPHome actions)

[YAML reference](./01-yaml-api.md#position-mode-actions)

#### Speed mode only

```cpp
void run_continuous(std::optional<Speed> speed, std::optional<Acceleration> acceleration);  // YAML: stepper.run_continuous
```

[YAML reference](./01-yaml-api.md#speed-mode-actions)

### Runtime configuration changes (action-based)

```cpp
void set_work_mode(WorkMode mode);  // SR_OPEN, SR_CLOSE, SR_VFOC, YAML: stepper.set_work_mode
void set_working_current(float current_milliamps);  // YAML: stepper.set_working_current
void set_holding_current_percent(uint8_t percent);  // YAML: stepper.set_holding_current_percent
void set_microstepping(uint16_t subdivision);  // 1–256, YAML: stepper.set_microstepping
void set_speed(Speed speed);  // YAML: stepper.set_speed
void set_acceleration(Acceleration acceleration);  // YAML: stepper.set_acceleration (affects both accel and decel)
```

[YAML reference](./01-yaml-api.md#runtime-configuration-actions)

> [!IMPORTANT]
> **Hardware Limitation:** The motor controller does not support separate acceleration and deceleration values. The `set_acceleration` method affects both rates. A `stepper.set_deceleration` action in YAML will cause a validation error.

> [!NOTE]
> All methods are non-blocking: they enqueue Modbus commands and return immediately. Unit conversions and hardware clamps are applied before enqueueing.

### Unit Type Definitions

The component uses strongly-typed classes for values with units to enable compile-time and runtime unit conversions with automatic overflow protection:

#### Speed Type

```cpp
class Speed {
  friend class ServoXxdModbus;
 public:
  Speed(float value, SpeedUnit unit);
  Speed() = default;
  
  float rpm() const;
  int16_t rpm_as_i16() const;
  int16_t rpm_for_hardware(const ServoXxdModbus* parent) const;  // Apply microstepping scaling
  float steps_per_sec(const ServoXxdModbus* parent) const;
 private:
  int16_t rpm_{0};
};

enum class SpeedUnit : uint8_t {
  STEPS_PER_SEC = 0,
  RPM = 1,
  REV_PER_SEC = 2,
  DEGREES_PER_SEC = 3,
  RADIANS_PER_SEC = 4,
  DEGREES_PER_MIN = 5,
  DEGREES_PER_HOUR = 6
};
```

> [!CAUTION]
> **Hardware Speed Calibration (Microstepping Dependency)**
>
> The motor controller's speed values are **calibrated for 16/32/64 subdivisions** as reference. For other microstepping settings, the hardware applies an automatic scaling factor based of 16 subdivisions:
> `asked_speed = actual_speed × (16 / current_microsteps)`
>
> | Microsteps | Hardware Scaling | Example: Send 1200 → Actual Motor Speed |
> |------------|------------------|------------------------------------------|
> | 8          | * 2              | 1200 → 2400 RPM                          |
> | 16/32/64   | × 1 (reference)  | 1200 → 1200 RPM                          |
> | 128        | ÷ 8              | 1200 → 150 RPM                           |
> | 256        | ÷ 16             | 1200 → 75 RPM                            |
>
> **Implementation:** `rpm_for_hardware(parent)` should apply inverse scaling if microstepping is not 16, 32 or 64.

[YAML reference](./01-yaml-api.md#speed-type)

#### Acceleration Type

```cpp
class Acceleration {
  friend class ServoXxdModbus;
 public:
  Acceleration(float value, AccelerationUnit unit);
  Acceleration() = default;
  
  uint8_t acc_internal() const;  // Hardware value 0-255
  float rpm_per_sec() const;     // Approximate RPM/s (for display/logging)
  float steps_per_sec2(const ServoXxdModbus* parent) const;
 private:
  uint8_t acc_{0};  // Hardware-native: 0-255 (inverse time mapping)
};

enum class AccelerationUnit : uint8_t {
  STEPS_PER_SEC_SQ = 0,
  RPM_PER_SEC = 1,
  REV_PER_SEC_SQ = 2,
  DEGREES_PER_SEC_SQ = 3,
  RADIANS_PER_SEC_SQ = 4
};
```

> [!CAUTION]
> **Hardware Acceleration Encoding (Inverse Time Mapping)**
>
> Die Hardware verwendet einen **nicht-linearen Wert 0-255**, der die Zeitdauer zwischen diskreten Geschwindigkeitsänderungen von ±1 RPM steuert:
>
> **Fundamentale Beziehung:**
>
> ```text
> acc = 0:    Keine Beschleunigung (Motor springt sofort auf Zielgeschwindigkeit)
> acc = 1-255: Δt = (256 - acc) × 50 μs  (Zeit zwischen ±1 RPM Änderungen)
> ```
>
> - `acc`: Hardware-Wert (0-255, wird an Motor gesendet)
> - `Δt`: Zeitintervall zwischen aufeinanderfolgenden Geschwindigkeitsänderungen von ±1 RPM
> - Bei `acc=0`: **Spezialfall** - keine Rampe, direkte Geschwindigkeitsänderung (∞ RPM/s)
> - Bei `acc=1`: Langsamste Beschleunigung (Δt = 12,75 ms → ~78 RPM/s)
> - Bei `acc=255`: Schnellste Beschleunigung (Δt = 50 μs → 20000 RPM/s)
>
> **Umrechnung User-Eingabe → Hardware:**
>
> Gegeben: Gewünschte Beschleunigung `a_user` in RPM/s
>
> ```text
> Für a_user = ∞ (keine Rampe):   acc = 0
> Für a_user > 0 (mit Rampe):     acc = 256 - (20000 / a_user)
>                                 acc = max(1, min(255, berechnet))
> ```
>
> **Umrechnung Hardware → Effektive Rate:**
>
> ```text
> Für acc = 0:       a_eff = ∞ (sofortige Änderung)
> Für acc = 1-255:   a_eff = 20000 / (256 - acc)  [RPM/s]
> ```
>
> **Wichtig:**
>
> - **acc=0 ist Spezialfall:** Wert aus YAML wird direkt übernommen, keine Rampe
> - Nicht microstepping-kalibriert (im Gegensatz zu Speed)
> - Gilt für bereits skalierte RPM-Werte
> - Nicht-lineare Beziehung: kleine acc-Änderungen bei hohen Werten → große Effekt-Unterschiede

> [!NOTE]
> **Friend Class Design:** Stores only `uint8_t acc_` (1 byte) hardware value. Conversions to user-facing RPM/s are approximate due to non-linear mapping. Constructor maps user units to nearest hardware value using inverse formula.

[YAML reference](./01-yaml-api.md#acceleration-type)

#### Position Type

```cpp
class Position {
  friend class ServoXxdModbus;
 public:
  Position(float value, PositionUnit unit);
  Position() = default;
  
  static Position from_ticks_total(uint64_t total_ticks);
  static Position from_parts(int32_t revolutions, uint16_t angle_ticks);
  
  int32_t revolutions() const;
  uint16_t angle_ticks() const;
  uint64_t ticks_total() const;
  int32_t steps(const ServoXxdModbus* parent) const;
  float degrees() const;
  float radians() const;
  uint32_t steps_as_u32(const ServoXxdModbus* parent) const;
  
  Position operator+(const Position& rhs) const;
  Position operator-(const Position& rhs) const;
  bool operator==(const Position& rhs) const;
 private:
  static constexpr uint32_t TICKS_PER_REV = 16384u;
  int32_t revs_{0};
  uint16_t angle_ticks_{0};
};

enum class PositionUnit : uint8_t {
  STEPS = 0,
  REVOLUTIONS = 1,
  DEGREES = 2,
  RADIANS = 3,
  ARCMINUTES = 4,
  ARCSECONDS = 5
};
```

> [!CAUTION]
> **Hardware Position Encoding (Split Format)**
>
> Die Hardware verwendet ein **Split-Format** basierend auf dem integrierten Encoder:
>
> **Fundamentale Struktur:**
>
> ```text
> Position = revolutions + (angle_ticks / 16384)
> ```
>
> - `revolutions`: Vollständige Umdrehungen (int32_t, -2^31..2^31-1)
> - `angle_ticks`: Winkelposition innerhalb einer Umdrehung (uint16_t, 0-16383)
> - `TICKS_PER_REV = 16384`: Encoder-Konstante (Hardware-fest, 2^14 Ticks pro Umdrehung)
>
> **Carry/Borrow Verhalten:**
>
> ```text
> Wenn angle_ticks ≥ 16384:  revolutions++, angle_ticks -= 16384  (Überlauf → nächste Umdrehung)
> Wenn angle_ticks < 0:      revolutions--, angle_ticks += 16384  (Unterlauf → vorherige Umdrehung)
> ```
>
> Valider Bereich: `angle_ticks` ist immer [0, 16383], Über-/Unterlauf wird in `revolutions` getragen
>
> **Umrechnung:**
>
> User → Hardware (Split):
>
> ```text
> total_ticks = (value × steps_per_rev × 16384) / steps_per_rev
> revs = total_ticks / 16384           (Division mit Vorzeichen)
> angle_ticks = total_ticks % 16384    (Modulo immer positiv 0-16383)
> ```
>
> Hardware (Split) → Total Ticks:
>
> ```text
> total_ticks = (revs × 16384) + angle_ticks
> ```
>
> **Wichtig:**
>
> - Encoder-natives Format (direkt aus Hardware-Feedback)
> - Arithmetik erfolgt auf `total_ticks`, dann Re-Split mit Carry/Borrow-Handling
> - Conversions zu Steps benötigen `steps_per_revolution` (via parent pointer)

**Key Design Features:**

1. **Friend classes**: Access to parent's `steps_per_revolution_` and `microsteps_` without storing pointers
2. **Hardware-native storage**:
   - Speed: `int16_t rpm_` (2 bytes, -3000..3000, microstepping-calibrated)
   - Acceleration: `uint8_t acc_` (1 byte, 0-255, inverse time mapping, NOT microstepping-scaled)
   - Position: `int32_t revs_` + `uint16_t angle_ticks_` (6 bytes, encoder-aligned split format)
3. **Minimal footprint**: Total 9 bytes for all three types

**Usage Example:**

```cpp
// Construction
Speed speed(100.0f, SpeedUnit::RPM);
Acceleration accel(100.0f, AccelerationUnit::RPM_PER_SEC);
Position pos(180.0f, PositionUnit::DEGREES);

// Internal storage: rpm_=100, acc_=56, revs_=0, angle_ticks_=8192 (9 bytes total)

// Hardware access
int16_t hw_rpm = speed.rpm_for_hardware(this);  // Needs parent for microstepping
uint8_t hw_acc = accel.acc_internal();          // Direct hardware value
uint32_t steps = pos.steps_as_u32(this);        // Needs parent for steps_per_rev

// Display/logging conversions
float display_rpm = speed.rpm();                // 100.0 RPM
float display_acc = accel.rpm_per_sec();        // ~100 RPM/s (approximate)
float display_deg = pos.degrees();              // 180.0° (exact)
```

[YAML reference](./01-yaml-api.md#position-type)

## Class design

### Overview

The component follows ESPHome's standard architecture by inheriting from three base classes:

1. **[`stepper::Stepper`](https://github.com/esphome/esphome/blob/dev/esphome/components/stepper/stepper.h)** (ESPHome base stepper interface)
   - Provides the standard stepper API that ESPHome automation expects (e.g., `set_target()`, `current_position`, `target_position`)
   - Defines lifecycle hooks and state management common to all steppers
   - Ensures compatibility with ESPHome's stepper actions and lambdas

2. **[`modbus::ModbusDevice`](https://github.com/esphome/esphome/blob/dev/esphome/components/modbus/modbus_controller.h)** (ESPHome Modbus client)
   - Handles RS485 communication via ESPHome's modbus component
   - Provides `send()`, `on_modbus_data()`, `on_modbus_error()` for request/response flow
   - Manages device address and parent modbus controller reference

3. **[`Component`](https://github.com/esphome/esphome/blob/dev/esphome/core/component.h)** (ESPHome component lifecycle)
   - Provides `setup()`, `loop()`, `dump_config()` lifecycle methods
   - Enables `set_interval()` and `set_timeout()` for periodic tasks

> [!Important]
> The `stepper::Stepper` base class provides a **position-centric abstraction** (absolute target, current position). Our component extends this to support both **Position Mode** (using the base Stepper API) and **Speed Mode** (continuous rotation, bypassing position tracking). Internal state and action routing adapt to the configured operating mode.



> [!Important]
> **Required Method Overrides**
>
> The following virtual methods from the base classes **must** be overridden:
>
> From `stepper::Stepper`:
>
> - `virtual void on_update_speed()` - Optional override, called by `SetSpeedAction` after speed changes
>   - Default implementation is empty
>   - Override if you need to react to runtime speed changes
>
> **IMPORTANT - Non-overridable methods:**
> The following `stepper::Stepper` methods are **NOT virtual** and **CANNOT be overridden**, but they **CAN be overloaded**:
>
> - `void set_target(int32_t steps)` - Directly sets `target_position` member (base class)
> - `void report_position(int32_t steps)` - Directly sets `current_position` member (base class)
> - `bool has_reached_target()` - Compares `current_position == target_position`
> - `void set_max_speed(float steps_per_second)` - Directly sets `max_speed_` member (base class, non-virtual)
>
> **Overloading Strategy:**
> We define additional overloads with our custom types:
>
> - `void set_target(Position target)` - Our Position-based API (converts to steps internally)
> - `void report_position(Position position)` - Our Position-based API (converts to steps internally)
> - `void set_speed(Speed speed)` - Our Speed-based API (coexists with base class `set_max_speed()`)
>
> **Speed Handling:**
>
> - ESPHome may call base class `set_max_speed(float steps_per_second)` → updates `max_speed_` directly
> - Our YAML config calls our `set_speed(Speed)` → converts from units and updates internal state
> - Component must monitor `max_speed_` in `loop()` for external changes and sync internal Speed representation if needed
>
> Both signatures coexist:
>
> - ESPHome actions call `set_target(int32_t)` → updates `target_position` directly
> - Our YAML actions call `set_target(Position)` → updates `target_pos_` and syncs `target_position`
> - In `loop()`, check if `target_position` changed externally and sync to `target_pos_` if needed
>
> These methods directly manipulate the public `current_position` and `target_position` members.
> Our implementation must:
>
> 1. Define our own Position-based members (`current_pos_`, `target_pos_`)
> 2. Keep the base class members synchronized whenever our Position objects change
> 3. Provide overloaded methods for both int32_t (ESPHome) and Position (our API) types
> 4. Monitor base class members in loop() for external changes from ESPHome actions
>
> From `modbus::ModbusDevice`:
>
> - `void on_modbus_data(const std::vector<uint8_t> &data)` - **Required override** (pure virtual `= 0`). Process successful Modbus responses
> - `void on_modbus_error(uint8_t function_code, uint8_t exception_code)` - Optional override (virtual with empty default `{}`). Handle Modbus communication errors
> - `void on_modbus_read_registers(uint8_t function_code, uint16_t start_address, uint16_t number_of_registers)` - Optional override (virtual with empty default `{}`). Handle read register requests (server mode only)
> - `void on_modbus_write_registers(uint8_t function_code, const std::vector<uint8_t> &data)` - Optional override (virtual with empty default `{}`). Handle write register requests (server mode only)
>
> From `Component`:
>
> - `void setup()` - **Required override** (virtual). Component initialization (motor configuration, initial state)
> - `void loop()` - **Required override** (virtual). Called every iteration (command queue processing)
> - `void dump_config()` - **Required override** (virtual). Log component configuration for diagnostics
> - `void on_shutdown()` - Optional override (virtual with empty default `{}`). Called before system shutdown
> - `void on_safe_shutdown()` - Optional override (virtual with empty default `{}`). Called during safe shutdown sequence
> - `bool teardown()` - Optional override (virtual returning `true`). Gracefully finish operations before powerdown
> - `void on_powerdown()` - Optional override (virtual with empty default `{}`). Power down hardware after teardown
>
> These overrides bridge ESPHome's standard interfaces to our motor-specific implementation.

### class ServoXxdModbus : public stepper::Stepper, public modbus::ModbusDevice, public Component

**Inheritance:** See [Overview](#overview) above.

#### ServoXxdModbus Responsibilities

- Lifecycle: setup(), dump_config(), loop()
- Periodic polling via set_interval("status_poll", ...): encoder, speed, motor status, protection status
- Bridge Modbus callbacks to CommandQueue: on_modbus_data() → queue.process_response(), on_modbus_error() → queue.handle_error()
- Public API implementing the YAML actions (above)
- Holds configuration values, last-used runtime parameters, and current state
- Coordinates helpers and sub-components

#### Key configuration fields

(backed by [01-yaml-api.md](./01-yaml-api.md))

```cpp
// Basic configuration
float steps_per_revolution;           // Required for unit conversions (YAML: steps_per_revolution)
uint16_t microsteps;                  // 1..256 (YAML: microsteps)
WorkMode control_mode;                // SR_OPEN, SR_CLOSE, SR_VFOC (YAML: control_mode)
uint16_t working_current_ma;          // YAML: working_current
uint8_t holding_current_percent;      // Ignored in SR_VFOC (YAML: holding_current_percent)
uint32_t sleep_when_done_ms;          // UINT32_MAX=disabled, 0=immediate, 1+=delay (YAML: sleep_when_done)
bool auto_screen_off;                 // YAML: auto_screen_off
bool lock_keys_at_startup;            // YAML: lock_keys_at_startup

// Position mode configuration (homing.*)
bool homing_at_startup;               // YAML: homing.at_startup
HomingMode homing_mode;               // ENDSTOP, SENSORLESS, VIRTUAL (YAML: homing.mode)
HomingDirection homing_direction;     // CW, CCW, NEAREST (NEAREST only for VIRTUAL) (YAML: homing.direction)
Speed homing_speed;                   // YAML: homing.speed
uint16_t homing_current_ma;           // SENSORLESS threshold (YAML: homing.current)
EndstopTrigger homing_endstop_trigger; // If ENDSTOP mode (YAML: homing.endstop_trigger)
uint8_t zero_mode_speed_level;        // 0..4 for VIRTUAL mode (YAML: homing.speed as zeroing_speed)

// Speed mode configuration
// (no additional config - uses initial_speed and initial_acceleration)
```

#### Runtime state

```cpp
// Position tracking (encoder-split representation)
Position current_pos_;                // Current position (from encoder + offset)
Position target_pos_;                 // Target position for moves
Position position_offset_;            // Offset for report_position zeroing

// Stepper base class members (inherited from stepper::Stepper)
// IMPORTANT: These are PUBLIC members from the base class and CANNOT be overridden:
//   int32_t current_position;  // Must be kept in sync with current_pos_.steps()
//   int32_t target_position;   // Must be kept in sync with target_pos_.steps()
// These must be updated whenever current_pos_ or target_pos_ change to maintain
// ESPHome stepper API compatibility (used by automations, lambdas, and has_reached_target())

// Motor status
int16_t motor_speed_rpm;              // Signed, from motor
uint8_t motor_status;                 // Motor status register (0x3A: 0=disabled, 1=enabled)
uint8_t protection_status;            // Hardware protection register (0x3E: 0=OK, 1=protected by locked-rotor)
bool emergency_flag_;                 // Component-level emergency flag (set by emergency_stop(), cleared by release_protection())
bool motor_auto_disabled;             // sleep_when_done applied
bool target_synced;                   // Motor has been commanded to current target
```

#### Last-used runtime parameters

(persist across calls)

```cpp
Speed last_speed;                     // For speed mode single-parameter updates (includes unit)
Acceleration last_accel;              // Shared accel/decel (includes unit)
```

#### ServoXxdModbus Contracts

- All public methods must be non-blocking: they enqueue Modbus commands via CommandQueue and return immediately.
- Each user-facing action validates inputs and clamps to hardware-safe ranges before enqueueing.
- Unit conversion must centralize in Helpers to avoid duplication and drift.
- Position synchronization:
  - Whenever `current_pos_` or `target_pos_` are updated, the inherited public members `current_position` and `target_position` MUST be updated accordingly using `.steps(steps_per_revolution)` to maintain ESPHome stepper API compatibility
  - In `loop()`, check if `target_position` changed externally (ESPHome action called base class `set_target(int32_t)`) and sync to `target_pos_` if changed
  - Both `set_target(Position)` and `set_target(int32_t)` overloads must update the same internal state consistently

### class CommandQueue

#### CommandQueue Responsibilities

- FIFO queue of commands with strict **single-flight execution**: only one command can be in EXECUTING state at any time
- **Execution Flow:**
  1. `execute_next()` checks if a command is currently executing
     - If yes: return immediately (wait for response)
     - If no: dequeue next PENDING command, send via ModbusDevice::send(), transition to EXECUTING, start timeout timer
  2. `on_modbus_data()` or `on_modbus_error()` completes current command
     - Process response data (success) or error
     - Transition command to COMPLETED or FAILED
     - Mark execution slot as free
     - Call `execute_next()` to process next pending command
  3. `process_timeout()` checks for command timeout
     - If current EXECUTING command exceeded timeout: mark as TIMEOUT, free execution slot
     - Call `execute_next()` to retry or process next command
- **Smart Command Management:**
  - **Deduplication**: Duplicate read requests for the same register shall be ignored if already queued or executing
  - **Coalescing**: Successive position/speed commands shall replace earlier pending ones (keep only newest target)
  - **Critical Commands**: emergency_stop shall clear all pending (non-executing) commands before enqueueing to ensure immediate response
- **Error Handling:**
  - On Modbus error: mark current command as FAILED, trigger completion callback, advance to next
  - On timeout: mark as TIMEOUT, advance to next command (no automatic retry - retry logic is caller's responsibility)
  - Queue never blocks: failed commands are removed, execution continues

#### Single-Flight Guarantee

**The **Single-Flight Execution Requirements:**

The CommandQueue shall ensure strictly serialized Modbus communication with the following requirements:

1. **Execution Guard:**
   - Only one command may be in EXECUTING state at any time
   - The queue shall maintain an execution guard flag to enforce single-flight behavior
   - Any attempt to execute a command while another is executing shall be deferred

2. **Command Execution Flow:**
   - `execute_next()` shall check execution guard before processing
   - If guard is set (command executing): return immediately without action
   - If guard is clear and queue not empty: execute next PENDING command
   - Executing a command shall: set execution guard, transition command to EXECUTING, start timeout timer, send Modbus request

3. **Response Handling:**
   - `on_response_received()` shall:
     - Validate that a command is currently executing
     - Process response data for current command
     - Transition command to COMPLETED state
     - Clear execution guard (CRITICAL: enables next command)
     - Call `execute_next()` to process next pending command
   - Unexpected responses (no command executing) shall be logged as warnings

4. **Error Handling:**
   - `on_error_received()` shall:
     - Validate that a command is currently executing
     - Log error details (function code, exception code)
     - Transition command to FAILED state
     - Clear execution guard (CRITICAL: enables recovery)
     - Call `execute_next()` to continue with next command
   - Failed commands shall be removed from queue, execution continues

5. **Timeout Recovery:**
   - `check_timeout()` shall periodically verify current executing command
   - If timeout exceeded:
     - Log timeout with command details
     - Transition command to TIMEOUT state
     - Clear execution guard (CRITICAL: prevents queue stall)
     - Call `execute_next()` to continue with next command

6. **Queue Processing:**
   - Completed/failed/timeout commands shall be removed from queue head
   - After removing a command, `execute_next()` shall be called (tail-recursive processing)
   - Queue shall never block: failed commands are discarded, execution continues

**Key Invariants:**

1. **Execution guard flag**: Acts as mutex - only true when waiting for Modbus response
2. **Blocking behavior**: `execute_next()` shall return immediately if execution guard is set
3. **Response unlocks**: `on_response_received()` and `on_error_received()` shall clear execution guard
4. **Timeout recovery**: `check_timeout()` shall detect stuck commands and clear execution guard
5. **Tail recursion**: After completing/failing a command, `execute_next()` shall be called to process the next one

**Integration Requirements with ServoXxdModbus::loop():**

The CommandQueue shall integrate with the component's main loop as follows:

1. **Periodic Timeout Check:** `check_timeout()` shall be called each loop iteration to detect stuck commands
2. **Opportunistic Execution:** `execute_next()` shall be called each loop iteration to start next pending command if queue is idle
3. **Response-Driven Execution:** `execute_next()` is also called after every response/error callback, so the loop calls primarily handle:
   - Initial kickstart when queue becomes non-empty
   - Recovery if callbacks are missed or delayed
   - Ensuring queue progresses even under edge conditions

### Modbus command classes

#### Command State Machine

Each Modbus command follows a lifecycle state machine that ensures reliable execution and error handling.

**States:**

```cpp
enum class CommandState {
  PENDING,    // Command in queue, not yet sent
  EXECUTING,  // Command sent to hardware, waiting for response
  COMPLETED,  // Response received and processed successfully
  FAILED,     // Modbus error received (exception code)
  TIMEOUT     // No response within timeout period
};
```

**State Transitions:**

| From → To | Trigger | Condition | Action |
|-----------|---------|-----------|--------|
| (created) → PENDING | enqueue() | - | Add command to queue |
| PENDING → EXECUTING | execute_next() | is_executing_ == false | Send Modbus request via ModbusDevice::send(), start timeout timer, set is_executing_ = true |
| EXECUTING → COMPLETED | on_response_received() | Valid response data | Process response data, call completion callback(success=true), set is_executing_ = false |
| EXECUTING → FAILED | on_error_received() | Modbus exception code | Log error details, call completion callback(success=false), set is_executing_ = false |
| EXECUTING → TIMEOUT | check_timeout() | Elapsed time > timeout | Log timeout, call completion callback(success=false), set is_executing_ = false |
| COMPLETED/FAILED/TIMEOUT → (deleted) | execute_next() | Command processed | Remove from queue head, call execute_next() for next command |

**Event Processing:**

- **enqueue()**: Add command to queue tail in PENDING state
- **execute_next()**: If no command is executing and queue not empty, transition first PENDING command to EXECUTING
- **on_response_received()**: Complete current EXECUTING command → COMPLETED, free execution slot, process next
- **on_error_received()**: Fail current EXECUTING command → FAILED, free execution slot, process next
- **check_timeout()**: Called periodically in loop(), detects stuck commands → TIMEOUT, free execution slot, process next

**Callbacks:**

- Completion callback: `std::function<void(bool success)>` - called when command reaches terminal state (COMPLETED, FAILED, or TIMEOUT)
- Data callback (optional): `std::function<void(const std::vector<uint8_t>&)>` - called on successful response with parsed data

**State Machine Invariants:**

1. Only one command can be in EXECUTING state at any time (single-flight guarantee)
2. Commands in PENDING state remain in queue until their turn
3. Terminal states (COMPLETED/FAILED/TIMEOUT) are immediately removed from queue
4. is_executing_ flag acts as mutex - prevents command interleaving

#### Command Classes

- **BaseCommand** (abstract base)
  - Metadata: register address, timeout duration
  - State: CommandState (PENDING → EXECUTING → COMPLETED | FAILED | TIMEOUT)
  - Callbacks: completion(success), optional data callback
  - Virtuals: execute(device), process_response(data), get_function_code(), get_command_name()

- **ReadCommand** (0x04 - Read Input Registers)
  - Constructor: ReadCommand(register_address, quantity)
  - Response validation: data must contain quantity×2 bytes
  - Processing: extract register values, call data callback

- **WriteCommand** (0x06 - Write Single Register)
  - Constructor: WriteCommand(register_address, value)
  - Response validation: echo of register address + value
  - Processing: confirm write success

- **MultiWriteCommand** (0x10 - Write Multiple Registers)
  - Constructor: MultiWriteCommand(register_address, values[])
  - Response validation: echo of start address and quantity (quantity 0 tolerated if address matches)
  - Processing: confirm multi-register write success

### class MotorControl

#### MotorControl Responsibilities

- Implement basic actions: enable/disable, emergency_stop, stop (controlled), restart
- Implement speed mode: run_continuous (accept optional speed and accel with units), stop with optional decel
- Implement homing modes and calibration
- Runtime configuration setters: set_work_mode, set_working_current, set_holding_current_percent, set_microstepping, key_lock/unlock, set_zero (persistent)

#### MotorControl Contracts and conversions

- run_continuous:
  - Accepts optional Speed and Acceleration objects
  - At least one parameter must be provided per call
  - Omitted parameters keep their last-used values
  - Speed sign determines direction (positive = CW, negative = CCW)
  - Get RPM from speed.rpm() and acceleration from accel.acc_internal()
  - Clamp RPM to mode-dependent limit (SR_OPEN: 400, SR_CLOSE: 1500, SR_VFOC: 3000)
- set_working_current:
  - Must validate against servo_type limits before sending to hardware
  - SERVO28D/35D/42D: max 3000mA (3.0A)
  - SERVO57D: max 5200mA (5.2A)
  - Log error and clamp if value exceeds maximum for configured servo_type
- set_homing_current:
  - Must validate against servo_type limits before sending to hardware (same limits as working_current)
  - SERVO28D/35D/42D: max 3000mA (3.0A)
  - SERVO57D: max 5200mA (5.2A)
  - Log error and clamp if value exceeds maximum for configured servo_type
- set_microstepping:
  - Must update steps_per_revolution accordingly (multiply/divide by ratio of old/new microstepping)
  - All stored Position values and targets are implicitly affected (steps change)
  - Target/offset logic remains consistent in "steps" of new microstep setting
- stop:
  - If deceleration provided, get hardware value from decel.acc_internal()
  - Else use last_accel.acc_internal() or default
  - Never use instant stop here (that's emergency_stop)
- home:
  - VIRTUAL: configure 0_Mode params (using speed level 0-4) and restart device to return to stored zero
  - SENSORLESS: use homing_current as threshold; direction and speed as configured; get RPM from speed.rpm()
  - ENDSTOP: configure endstop trigger and speed (get RPM from speed.rpm()); then issue GoHome command

### class Position

#### Position Responsibilities

- Provide position-mode moves using the device's supported modes
  - Mode 2: abs position (preferred, matches set_target)
  - Mode 3: relative position in encoder ticks (available)
  - Mode 4: absolute position in encoder ticks (available)
- Implement status queries (encoder, speed, status, protection, pulse count)
- Map encoder ticks ↔ steps using steps_per_revolution
- Maintain position_offset_ for report_position

#### Position Contracts and conversions

- set_target accepts Position object
- Get steps from target.steps(steps_per_revolution)
- Convert steps to device's expected units:
  - Modes that expect pulses (full steps): divide by microsteps if required
  - Modes that expect encoder ticks: use 16384 ticks/rev conversion
- report_position(Position):
  - Get steps from position.steps(steps_per_revolution)
  - Set position_offset_ = encoder_position_steps − steps
  - Update current_pos_ and sync base class current_position
  - Update target_pos_ = current_pos_ and sync base class target_position


## Units and conversions

The component supports multiple unit types for speed, acceleration, and position to provide flexibility and compatibility with ESPHome's stepper interface while enabling more intuitive physical units.

### Speed Conversions

**Public API:** Actions accept `Speed` struct with value and unit ([YAML type](./01-yaml-api.md#speed-type))
**Internal representation:** Motor controller requires RPM (signed int16_t)

Conversion formulas (implemented in `Speed` class constructor):

- `STEPS_PER_SEC`: `rpm = (value * 60.0) / steps_per_revolution`
- `RPM`: `rpm = value` (direct)
- `REV_PER_SEC`: `rpm = value * 60.0`
- `DEGREES_PER_SEC`: `rpm = (value * 60.0) / 360.0`
- `RADIANS_PER_SEC`: `rpm = (value * 60.0) / (2π)`
- `DEGREES_PER_MIN`: `rpm = value / 6.0`
- `DEGREES_PER_HOUR`: `rpm = value / 360.0`

Implementation: `Speed::Speed(float value, SpeedUnit unit)` constructor performs conversion and stores `rpm_` internally

### Acceleration Conversions

**Public API:** Actions accept `Acceleration` struct with value and unit ([YAML type](./01-yaml-api.md#acceleration-type))
**Internal representation:** Motor controller requires hardware value 0-255 (inverse time mapping)

Conversion formulas (implemented in `Acceleration` class constructor):

- `STEPS_PER_SEC_SQ`: `rpm_per_s = (value * 60.0) / steps_per_revolution`
- `RPM_PER_SEC`: `rpm_per_s = value` (direct)
- `REV_PER_SEC_SQ`: `rpm_per_s = value * 60.0`
- `DEGREES_PER_SEC_SQ`: `rpm_per_s = (value * 60.0) / 360.0`
- `RADIANS_PER_SEC_SQ`: `rpm_per_s = (value * 60.0) / (2π)`

Implementation: `Acceleration::Acceleration(float value, AccelerationUnit unit)` constructor performs:

1. Convert user units → RPM/s
2. Apply inverse formula: `acc_ = 256 - (20000 / rpm_per_s)`
3. Clamp to hardware range 0-255

> [!NOTE]
> The motor firmware uses a non-linear 0-255 acceleration value. The `Acceleration` class handles both conversions internally

### Position Conversions

**Public API:** Actions accept a `Position` value ([YAML type](./01-yaml-api.md#position-type))  
**Internal representation:** Split `Position` with `int32_t revolutions` + `uint16_t angle_ticks (0..16383)`

```cpp
// From YAML Position value - direct construction
Position pos(90.0f, PositionUnit::DEGREES, steps_per_revolution);

// Access in any unit
int32_t steps = pos.steps(steps_per_revolution);    // Microsteps (signed)
uint64_t ticks = pos.ticks_total();                 // Encoder ticks total (16384/rev)
float revs = pos.revolutions_f();                   // Revolutions as float
float degs = pos.degrees();                         // Degrees
float rads = pos.radians();                         // Radians

// Safe type conversions for hardware
uint32_t motor_pulses = pos.steps_as_u32(steps_per_revolution, microsteps);
```

Conversion formulas (implemented in `Position` class constructor and methods):

- Total ticks: `ticks_total = (int64_t)revolutions * 16384 + angle_ticks`
- Ticks ↔ Steps: `steps = (ticks_total * steps_per_revolution) / 16384`
- Revolutions (float): `revs_f = ticks_total / 16384.0`
- Degrees: `degrees = (revs_f * 360.0)`
- Radians: `radians = (revs_f * 2π)`
- Arcminutes: `arcminutes = degrees * 60.0` (1° = 60')
- Arcseconds: `arcseconds = degrees * 3600.0` (1° = 3600")

Implementation: `Position::Position(float value, PositionUnit unit)` constructor and accessor methods like `steps()`, `degrees()`, `radians()` perform conversions on-demand

### Encoder Ticks Mapping

Der Encoder liefert 16384 Ticks pro Umdrehung als Ground Truth. Die `Position`-Klasse kapselt das als Split-Repräsentation:

```cpp
// Reading encoder from motor
uint64_t encoder_ticks = read_encoder_register();
Position pos = Position::from_ticks_total(encoder_ticks);

// Representations stay consistent
assert(pos.ticks_total() == encoder_ticks);
assert(pos.steps(steps_per_revolution) == (encoder_ticks * steps_per_revolution) / 16384);
```

### Hardware Limits and Clamping

All conversions must respect control-mode-dependent ceilings:

- **SR_OPEN:** max 400 RPM
- **SR_CLOSE:** max 1500 RPM
- **SR_VFOC:** max 3000 RPM

Safe minima (e.g., speed ≥ 10 RPM for reliable operation) are also enforced.

Implementation: Clamps applied in action methods before enqueueing commands, using values from [max_speed YAML config](./01-yaml-api.md#max_speed)


## Behavior details and edge cases

- Shared acceleration/deceleration
  - Hardware limitation: The device only supports one internal acceleration/deceleration value
  - `set_acceleration()` affects both acceleration and deceleration rates
  - `stop()` uses the shared accel value unless an explicit decel Acceleration struct is provided
  - YAML validation must prevent use of `deceleration` config field or `stepper.set_deceleration` action

- stepper.stop in both modes ([YAML](./01-yaml-api.md#stepperstop))
  - Position mode: send a position-mode stop compatible sequence or speed-mode stop with speed=0 and controlled decel
  - Speed mode: write speed-mode (speed=0, acc>0) for a smooth stop
  - Deceleration parameter is optional and uses Acceleration struct with units

- emergency_stop ([YAML](./01-yaml-api.md#stepperemergency_stop))
  - **Hardware behavior:** Sends command 0xF7 for immediate stop (returns success/fail only)
  - **Component design decision:** Transitions to Error state and disables motor for safety
  - Sets internal `emergency_flag_` that must be cleared with release_protection()
  - **Important:** Hardware does NOT set protection register (0x3E) - this is component-level safety logic
  - Rationale: Prevents accidental motion commands after emergency stop until explicit recovery
  - Warning: Mechanically harsh at speeds >1000 RPM - prefer controlled stop() when possible

- sleep_when_done ([YAML](./01-yaml-api.md#sleep_when_done), [auto_sleep type](./01-yaml-api.md#auto_sleep-type))
  - UINT32_MAX = feature disabled (motor stays powered indefinitely)
  - 0 = disable motor immediately when idle
  - 1+ = disable motor after specified milliseconds of idle time
  - If enabled and motor is idle at target (small tolerance), disable motor according to timeout value

- steps_per_revolution unset (0)
  - Unit conversions cannot be performed; component will log errors or use fallback behavior
  - Treat encoder ticks as steps for position calculations when no conversion is available

- Changing microstepping at runtime ([YAML action](./01-yaml-api.md#stepperset_microstepping), [config](./01-yaml-api.md#microsteps))
  - steps_per_revolution must be updated accordingly by the component
  - Target/offset logic remains consistent: set_target and report_position operate on "steps" of the current microstep setting
  - All stored Speed, Acceleration, Position values with STEPS-based units are implicitly affected

- High-speed stops
  - stop() must avoid harsh decel at high RPM; clamp to a safe decel or require caller-provided decel via Acceleration parameter
  - emergency_stop reserved for hard stops where mechanical safety is less critical than immediate halt


## Mapping YAML actions to C++ methods

- enable/disable → enable(), disable() ([YAML](./01-yaml-api.md#basic-actions))
- emergency_stop → emergency_stop() ([YAML](./01-yaml-api.md#stepperemergency_stop))
- calibrate → calibrate() ([YAML](./01-yaml-api.md#steppercalibrate))
- release_protection → release_protection() ([YAML](./01-yaml-api.md#stepperrelease_protection))
- restart → restart() ([YAML](./01-yaml-api.md#stepperrestart))
- set_work_mode → set_work_mode(WorkMode) ([YAML](./01-yaml-api.md#stepperset_work_mode))
- set_working_current → set_working_current(float mA) ([YAML](./01-yaml-api.md#stepperset_working_current))
- set_holding_current_percent → set_holding_current_percent(uint8_t) ([YAML](./01-yaml-api.md#stepperset_holding_current_percent))
- set_microstepping → set_microstepping(uint16_t) ([YAML](./01-yaml-api.md#stepperset_microstepping))
- set_speed → set_speed(Speed) ([YAML](./01-yaml-api.md#stepperset_speed))
- set_acceleration → set_acceleration(Acceleration) ([YAML](./01-yaml-api.md#stepperset_acceleration))
- stop → `stop(optional<Acceleration>)` ([YAML](./01-yaml-api.md#stepperstop))
- key_lock / key_unlock → key_lock(), key_unlock() ([YAML](./01-yaml-api.md#stepperkey_lock--stepperkey_unlock))
- set_target → set_target(Position) ([YAML](./01-yaml-api.md#stepperset_target))
- report_position → report_position(Position) ([YAML](./01-yaml-api.md#stepperreport_position))
- home → home(bool no_restart = false) ([YAML](./01-yaml-api.md#stepperhome))
- set_zero → set_zero() ([YAML](./01-yaml-api.md#stepperset_zero))
- run_continuous → `run_continuous(optional<Speed>, optional<Acceleration>)` ([YAML](./01-yaml-api.md#stepperrun_continuous))


## Lifecycle

- setup()
  - Initialize queue and submodules
  - If `lock_keys_at_startup` is configured, send key lock command to device
  - Apply initial configuration (mode, microstepping, working current, EN polarity, auto screen off, zero-mode params)
  - Enable motor and schedule periodic polling intervals

- loop()
  - CommandQueue::process_next() and execute_next()

- update() (polling interval)
  - Query encoder, motor speed, motor status, protection status
  - Apply sleep_when_done
  - Restore working current after homing override (if used)


## Error handling and protection

**Hardware Protection (locked-rotor detection):**

- Register 0x3E tracks hardware protection status (0 = OK, 1 = protected)
- Automatically triggered when motor shaft is blocked (if protection feature enabled in settings)
- Transitions component to Error state
- Requires release_protection (command 0x3D) to clear hardware status

**Component Emergency Flag (design decision):**

- Internal `emergency_flag_` set by component when emergency_stop() is called
- NOT a hardware feature - this is component-level safety logic to prevent accidental motion after emergency
- Transitions component to Error state and disables motor
- Also requires release_protection() to clear flag and re-enable normal operation
- Rationale: Treat emergency_stop similar to hardware protection for consistent recovery flow

**Error Recovery Flow:**

- After protection event (hardware 0x3E = 1) OR emergency_stop (component flag):
  - Motor is in Error state
  - Call release_protection() to:
    - Send command 0x3D (clears hardware protection if set)
    - Clear internal emergency_flag_ (if set by emergency_stop)
    - Re-enable motor and transition back to Idle state

**Modbus Communication Errors:**

- Errors/timeout transition commands to FAILED/TIMEOUT
- Trigger completion callbacks
- Queue advances to next command (no blocking)


## Example call flows

- set_target(Position target) - our overload
  - If motor auto-disabled, enable first
  - Convert Position to steps using target.steps(steps_per_revolution)
  - Update target_pos_ = target
  - Update base class target_position = target_pos_.steps(steps_per_revolution)
  - Get speed_rpm from last_speed.rpm() or defaults
  - Get acceleration from last_accel.acc_internal() or defaults
  - Send Mode 2 absolute move with position_offset_ applied
  - Update target_synced flag

- loop() - monitoring external changes
  - If base class target_position != target_pos_.steps():
    - External change detected (ESPHome action called base class set_target(int32_t))
  - Reconstruct target_pos_ from target_position via Position::from_steps()
  - Trigger motor move command with new target

- `run_continuous(optional<Speed> speed, optional<Acceleration> accel)`
  - At least one parameter must be provided
  - If speed provided:
    - Extract sign to determine direction (positive = CW, negative = CCW)
    - Get RPM from speed.rpm()
    - Update last_speed
  - If accel provided:
    - Get hardware value from accel.acc_internal()
    - Update last_accel
  - Clamp RPM to control-mode limits
  - Send speed-mode payload (direction, acceleration, RPM)

- `stop(optional<Acceleration> decel)`
  - If decel provided:
    - Get hardware value from decel.acc_internal()
  - Else use last_accel.acc_internal() or default
  - Send speed-mode payload with speed=0 and controlled deceleration


## Unit Conversion Architecture

**Design Principle:** All unit conversions happen in C++ at runtime, not in Python at build time.

### Python Responsibility

The Python validators (`validate_speed_with_unit`, `validate_acceleration_with_unit`, `validate_position_with_unit`) handle:

1. **Unit Parsing**: Parse string inputs like "60 RPM", "1.5 rev/s", "360 deg/s"
2. **Unit Alias Mapping**: Map aliases to canonical enums (e.g., "rpm", "RPM", "rev/min" → `SpeedUnit::RPM`)
3. **Unit Prefix Handling**: Handle SI prefixes (k for kilo, m for milli, etc.)
4. **Validation**: Ensure values are positive, in valid ranges
5. **Output Format**: Always return `{value: float, unit: enum}` dict
6. **Pass to C++**: Pass both value and unit enum to C++

**Python does NOT:**

- Convert between units (no mathematical conversion)
- Access `steps_per_revolution` for conversion
- Perform build-time unit conversion

### C++ Responsibility

The C++ unit type classes (`Speed`, `Acceleration`, `Position`) handle:

1. **Runtime Conversion**: Convert at runtime using the actual `steps_per_revolution`
2. **Unit Enum Interpretation**: Switch on unit enum in constructor
3. **Hardware Adaptation**: Store hardware-native format (RPM, 0-255 acc, encoder ticks)
4. **Clamping**: Clamp to hardware limits after conversion

### Conversion Implementation

Type class constructors convert from any unit to hardware-native format:

- **Speed**: Constructor switches on `SpeedUnit`, converts to `int16_t rpm_`
- **Acceleration**: Constructor converts to RPM/s, then applies inverse formula to `uint8_t acc_` (0-255)
- **Position**: Constructor converts to total ticks, splits into `int32_t revs_` + `uint16_t angle_ticks_`

Conversion formulas are documented in the [Speed Conversions](#speed-conversions), [Acceleration Conversions](#acceleration-conversions), and [Position Conversions](#position-conversions) sections above.

### Implementation Flow

**Actions:**
- Python passes `{value, unit}` dict to C++ Action classes
- Action class stores both values as members
- In `play()`: construct type object (e.g., `Position(value_, unit_)`), conversion happens in constructor
- Call component method with constructed object

**Configuration:**
- Python constructs type objects during setup, passes to component setters
- Component stores objects (e.g., `last_speed_`, `last_accel_`)
- Access hardware values when needed via methods (`.rpm()`, `.acc_internal()`, `.steps()`)


## Open items and extensions

- Sensors (if defined later in 01-yaml-api.md): design sensor entities for current position, speed RPM, protection status etc.
- Fine-tune acceleration_to_internal mapping for better UX
- Add configurable polling interval; backoff on errors
- Optional batching of status queries to reduce bus load


## Acceptance criteria

See also in README:

- [Speed Mode](../../README.md#speed-mode)
- [Position Mode](../../README.md#position-mode)

Additional criteria:

- All YAML actions compile to corresponding C++ methods with matching names and types
- Unit conversions verified against examples in 01-yaml-api.md
- Shared accel/decel and stop-in-both-modes semantics implemented
- Command queue serializes Modbus requests with deduplication and timeouts
- Position and speed examples in README.md work as described
- **Unit conversion happens exclusively in C++ at runtime with `steps_per_revolution`**
- **Python validators only parse units and pass value+unit to C++**
