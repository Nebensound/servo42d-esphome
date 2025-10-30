# Layer 1: Core Component (ServoXxd) - Detailed Specification

**Parent Document:** [02-cpp-interface.md](./02-cpp-interface.md)  
**Status:** 🔵 SPECIFICATION – Layer 1 implementation details

**Navigation:**
- [← Back to Overview](./02-cpp-interface.md#layer-1-core-component-transport-agnostic)
- [→ Next: Layer 2 (StepperEngine)](./02b-layer2-stepper-engine.md)

---

## Overview

**Class:** [ServoXxd](../../components/servoxxd/stepper/servoxxd.h)  
**Files:** `servoxxd.h` / `servoxxd.cpp`  
**Inherits:** 
- [`stepper::Stepper`](https://github.com/esphome/esphome/blob/dev/esphome/components/stepper/stepper.h) (ESPHome base stepper interface)
- [`modbus::ModbusDevice`](https://github.com/esphome/esphome/blob/dev/esphome/components/modbus/modbus_controller.h) (ESPHome Modbus client)
- [`Component`](https://github.com/esphome/esphome/blob/dev/esphome/core/component.h) (ESPHome component lifecycle)

**Design Pattern:** Facade - provides simple interface to complex subsystem

**Role in Architecture:**
- Entry point for all YAML actions and configurations
- Delegates movement logic to StepperEngine (Layer 2)
- Bridges ESPHome framework to our motor-specific implementation
- Manages component lifecycle and periodic polling

## Base Class Integration

The component follows ESPHome's standard architecture by inheriting from three base classes:

### 1. stepper::Stepper (ESPHome base stepper interface)

- Provides the standard stepper API that ESPHome automation expects (e.g., `set_target()`, `current_position`, `target_position`)
- Defines lifecycle hooks and state management common to all steppers
- Ensures compatibility with ESPHome's stepper actions and lambdas

> [!Important]
> The `stepper::Stepper` base class provides a **position-centric abstraction** (absolute target, current position). Our component extends this to support both **Position Mode** (using the base Stepper API) and **Speed Mode** (continuous rotation, bypassing position tracking). Internal state and action routing adapt to the configured operating mode.

### 2. modbus::ModbusDevice (ESPHome Modbus client)

- Handles RS485 communication via ESPHome's modbus component
- Provides `send()`, `on_modbus_data()`, `on_modbus_error()` for request/response flow
- Manages device address and parent modbus controller reference

### 3. Component (ESPHome component lifecycle)

- Provides `setup()`, `loop()`, `dump_config()` lifecycle methods
- Enables `set_interval()` and `set_timeout()` for periodic tasks

## Required Method Overrides

> [!Important]
> **Required Method Overrides**
>
> The following virtual methods from the base classes **must** be overridden:
>
> **From `stepper::Stepper`:**
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
> **From `modbus::ModbusDevice`:**
>
> - `void on_modbus_data(const std::vector<uint8_t> &data)` - **Required override** (pure virtual `= 0`). Process successful Modbus responses
> - `void on_modbus_error(uint8_t function_code, uint8_t exception_code)` - Optional override (virtual with empty default `{}`). Handle Modbus communication errors
> - `void on_modbus_read_registers(uint8_t function_code, uint16_t start_address, uint16_t number_of_registers)` - Optional override (virtual with empty default `{}`). Handle read register requests (server mode only)
> - `void on_modbus_write_registers(uint8_t function_code, const std::vector<uint8_t> &data)` - Optional override (virtual with empty default `{}`). Handle write register requests (server mode only)
>
> **From `Component`:**
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

## Responsibilities

- **Lifecycle Management:** setup(), dump_config(), loop()
- **Periodic Polling:** via set_interval("status_poll", ...): encoder, speed, motor status, protection status
- **Modbus Bridge:** on_modbus_data() → queue.process_response(), on_modbus_error() → queue.handle_error()
- **Public API:** Implementing all YAML actions (see [02-cpp-interface.md](./02-cpp-interface.md#public-c-api-binding))
- **Configuration Storage:** Holds all YAML configuration values
- **State Management:** Tracks runtime state, last-used parameters
- **Coordination:** Delegates to StepperEngine (Layer 2), manages helpers and sub-components

## Key Configuration Fields

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

## Runtime State

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

## Last-used Runtime Parameters

(persist across calls)

```cpp
Speed last_speed;                     // For speed mode single-parameter updates (includes unit)
Acceleration last_accel;              // Shared accel/decel (includes unit)
```

## Contracts

- **Non-blocking Operations:** All public methods must be non-blocking - they enqueue commands via CommandQueue (Layer 3) and return immediately
- **Input Validation:** Each user-facing action validates inputs and clamps to hardware-safe ranges before enqueueing
- **Unit Conversion:** Must centralize in helper functions to avoid duplication and drift
- **Position Synchronization:**
  - Whenever `current_pos_` or `target_pos_` are updated, the inherited public members `current_position` and `target_position` MUST be updated accordingly using `.steps()` (which uses stored parent_) to maintain ESPHome stepper API compatibility
  - In `loop()`, check if `target_position` changed externally (ESPHome action called base class `set_target(int32_t)`) and sync to `target_pos_` if changed
  - Both `set_target(Position)` and `set_target(int32_t)` overloads must update the same internal state consistently

## Integration with Other Layers

**Layer 2 (StepperEngine):**
- ServoXxd creates and holds StepperEngine instance
- Delegates all movement commands to engine
- Provides configuration access via parent pointer
- Calls engine_->update() in loop()

**Layer 3 (CommandQueue):**
- StepperEngine manages the queue (not ServoXxd directly)
- ServoXxd only bridges Modbus callbacks to queue

**Layer 4 (Transport):**
- ServoXxd inherits ModbusDevice for Modbus communication
- Callbacks on_modbus_data() and on_modbus_error() delegate to StepperEngine

---

**Navigation:**
- [← Back to Overview](./02-cpp-interface.md#layer-1-core-component-transport-agnostic)
- [→ Next: Layer 2 (StepperEngine)](./02b-layer2-stepper-engine.md)
