# Layer 1: Core Component (ServoXxd) - Detailed Specification

**Parent Document:** [02-cpp-interface.md](./02-cpp-interface.md)  
**Status:** 🔵 SPECIFICATION – Layer 1 implementation details

**Navigation:**
- [← Back to Overview](./02-cpp-interface.md#layer-1-core-component-transport-agnostic)
- [→ Next: Layer 2 (StepperEngine)](./02b-layer2-stepper-engine.md)

---

## Overview

**Class:** ServoXxd  
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

## Required Method Overrides (essentials)

- From `stepper::Stepper`:
  - Optional: `on_update_speed()` if runtime reactions to speed changes are needed.
  - Non-virtual base methods (cannot override, may overload): `set_target(int32_t)`, `report_position(int32_t)`, `set_max_speed(float)`.
  - Overloads provided by this component: `set_target(Position)`, `report_position(Position)`, `set_speed(Speed)`.
  - Synchronization: Keep base members `current_position`/`target_position` in sync with internal `Position` objects; monitor external changes in `loop()`.
- From `modbus::ModbusDevice`:
  - Implement `on_modbus_data(...)` and `on_modbus_error(...)` - these forward to active ITransport implementation (Layer 4)
  - These are ESPHome-specific callbacks; SerialTransport would use different integration mechanism
  - These are implementation details of the transport layer and should not be called directly

- From `Component`:
  - Implement `setup()`, `loop()`, `dump_config()`; other lifecycle hooks optional.

These overrides bridge ESPHome's standard interfaces to the motor-specific implementation.
## Responsibilities

- **Lifecycle Management:** setup(), dump_config(), loop()
- **Periodic Polling:** via set_interval("status_poll", ...): encoder, speed, motor status, protection status
- **Transport Bridge:** ESPHome protocol callbacks → forward to ITransport implementation (Layer 4) → CommandQueue
- **Public API:** Implementing all YAML actions (see [02-cpp-interface.md](./02-cpp-interface.md#public-c-api-binding))
- **Configuration Storage:** Holds all YAML configuration values
- **State Management:** Tracks runtime state, last-used parameters
- **Coordination:** Delegates to StepperEngine (Layer 2), manages helpers and sub-components
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
- ServoXxd only bridges transport callbacks to engine
**Layer 4 (Transport + ServoCommandCodec):**
- ServoXxd inherits ModbusDevice (ESPHome framework requirement)
- ESPHome protocol callbacks are forwarded to active ITransport implementation
- Current implementation: ModbusTransport (via ModbusDevice callbacks)
- Future: SerialTransport would use different ESPHome integration (e.g., uart component callbacks)
- ServoCommandCodec (in Layer 4) provides encode/decode functions for all commands
- StepperEngine (Layer 2) uses codec to prepare command data and parse responses
- Transport layer is completely abstracted - upper layers only see ITransport and Command enumds
- StepperEngine (Layer 2) uses codec to prepare command data and parse responses

---

**Navigation:**
- [← Back to Overview](./02-cpp-interface.md#layer-1-core-component-transport-agnostic)
- [→ Next: Layer 2 (StepperEngine)](./02b-layer2-stepper-engine.md)
