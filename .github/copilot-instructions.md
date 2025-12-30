# ServoXXD ESPHome Component - AI Coding Agent Guide

## Project Overview

ESPHome external component for controlling MKS ServoXXD (28D/35D/42D/57D) closed-loop stepper motors via RS485/Modbus RTU. Provides position and speed control through YAML configuration and Home Assistant integration.

## Architecture: 4-Layer Design

**Critical**: This is a layered architecture - understand the boundaries before making changes.

```
Layer 1 (ServoXxd)         → Facade, YAML API, ESPHome integration
Layer 2 (StepperEngine)    → State machine, movement coordination
Layer 3 (CommandQueue)     → Single-flight execution, deduplication
Layer 4 (Transport)        → Modbus/Serial abstraction (Command enum, ITransport)
```

**Key files by layer:**
- L1: [servoxxd.{h,cpp}](../components/servoxxd/stepper/servoxxd.h)
- L2: [servoxxd_stepper_engine.{h,cpp}](../components/servoxxd/stepper/servoxxd_stepper_engine.h)
- L3: [servoxxd_command_queue.{h,cpp}](../components/servoxxd/stepper/servoxxd_command_queue.h)
- L4: [servoxxd_transport.h](../components/servoxxd/stepper/servoxxd_transport.h), [servoxxd_modbus.{h,cpp}](../components/servoxxd/stepper/servoxxd_modbus.h), [servoxxd_command_decoder.{h,cpp}](../components/servoxxd/stepper/servoxxd_command_decoder.h)

**Design principles:**
- Each layer communicates only with adjacent layers
- Layer 2 is transport-agnostic (enables future Serial/CAN support)
- Commands flow down; callbacks flow up
- Read [docs/specification/02-cpp-interface.md](../docs/specification/02-cpp-interface.md) before architectural changes

## Type-Safe Unit System

**Three wrapper classes** handle all physical quantities with compile-time unit safety:

1. **Speed** ([servoxxd_speed.{h,cpp}](../components/servoxxd/stepper/servoxxd_speed.h)) - 7 units: RPM, steps/s, rev/s, deg/s, rad/s, deg/min, deg/h
2. **Acceleration** ([servoxxd_acceleration.{h,cpp}](../components/servoxxd/stepper/servoxxd_acceleration.h)) - 5 units: RPM/s, steps/s², rev/s², deg/s², rad/s²
3. **Position** ([servoxxd_position.{h,cpp}](../components/servoxxd/stepper/servoxxd_position.h)) - 6 units: steps, revolutions, degrees, radians, arcminutes, arcseconds

**Pattern:** Internal storage in canonical units (RPM, RPM/s, steps), conversions via named constructors:
```cpp
Speed::from_rpm(60.0f, steps_per_rev);
Acceleration::from_steps_per_sec_squared(500.0f, steps_per_rev);
Position::from_degrees(90.0f, steps_per_rev);
```

**Critical**: All public APIs use these types, not raw floats. Hardware encoding happens only at transport boundary.

## Working with Specifications

**Critical**: This project is specification-driven. The specs in [../docs/specification/](../docs/specification/) are authoritative.

**Purpose of specifications:**
- Quickly grasp high-level concepts (for AI and human developers)
- Enable simultaneous work by multiple contributors (clear interfaces)
- Define "what" and "why", not "how" - keep specs concise, no implementation details

**Workflow:**
1. **Before changes**: Read relevant spec first ([01-yaml-api.md](../docs/specification/01-yaml-api.md), [02-cpp-interface.md](../docs/specification/02-cpp-interface.md))
2. **When changes needed**: Discuss with user, update spec first
3. **Implementation**: Follow spec exactly - spec is the contract
4. **Never**: Implement features not in spec without updating spec first

**Spec vs Implementation:**
- Spec describes interfaces, behaviors, contracts
- Implementation lives in code, not in spec
- If unsure where something belongs: Ask user
- Rough Guideline: If something is implementation only and does not affect other layer do not add it to the spec. Basically everything else should go into the spec.

## Development Workflows

### Building & Testing

**Unit tests** (preferred for type classes):
```bash
cd tests/unit
make test              # Run all tests (6 test suites)
make test_speed        # Build & run specific test
```

**ESPHome compilation test**:
```bash
source activate esphome  # if using conda
esphome compile tests/esphome/test_compile.yaml
```

**Hardware validation** (requires ESP32 + motor):
```bash
esphome compile tests/esphome/test_hardware.yaml
esphome upload tests/esphome/test_hardware.yaml # Do not use esphome run!
timeout 30s esphome logs tests/esphome/test_hardware.yaml  # Always wrap logs in timeout
```

### Adding New Commands

**Critical**: Determine the appropriate layer first - the separation is fundamental!

**For internal/transport-only commands** (Layer 4, no YAML exposure):
1. Add enum to `Command` in [servoxxd_transport.h](../components/servoxxd/stepper/servoxxd_transport.h)
2. Implement encoder/decoder in [servoxxd_command_decoder.{h,cpp}](../components/servoxxd/stepper/servoxxd_command_decoder.cpp)
3. Write unit test in [tests/unit/test_command_decoder.cpp](../tests/unit/test_command_decoder.cpp)
4. If affects movement: Update state machine in StepperEngine (Layer 2)
5. If async: Add to CommandQueue (Layer 3)

**For user-facing commands** (exposed in YAML/ESPHome):
1. **Update specification first** - Discuss with user, modify [docs/specification/01-yaml-api.md](../docs/specification/01-yaml-api.md)
2. Follow all steps above (Layer 4 → Layer 3 → Layer 2)
3. Add public API to ServoXxd (Layer 1) in [servoxxd.h](../components/servoxxd/stepper/servoxxd.h)
4. Implement Python validation in [components/servoxxd/__init__.py](../components/servoxxd/__init__.py)
5. Add YAML action/config example to [examples/](../examples/)
6. Update user docs in [README.md](../README.md)

**Pattern**: Commands flow Layer 4 → Layer 3 → Layer 2 → Layer 1 → YAML (if user-facing)

### YAML API Changes

**Specification-driven**: YAML API is defined in [docs/specification/01-yaml-api.md](../docs/specification/01-yaml-api.md)

1. Update specification first
2. Implement Python validation in [components/servoxxd/__init__.py](../components/servoxxd/__init__.py)
3. Update C++ config structs in [servoxxd.h](../components/servoxxd/stepper/servoxxd.h)
4. Add example to [examples/](../examples/)
5. Update [README.md](../README.md) user documentation

## Critical Conventions

### ESPHome Integration
- Component inherits from `stepper::Stepper`, `modbus::ModbusDevice`, `Component`
- Modbus callbacks (`on_modbus_data`, `on_modbus_error`) forward to transport layer
- Must keep base class `current_position`/`target_position` synchronized
- Use `#ifdef degrees / #undef degrees` to handle Arduino macro conflicts

### State Machine (Layer 2)
States: `Disabled`, `Idle`, `Moving`, `Running`, `Homing`, `Stopping`, `Error`
- Only Layer 2 manages state transitions
- `emergency_stop()` bypasses queue, clears all pending commands
- Position Mode vs Speed Mode determined by `OperatingMode` config

### Command Queue (Layer 3)
- **Single-flight**: Only one command executing at a time
- **Deduplication**: Identical read commands merged, callbacks combined
- **Timeouts**: 1000ms default, retries with exponential backoff
- **Priority**: `emergency_stop` clears entire queue immediately

### Hardware Communication
**Motor supports two protocols**: Modbus RTU (RS485) and Serial (UART)
- **Currently implemented**: Modbus RTU only
- **Architecture**: Transport-agnostic design (Layer 4) enables future Serial support
- Layer 2 (StepperEngine) is protocol-independent - uses `ITransport` interface

**Modbus RTU specifics**:
- Register addresses: 0x0000-0x00FF (see [docs/servo_hardware_doc/AI/](../docs/servo_hardware_doc/AI/))
- All multi-byte values are big-endian (Modbus standard)
- Position uses split format: encoder_carry (int16) + encoder_addition (uint16)
- Speed hardware encoding: RPM directly as uint16

## Testing Strategy

**Unit tests** for pure logic (type conversions, command encoding, queue behavior):
- Fast (<1s), no hardware needed
- 160+ assertions across 6 test suites
- Mock ESPHome dependencies via [tests/unit/esphome/](../tests/unit/esphome/)

**ESPHome tests** for integration/compilation:
- `test_compile.yaml` - Full ESP32 build validation
- `test_hardware.yaml` - Real motor interaction

**Test-first approach**: Write unit test before implementing complex logic (especially for type classes and command decoders).

## Common Pitfalls

1. **Don't bypass the type system** - Use `Speed::from_rpm()`, not raw float calculations
2. **Layer violations** - Layer 1 never calls Transport directly; always through Layer 2
3. **Missing microstep conversion** - Speed/Position conversions require `steps_per_revolution` parameter
4. **Arduino macro conflicts** - Always `#undef degrees` and `#undef radians` in headers
5. **Ignoring specifications** - Read [docs/specification/](../docs/specification/) before changing public APIs
6. **Hardcoded units** - User can specify any supported unit in YAML; validate in Python, convert in C++

## File Organization

```
components/servoxxd/
  __init__.py                    # Python validation (YAML → C++ config)
  stepper/
    servoxxd.{h,cpp}            # Layer 1: Main component class
    servoxxd_stepper_engine.*   # Layer 2: State machine
    servoxxd_command_queue.*    # Layer 3: Async execution
    servoxxd_transport.h        # Layer 4: ITransport interface
    servoxxd_modbus.*           # Layer 4: Modbus implementation
    servoxxd_command_decoder.*  # Layer 4: Command encode/decode
    servoxxd_{speed,accel,pos}.*# Type-safe unit wrappers
    servoxxd_actions.h          # YAML action templates
    servoxxd_commands.h         # Command enum definitions

docs/specification/
  01-yaml-api.md               # User-facing API spec (authoritative)
  02-cpp-interface.md          # C++ architecture overview
  02a-layer1-core.md           # Layer 1 details
  02b-layer2-stepper-engine.md # Layer 2 details
  02c-layer3-command-queue.md  # Layer 3 details
  02d-layer4-transport.md      # Layer 4 details
  03-homing-implementation.md  # Homing logic spec

examples/
  basic_stepper.yaml           # Minimal config
  advanced_positioning.yaml    # Full Home Assistant integration
  multi_motor.yaml             # Multi-axis setup
```

## References

- **Hardware manual**: [docs/servo_hardware_doc/AI/MKS_SERVO42D57D_RS485_User_Manual_V1.0.6.txt](../docs/servo_hardware_doc/AI/MKS_SERVO42D57D_RS485_User_Manual_V1.0.6.txt)
- **Modbus RTU**: Standard for RS485 communication (addresses 0x01-0xF7)
- **ESPHome docs**: https://esphome.io/ (especially stepper, modbus, component)
