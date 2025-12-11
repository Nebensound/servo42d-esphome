# Changelog

All notable changes to the ServoXXD ESPHome component will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2025-12-11

### Added
- Initial stable release of ServoXXD component for ESPHome
- Support for MKS ServoXXD motors (28D, 35D, 42D, 57D) via Modbus RTU
- Two operating modes: POSITION and SPEED
- Three control modes: SR_OPEN, SR_CLOSE, SR_VFOC
- Comprehensive unit support for position (steps, revolutions, degrees, radians, arcminutes, arcseconds)
- Comprehensive unit support for speed (steps/s, RPM, rev/s, deg/s, rad/s, deg/min, deg/h)
- Comprehensive unit support for acceleration (steps/s², RPM/s, rev/s²)
- ENDSTOP homing mode with configurable trigger level
- Complete action set (20 actions):
  - Movement: `set_target`, `run_continuous`, `stop`, `emergency_stop`, `home`
  - Position: `report_position`, `set_zero`
  - Control: `enable`, `disable`
  - Configuration: `set_speed`, `set_acceleration`, `set_work_mode`, `set_working_current`, `set_holding_current_percent`, `set_microstepping`
  - System: `release_protection`, `restart`, `calibrate`, `key_lock`, `key_unlock`
- Comprehensive documentation with examples for:
  - Basic stepper control
  - Advanced positioning with homing
  - Multi-motor setups
  - Astronomical telescope mounts
  - Display turntables
  - Precision optical alignment
- Unit tests for all type classes (Position, Speed, Acceleration)
- CI/CD pipeline with automated testing

### Known Limitations
- VIRTUAL homing mode is not fully implemented (use `stepper.set_target` with position 0 as workaround)
- SENSORLESS homing mode is not fully implemented (configuration is written but sequence needs testing)
- Protection status register (0x3E) polling is disabled (protection errors still detected via motor status)

### Documentation
- Complete README with configuration examples
- API specification documents in `docs/specification/`
- Hardware setup guide in `examples/README.md`
- Example configurations for common use cases

### Technical Details
- Modular architecture with transport layer abstraction
- State machine-based movement control
- Asynchronous command queue with timeout handling
- Hybrid polling strategy for status updates
- ESPHome stepper component compatibility

[1.0.0]: https://github.com/Nebensound/servoxxd-esphome/releases/tag/v1.0.0
