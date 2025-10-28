# Unit Tests

C++ unit tests for the ServoXxd component type classes.

## Overview

These tests verify the unit conversion logic for all three type classes:
- **Speed**: 7 units (steps/s, RPM, rev/s, deg/s, rad/s, deg/min, deg/h)
- **Acceleration**: 5 units (steps/s², RPM/s, rev/s², deg/s², rad/s²)
- **Position**: 6 units (steps, revolutions, degrees, radians, arcminutes, arcseconds)

## Requirements

- C++20 compiler (g++ or clang++)
- Make

## Running Tests

### Build and run all tests:
```bash
make test
```

### Run individual tests:
```bash
make test_speed
./test_speed

make test_acceleration
./test_acceleration

make test_position
./test_position
```

### Clean build artifacts:
```bash
make clean
```

## Test Coverage

### Speed Class (`test_speed.cpp`)
- ✅ All 7 unit conversions to RPM
- ✅ Microstepping compensation (8, 16, 32, 64, 128, 256)
- ✅ Steps/s conversion for ESPHome
- ✅ Negative values (reverse direction)
- ✅ Range clamping (int16_t limits)

### Acceleration Class (`test_acceleration.cpp`)
- ✅ All 5 unit conversions to hardware value (0-255)
- ✅ Inverse time mapping (non-linear)
- ✅ Instant acceleration (acc=0)
- ✅ Maximum acceleration (acc=255, 20000 RPM/s)
- ✅ RPM/s approximation accuracy
- ✅ Steps/s² conversion for ESPHome
- ✅ Boundary clamping

### Position Class (`test_position.cpp`)
- ✅ All 6 unit conversions to encoder ticks
- ✅ Split format (revolutions + angle_ticks)
- ✅ Carry/borrow behavior
- ✅ Negative positions
- ✅ Arithmetic operators (+, -, ==)
- ✅ Zero position
- ✅ All unit accessor methods

## Notes

- Tests use `std::numbers::pi_v<float>` from C++20 for mathematical constants
- Float comparisons use EPSILON tolerance for floating-point accuracy
- Mock classes provide minimal ServoXxdModbus interface for testing
- All tests include verbose output for debugging
