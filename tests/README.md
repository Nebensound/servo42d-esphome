# Servo42D RS485 Component Tests

This directory contains test configurations for validating the component compilation.

## Test Files

### `test_compile.yaml`
**Purpose:** Full ESP32 hardware compilation test

- Uses `esp32` platform with Arduino framework
- Tests hardware-specific features including UART/RS485
- Validates complete build chain
- Requires ESP32 toolchain

**Run:** `esphome compile tests/test_compile.yaml`

**When to use:**
- ✅ Before pushing to production
- ✅ Testing hardware-specific features
- ✅ Validating complete build process
- ✅ Before creating releases
- ✅ Primary compilation test for this component

### `test_compile_host.yaml`  
**Status:** Currently not functional

**Note:** The `host` platform doesn't support UART peripherals which are essential for this RS485/MODBUS component. We focus on ESP32 hardware testing instead.

For now, use `test_compile.yaml` for all compilation tests.

## Running Tests

### Primary Test (ESP32 Hardware)
```bash
esphome compile tests/test_compile.yaml
```

This is the main test for the component since it requires UART/RS485 functionality.

## CI/CD Integration

For automated testing in CI/CD pipelines, use the host test for speed:

```yaml
# Example GitHub Actions
- name: Test ESPHome Component
  run: |
    pip install esphome
    esphome compile tests/test_compile_host.yaml
```

## Expected Results

Both tests should compile without errors. Warnings are acceptable if they come from ESPHome core or Arduino framework.

**Success:** Component compiles cleanly  
**Failure:** Check error messages for syntax or dependency issues
