# YAML API Specification

**Status:** 🔵 **SPECIFICATION** – Defines the user-facing YAML configuration interface

**Audience:** This document is for **developers** implementing the component. For end-user documentation, see [README.md](../../README.md).

**Purpose:** This document specifies the complete YAML configuration API for the `servo42d_rs485` component. It defines what users can configure, valid values, defaults, and validation rules. This serves as the authoritative reference for implementing the Python validation layer (`__init__.py`).


## Type Definitions

This section defines reusable types used throughout the configuration and actions.

### `modbus_address` Type

**Description:** Modbus RTU slave address in hexadecimal format.

**Accepted Values:**
- Hexadecimal format: `0x01` to `0xF7` (1-247 decimal)
- Examples: `0x01`, `0x10`, `0xF7`

**Validation:**
```python
cv.All(
    cv.hex_uint8_t,           # ESPHome: Validates hex format (0x00-0xFF)
    cv.Range(min=1, max=247)  # Component: Validates Modbus RTU range
)
```

**C++ Type:** `uint8_t`

**References:**
- [`cv.hex_uint8_t`](https://github.com/esphome/esphome/blob/dev/esphome/config_validation.py) from ESPHome core
- Range 1-247 per Modbus RTU specification (0, 248-255 reserved)
### `speed` Type

**Description:** Motor speed with multiple unit options. Converted to RPM for motor communication.

**Default Unit:** `STEPS_PER_SEC` (for ESPHome stepper compatibility)

**Accepted Values:**
- **Steps per second:** `steps/s`, `steps/sec` → `STEPS_PER_SEC`
  - Example: `1000 steps/s`
- **Revolutions per minute:** `RPM` → `RPM`
  - Example: `60 RPM`
- **Revolutions per second:** `rev/s`, `rev/sec`, `revolutions/s` → `REV_PER_SEC`
  - Example: `1.5 rev/s`
- **Degrees per second:** `deg/s`, `deg/sec`, `°/s` → `DEGREES_PER_SEC`
  - Example: `360 deg/s`
- **Radians per second:** `rad/s`, `rad/sec` → `RADIANS_PER_SEC`
  - Example: `6.28 rad/s`
- **Metric prefixes:** `k` (kilo), `M` (mega), `m` (milli), `µ/u` (micro)

> [!NOTE]
> Unit aliases (e.g., `steps/s`, `steps/sec`) are mapped to the same enum value during validation. The `unit` field in dicts is **not templatable** and must be a compile-time constant.

> [!NOTE]
> Metric prefixes may only be used when the value is a plain number or string, not if its a lambda.

**Validation:**
- Use `cv.float_with_unit()` for metric prefix support
- Map unit string aliases to enum values
- Return dict: `{"value": float, "unit": enum_string}`

**Code Generation:**

Static values → Convert to RPM at build-time:
- `STEPS_PER_SEC`: `rpm = (value * 60.0) / steps_per_revolution`
- `RPM`: `rpm = value`
- `REV_PER_SEC`: `rpm = value * 60.0`
- `DEGREES_PER_SEC`: `rpm = (value * 60.0) / 360.0`
- `RADIANS_PER_SEC`: `rpm = (value * 60.0) / (2π)`

Lambdas → Pass to C++ for runtime conversion:
- `value`: Lambda code (templatable)
- `unit`: Enum integer

**Unit Enum (C++):**
```cpp
enum class SpeedUnit : uint8_t {
    STEPS_PER_SEC,     // Default - ESPHome stepper compatibility
    RPM,
    REV_PER_SEC,
    DEGREES_PER_SEC,
    RADIANS_PER_SEC
};
```

**C++ Type:** `int16_t` (RPM, signed for bidirectional)

**References:**
- Uses [`cv.float_with_unit()`](https://github.com/esphome/esphome/blob/dev/esphome/config_validation.py) internally per unit



### `acceleration` Type

**Description:** Motor acceleration with multiple unit options. Converted to RPM/s for motor communication.

**Default Unit:** `STEPS_PER_SEC_SQ` (for ESPHome stepper compatibility)

**Accepted Values:**
- **Steps per second squared:** `steps/s²`, `steps/s/s` → `STEPS_PER_SEC_SQ`
  - Example: `1000 steps/s²`
- **RPM per second:** `RPM/s`, `RPM/sec` → `RPM_PER_SEC`
  - Example: `100 RPM/s`
- **Revolutions per second squared:** `rev/s²`, `rev/s/s` → `REV_PER_SEC_SQ`
  - Example: `1.5 rev/s²`
- **Degrees per second squared:** `deg/s²`, `deg/s/s`, `°/s²` → `DEGREES_PER_SEC_SQ`
  - Example: `360 deg/s²`
- **Radians per second squared:** `rad/s²`, `rad/s/s` → `RADIANS_PER_SEC_SQ`
  - Example: `6.28 rad/s²`
- **Metric prefixes:** `k` (kilo), `M` (mega), `m` (milli), `µ/u` (micro)

> [!NOTE]
> Unit aliases (e.g., `steps/s²`, `steps/s/s`) are mapped to the same enum value during validation. The `unit` field in dicts is **not templatable** and must be a compile-time constant.

**Validation:**
- Use `cv.float_with_unit()` for metric prefix support
- Map unit string aliases to enum values
- Return dict: `{"value": float, "unit": enum_string}`

**Code Generation:**

Static values → Convert to RPM/s at build-time (clamp to 0-65535):
- `STEPS_PER_SEC_SQ`: `rpm_per_s = (value * 60.0) / steps_per_revolution`
- `RPM_PER_SEC`: `rpm_per_s = value`
- `REV_PER_SEC_SQ`: `rpm_per_s = value * 60.0`
- `DEGREES_PER_SEC_SQ`: `rpm_per_s = (value * 60.0) / 360.0`
- `RADIANS_PER_SEC_SQ`: `rpm_per_s = (value * 60.0) / (2π)`

Lambdas → Pass to C++ for runtime conversion:
- `value`: Lambda code (templatable)
- `unit`: Enum integer

**Unit Enum (C++):**
```cpp
enum class AccelerationUnit : uint8_t {
    STEPS_PER_SEC_SQ = 0,  // Default - ESPHome stepper compatibility
    RPM_PER_SEC = 1,
    REV_PER_SEC_SQ = 2,
    DEGREES_PER_SEC_SQ = 3,
    RADIANS_PER_SEC_SQ = 4
};
```

**C++ Type:** `uint16_t` (RPM/s, unsigned since acceleration is always positive)

**References:**
- ESPHome `cv.float_with_unit`: https://esphome.io/components/sensor/index.html#config-validation



### `position` Type

**Description:** Motor position with multiple unit options. Converted to steps for motor communication.

**Default Unit:** `STEPS` (for ESPHome stepper compatibility)

**Accepted Values:**
- **Steps:** `steps`, `step` → `STEPS`
  - Example: `3200 steps`
- **Revolutions:** `rev`, `revolutions` → `REVOLUTIONS`
  - Example: `2.5 rev`
- **Degrees:** `deg`, `degrees`, `°` → `DEGREES`
  - Example: `720 deg`
- **Radians:** `rad`, `radians` → `RADIANS`
  - Example: `6.28 rad`
- **Metric prefixes:** `k` (kilo), `M` (mega), `m` (milli), `µ/u` (micro)

> [!NOTE]
> Unit aliases (e.g., `rev`, `revolutions`) are mapped to the same enum value during validation. The `unit` field in dicts is **not templatable** and must be a compile-time constant.

> [!NOTE]
> Metric prefixes may only be used when the value is a plain number or string, not if its a lambda.

**Validation:**
- Use `cv.float_with_unit()` for metric prefix support
- Map unit string aliases to enum values
- Return dict: `{"value": float, "unit": enum_string}`

**Code Generation:**

Static values → Convert to steps at build-time:
- `STEPS`: `steps = value`
- `REVOLUTIONS`: `steps = value * steps_per_revolution`
- `DEGREES`: `steps = (value / 360.0) * steps_per_revolution`
- `RADIANS`: `steps = (value / (2π)) * steps_per_revolution`

Lambdas → Pass to C++ for runtime conversion:
- `value`: Lambda code (templatable)
- `unit`: Enum integer

**Unit Enum (C++):**
```cpp
enum class PositionUnit : uint8_t {
    STEPS,         // Default - ESPHome stepper compatibility
    REVOLUTIONS,
    DEGREES,
    RADIANS
};
```

**C++ Type:** `int32_t` (steps, signed for bidirectional positioning)

**References:**
- Uses [`cv.float_with_unit()`](https://github.com/esphome/esphome/blob/dev/esphome/config_validation.py) internally per unit



### `auto_sleep` Type

**Description:** Automatic motor power-down after idle time. Accepts boolean for enable/disable or time period for delayed shutdown.

**Accepted Values:**
- `false` or `inf` - Feature disabled, motor stays powered indefinitely
- `true` or `0s` - Disable motor immediately when idle
- Time period - Disable motor after specified idle time (e.g., `5s`, `30s`, `2min`)
  - Range: `1ms` to `4294967294ms` (~49.7 days)

**Validation:**
- Accept `false`/`inf` → return `UINT32_MAX` (disabled)
- Accept `true`/`0s` → return `0` (immediate)
- Accept time period → use `cv.positive_time_period_milliseconds`, clamp to `UINT32_MAX - 1`

**C++ Type:** `uint32_t` (milliseconds)

**C++ Semantics:**
- `UINT32_MAX` (4294967295) = Feature disabled (`false`/`inf`)
- `0` = Immediately disable (`true`/`0s`)
- `1` to `4294967294` = Delay in milliseconds

**Example Usage in C++:**
```cpp
static constexpr uint32_t AUTO_SLEEP_DISABLED = UINT32_MAX;

void on_idle() {  // Called when motor becomes idle (e.g., movement complete)
    if (auto_sleep_delay_ms_ == AUTO_SLEEP_DISABLED) {
        // false/inf: Feature disabled, motor stays powered
        return;
    } else if (auto_sleep_delay_ms_ == 0) {
        // true/0ms: Disable immediately
        this->disable_motor();
    } else {
        // 1+ms: Start delay timer
        this->set_timeout("auto_sleep", auto_sleep_delay_ms_, [this]() {
            this->disable_motor();
        });
    }
}
```

**YAML Examples:**
```yaml
auto_sleep: false      # Never disable (default)
auto_sleep: inf        # Never disable (alternative syntax)
auto_sleep: true       # Disable immediately (0ms)
auto_sleep: 0s         # Disable immediately (explicit)
auto_sleep: 30s        # Disable after 30 seconds
auto_sleep: 5min       # Disable after 5 minutes
```

**References:**
- [`cv.positive_time_period_milliseconds`](https://github.com/esphome/esphome/blob/dev/esphome/config_validation.py#L854) from ESPHome core
- [`cv.Any`](https://github.com/esphome/esphome/blob/dev/esphome/config_validation.py) for multi-type validation



### `current` Type

**Description:** Motor current with unit support. Converted to milliamperes (mA) for motor communication.

**Accepted Values:**
- **Amperes:** `A` - Current in amperes (e.g., `1.5A`, `2.5A`)
- **Milliamperes:** `mA` - Current in milliamperes (e.g., `1500mA`, `2500mA`)
- **Plain number:** Interpreted as milliamperes (e.g., `1500` = `1500mA`)
- **Metric prefixes:** Supported via ESPHome's current validator

**Validation:**
- Use ESPHome's `cv.current` (handles A/mA/plain numbers automatically)
- Convert to milliamperes: `current_ma = current_amps * 1000.0`
- Enforce servo_type limits (see defaults/maximums above)

**C++ Type:** `uint16_t` (milliamperes, 0-65535 mA)

**YAML Examples:**
```yaml
working_current: 1.5A      # Amperes
working_current: 1500mA    # Milliamperes
working_current: 1500      # Plain number (interpreted as mA)
working_current: 2.5A      # With decimal
```

**References:**
- [`cv.current`](https://github.com/esphome/esphome/blob/dev/esphome/config_validation.py) from ESPHome core
- Handles amperes (A), milliamperes (mA), and plain numbers automatically



### `direction` Type

**Description:** Rotational direction used where a concrete rotation sense is required.

**Accepted Values:**
- `CW` – Clockwise (positive direction)
- `CCW` – Counter-clockwise (negative direction)

**Validation:**

Use `cv.one_of("CW", "CCW", lower=True)`

**C++ Type:** enum (component-internal)



### `homing_direction` Type

**Description:** Direction for homing operations. Extends [`direction`](#direction-type) with `NEAREST` for virtual homing.

**Accepted Values:**
- `NEAREST` – Shortest path to zero
- All values from [`direction`](#direction-type)


> [!IMPORTANT]
> `NEAREST` is only valid when `homing.mode` is `VIRTUAL`. Validation must check this context.

**Validation:**

Use `cv.one_of("CW", "CCW", "NEAREST", lower=True)`

Additional context check: If `NEAREST`, require `homing.mode == "VIRTUAL"`

**C++ Type:** enum (component-internal)



### `zeroing_speed` Type

**Description:** Speed levels for virtual homing (0_Mode/No_Limit return-to-zero). Only used when `homing.mode` is `VIRTUAL`.

**Accepted Values:**
- `VERY_SLOW` – Slowest speed level (0)
- `SLOW` – Slow speed level (1)
- `MEDIUM` – Medium speed level (2)
- `FAST` – Fast speed level (3)
- `VERY_FAST` – Fastest speed level (4)

> [!NOTE]
> The controller firmware quantizes these into five discrete speed levels (0–4). The actual RPM values are hardware-dependent and not user-configurable.

**Validation:**

Use `cv.one_of("VERY_SLOW", "SLOW", "MEDIUM", "FAST", "VERY_FAST", lower=True)`

**C++ Type:** enum (component-internal, maps to firmware speed levels 0–4)

**C++ Implementation Example:**
```cpp
enum class ZeroingSpeed : uint8_t {
    VERY_SLOW = 0,
    SLOW = 1,
    MEDIUM = 2,
    FAST = 3,
    VERY_FAST = 4
};
```


### `templatable_with_unit` Type

**Description:** Generic type for templatable values with optional unit specification. Used for actions where the value can be either a number or a lambda, but the unit is always a static enum (e.g., for position, speed, acceleration).

**Motivation:** ESPHome's standard `stepper` component calculates positions in steps, which requires users to think in motor steps rather than the actual physical units they care about (revolutions, degrees). This component internally uses angles and revolutions for calculations. The `templatable_with_unit` type allows users to specify values in their preferred unit while maintaining compatibility with ESPHome's stepper interface. When no unit is specified, the value falls back to the standard stepper unit (steps) to ensure compatibility with existing stepper configurations and automations.

**Accepted Formats:**
1. **Plain number**: Default unit is used
   ```yaml
   parameter: 1000
   ```
2. **String with unit**: Parsed at compile-time
   ```yaml
   parameter: "60 RPM"
   parameter: "5.5 revolutions"
   ```
3. **Dict with explicit value and unit**:
   ```yaml
   parameter:
     value: 60        # Number or lambda
     unit: RPM        # Unit parsed at compile-time
   ```
4. **Lambda without unit** (uses default unit):
   ```yaml
   parameter: !lambda "return id(sensor).state;"
   ```
5. **Dict with lambda and unit**:
   ```yaml
   parameter:
     value: !lambda "return id(sensor).state;"
     unit: RPM
   ```

**Validation:**

Handle 5 formats:
1. Plain number → `{"value": float, "unit": default_unit}`
2. String with unit → Parse and return `{"value": float, "unit": enum_string}`
3. Dict with value/unit → Validate with `cv.templatable(cv.float_)` and `cv.enum()`
4. Lambda without unit → `{"value": lambda, "unit": default_unit}`
5. Dict with lambda and unit → Same as 3

Returns: `{"value": templatable, "unit": enum_string}`

**Usage:**
- Actions for position, speed, acceleration, etc. (e.g., `set_speed`, `set_acceleration`, `move_to`)

**C++ API Pattern:**
```cpp
void action(float value, UnitEnum unit = UnitEnum::DEFAULT);
```

**References:**
- [`cv.templatable()`](https://github.com/esphome/esphome/blob/dev/esphome/config_validation.py) – Validates static values, passes lambdas unchanged
- Conversion uses `steps_per_revolution` from component configuration


## Configuration

This section defines all configuration fields for the component, organized by scope. Possible configurations depend on the selected operating mode.

### Basic Configuration

Fields common to both Speed Mode and Position Mode.

#### `id`

Component instance identifier for referencing in actions and automations.

- **Type:** `ID`
- **Required:** ✅ Yes
- **Validation:** [`cv.declare_id(Servo42dRs485)`](https://github.com/esphome/esphome/blob/dev/esphome/config_validation.py)

**Example:**
```yaml
stepper:
  - platform: servo42d_rs485
    id: my_stepper
```

#### `modbus_id`

Reference to the Modbus controller this motor is connected to.

- **Type:** `ID`
- **Required:** ❌ Optional (only needed with multiple Modbus buses)
- **Default:** Auto-detected single bus
- **Validation:** [`cv.use_id(Modbus)`](https://github.com/esphome/esphome/blob/dev/esphome/config_validation.py)

**Example:**
```yaml
modbus:
  - id: modbus1
    uart_id: uart_bus

stepper:
  - platform: servo42d_rs485
    id: motor1
    modbus_id: modbus1
```

#### `address`

Modbus RTU slave address of the motor.

- **Type:** [`modbus_address`](#modbus_address-type)
- **Required:** ❌ Optional
- **Default:** `0x01`
- **Range:** `0x01` to `0xF7` (1-247 decimal)

**Example:**
```yaml
stepper:
  - platform: servo42d_rs485
    address: 0x10  # Decimal 16
```



#### `steps_per_revolution`

Number of steps required for one complete 360° rotation.

- **Type:** `float`
- **Required:** ✅ Yes
- **Validation:** [`cv.positive_float`](https://github.com/esphome/esphome/blob/dev/esphome/config_validation.py)

> [!CRITICAL]
> **This value is ESSENTIAL for all unit conversions!**
> 
> Used to convert between `steps`, `RPM`, and `degrees` in speed/acceleration/position.
> 
> **Calculation:**
> ```
> steps_per_revolution = base_steps * microsteps
> ```
> 
> **Examples:**
> - 1.8° motor (200 steps/rev) with 16 microsteps: `3200`
> - 0.9° motor (400 steps/rev) with 32 microsteps: `12800`

**Example:**
```yaml
stepper:
  - platform: servo42d_rs485
    steps_per_revolution: 3200  # 200 * 16
    microsteps: 16
```

#### `microsteps`

Microstepping subdivision value.

- **Type:** `uint16`
- **Required:** ❌ Optional
- **Default:** `16`
- **Range:** `1` to `256`
- **Validation:** `cv.int_range(min=1, max=256)`

> [!WARNING]
> **Speed Calibration Limitation**
> 
> Motor speed is factory-calibrated only for microstepping values **16, 32, and 64**.
> 
> For other values, apply this correction in your code:
> ```python
> actual_rpm = commanded_rpm * (microsteps / 16.0)
> ```

**Example:**
```yaml
stepper:
  - platform: servo42d_rs485
    microsteps: 32  # Calibrated value (no correction needed)
```

#### `max_speed`

Maximum allowed motor speed (hard ceiling limit).

- **Type:** [`speed`](#speed-type)
- **Required:** ❌ Optional
- **Default:** Depends on `control_mode`:
  - `SR_OPEN`: `400 RPM`
  - `SR_CLOSE`: `1500 RPM`
  - `SR_VFOC`: `3000 RPM`

> [!IMPORTANT]
> **Semantic Difference from ESPHome Stepper**
> 
> - **ESPHome Stepper:** `max_speed` = target speed after acceleration
> - **servo42d:** `max_speed` = absolute ceiling (hard limit), use `initial_speed` for target
> 
> The motor will **NEVER exceed** this speed, even if commanded higher.

**Examples:**
```yaml
stepper:
  - platform: servo42d_rs485
    max_speed: 100 RPM          # RPM
    
  - platform: servo42d_rs485
    max_speed: 6000 steps/s     # Steps per second
    
  - platform: servo42d_rs485
    max_speed: 360k steps/min   # With metric prefix
```

#### `initial_speed`

Default speed used when actions don't specify a speed parameter.

- **Type:** [`speed`](#speed-type)
- **Required:** ❌ Optional
- **Default:** `1 RPM`
- **Constraint:** Must be ≤ `max_speed`

**Examples:**
```yaml
stepper:
  - platform: servo42d_rs485
    initial_speed: 50 RPM
    max_speed: 100 RPM
    
  - platform: servo42d_rs485
    initial_speed: 3000 steps/s
    max_speed: 6000 steps/s
```

  #### `initial_acceleration`

Motor acceleration and deceleration rate.

- **Type:** [`acceleration`](#acceleration-type)
- **Required:** ❌ Optional
- **Default:** `inf` (infinite - instant acceleration)

> [!NOTE]
> Unlike the [ESPHome Stepper Component](https://esphome.io/components/stepper/), this component does not support separate acceleration and deceleration values. Both use the same internal value.

**Examples:**
```yaml
stepper:
  - platform: servo42d_rs485
     initial_acceleration: 100 RPM/s     # RPM per second
    
  - platform: servo42d_rs485
     initial_acceleration: 6k RPM/min    # With metric prefix
    
  - platform: servo42d_rs485
     initial_acceleration: 1000 steps/s² # Steps per second squared
```

#### `sleep_when_done`

Motor power-down behavior after movement completion.

- **Type:** [`auto_sleep`](#auto_sleep-type)
- **Required:** ❌ Optional
- **Default:** `false` or `inf` (motor stays powered)

**Examples:**
```yaml
stepper:
  - platform: servo42d_rs485
    sleep_when_done: false      # Always hold position
    
  - platform: servo42d_rs485
    sleep_when_done: true       # Power off immediately
    
  - platform: servo42d_rs485
    sleep_when_done: 30s        # Power off after 30 seconds
```

**See:** [`auto_sleep` type definition](#auto_sleep-type) for detailed validation and C++ semantics.



#### `servo_type`

Physical motor model type.

- **Type:** `enum`
- **Required:** ✅ Yes
- **Values:**
  - `SERVO28D` - NEMA11 size (default: 0.6A, max: 3.0A)
  - `SERVO35D` - NEMA14 size (default: 0.8A, max: 3.0A)
  - `SERVO42D` - NEMA17 size (default: 1.6A, max: 3.0A)
  - `SERVO57D` - NEMA23 size (default: 3.2A, max: 5.2A)

> [!IMPORTANT]
> **Must match your physical motor model!**
> 
> This setting cannot be auto-detected and determines:
> - Default `working_current` if not explicitly set
> - Maximum allowed `working_current` (validation enforced)
> - Sensorless homing current defaults
> 
> Setting the wrong type may damage the motor due to incorrect current limits.

**Example:**
```yaml
stepper:
  - platform: servo42d_rs485
    servo_type: SERVO42D     # Required!
    working_current: 1.6A    # Optional: uses default if omitted
```

#### `control_mode`

Motor control algorithm selection.

- **Type:** `enum`
- **Required:** ❌ Optional
- **Default:** `SR_VFOC`
- **Values:**
  - `SR_OPEN` - Open-loop mode, stepper behaves like a regular stepper motor. Working current is `working_current`, holding current is `holding_current_percent` of working current.
  - `SR_CLOSE` - Closed-loop mode, same as `SR_OPEN` but with position feedback from encoder to prevent missed steps.
  - `SR_VFOC` - FOC mode (recommended), same as `SR_CLOSE` but current may be adapted to the stepper's needs up to the max `working_current`. `holding_current_percent` is ignored in this mode.

> [!NOTE]
> Hardware speed limits depend on control mode:
> - `SR_OPEN`: 400 RPM
> - `SR_CLOSE`: 1500 RPM  
> - `SR_VFOC`: 3000 RPM
> 
> These limits are automatically applied to `max_speed` if not explicitly set.

**Example:**
```yaml
stepper:
  - platform: servo42d_rs485
    control_mode: SR_VFOC  # FOC mode (recommended)
```

#### `working_current`

Motor current during movement.

- **Type:** [`current`](#current-type)
- **Required:** ❌ Optional
- **Default:** Depends on `servo_type`:
  - `SERVO28D`: `600mA` (0.6A)
  - `SERVO35D`: `800mA` (0.8A)
  - `SERVO42D`: `1600mA` (1.6A)
  - `SERVO57D`: `3200mA` (3.2A)
- **Maximum:** Depends on `servo_type`:
  - `SERVO28D`: `3000mA` (3.0A)
  - `SERVO35D`: `3000mA` (3.0A)
  - `SERVO42D`: `3000mA` (3.0A)
  - `SERVO57D`: `5200mA` (5.2A)

> [!WARNING]
> Exceeding the maximum current for your servo type will be rejected during validation.

> [!NOTE]
> **Control Mode Behavior**
> 
> - `SR_OPEN` / `SR_CLOSE`: Fixed current at this exact value during movement
> - `SR_VFOC`: Maximum allowed current - actual current may be lower, adapted automatically to motor needs

**Examples:**
```yaml
stepper:
  - platform: servo42d_rs485
    servo_type: SERVO42D
    control_mode: SR_OPEN
    working_current: 2.5A       # Fixed 2.5A during movement
    
  - platform: servo42d_rs485
    servo_type: SERVO57D
    control_mode: SR_VFOC
    working_current: 4500mA     # Max 4.5A, may use less
    
  - platform: servo42d_rs485
    servo_type: SERVO42D
    # working_current omitted - uses default 1.6A
```

#### `holding_current_percent`

Current applied when motor is stationary (as percentage of `working_current`).

- **Type:** `percent`
- **Required:** ❌ Optional
- **Default:** `50%`
- **Range:** `0%` to `100%`
- **Validation:** [`cv.percentage`](https://github.com/esphome/esphome/blob/dev/esphome/config_validation.py)

> [!NOTE]
> **Control Mode Behavior**
> 
> - `SR_OPEN` / `SR_CLOSE`: Holding current = `working_current` × `holding_current_percent`
> - `SR_VFOC`: This setting is **ignored** - FOC mode manages holding current automatically

**Examples:**
```yaml
stepper:
  - platform: servo42d_rs485
    control_mode: SR_CLOSE
    working_current: 2.5A
    holding_current_percent: 30%  # 0.75A when stopped
    
  - platform: servo42d_rs485
    control_mode: SR_VFOC
    working_current: 2.5A
    holding_current_percent: 30%  # Ignored in FOC mode
```

#### `en_pin_active`

Enable pin logic level polarity.

- **Type:** `enum`
- **Required:** ❌ Optional
- **Default:** `ALWAYS`
- **Values:**
  - `LOW` - Motor enabled when pin is LOW
  - `HIGH` - Motor enabled when pin is HIGH
  - `ALWAYS` - Motor always enabled (ignore EN pin)

**Example:**
```yaml
stepper:
  - platform: servo42d_rs485
    en_pin_active: ALWAYS  # Ignore EN pin (default)
    
  - platform: servo42d_rs485
    en_pin_active: LOW     # Enable when pin is LOW
```



#### `auto_screen_off`

Automatically turn off motor's built-in display after timeout.

- **Type:** `bool`
- **Required:** ❌ Optional
- **Default:** `true`

**Example:**
```yaml
stepper:
  - platform: servo42d_rs485
    auto_screen_off: true   # Display turns off after 15 seconds (default)
    
  - platform: servo42d_rs485
    auto_screen_off: false  # Display stays on
```

#### `lock_keys_at_startup`

Lock physical buttons on the motor at power-up.

- **Type:** `bool`
- **Required:** ❌ Optional
- **Default:** `false`

**Example:**
```yaml
stepper:
  - platform: servo42d_rs485
    lock_keys_at_startup: true  # Prevent manual control
```



#### `mode`

Operating mode determines available actions and behavior.

- **Type:** `enum`
- **Required:** ❌ Optional
- **Default:** `POSITION`
- **Values:**
  - `POSITION` - Position control mode (stepper behavior)
  - `SPEED` - Continuous rotation mode (velocity control)

> [!IMPORTANT]
> **This setting fundamentally changes how the motor operates!**
> 
> **Component Behavior Changes:**
> - Motor control strategy (position tracking vs. continuous velocity)
> - Internal state management (target position vs. current speed/direction)
> - Communication protocol with motor controller
> 
> **User-Facing Changes:**
> - **Available Configuration:** Mode-specific fields (e.g., `homing` only in position mode)
> - **Available Actions:** Different action sets per mode (see below)
> - **Motor Response:** Position tracking vs. continuous rotation

**Examples:**
```yaml
# Position Mode - for applications requiring precise positioning
stepper:
  - platform: servo42d_rs485
    id: camera_slider
    mode: POSITION
    steps_per_revolution: 3200
    initial_speed: 500 steps/s
    initial_acceleration: 200 steps/s²
    homing:
      mode: ENDSTOP
      at_startup: true

# Speed Mode - for applications requiring continuous rotation
stepper:
  - platform: servo42d_rs485
    id: conveyor_motor
    mode: SPEED
    steps_per_revolution: 3200
    initial_speed: 300 RPM
    initial_acceleration: 100 RPM/s
```
### Position Mode Configuration

Additional fields available only when [mode](#mode) is `POSITION`.

#### `homing`

Contains homing configuration. Optional but required for homing functionality.

- **Type:** `map`
- **Required:** ❌ Optional
- **Fields:** See below for sub-fields

#### `homing.mode`

Homing method selection. This setting unifies three firmware concepts into one simple option: (1) real homing with an endstop (ENDSTOP), (2) sensorless homing using stall detection (SENSORLESS), and (3) the controller's 0_Mode/No_Limit return-to-zero without an endstop (VIRTUAL). The following values map to the motor's internal behaviors.

- **Type:** `enum`
- **Required:** ✅ Yes
- **Values:**
  - `ENDSTOP` - Use physical endstop switch
  - `SENSORLESS` - Use steppers encoder to detect running into end (stallguard)
  - `VIRTUAL` - Software-defined home position (no physical detection)

Detailed behavior and guidance for each mode:

- ENDSTOP
  - Requirements: A physical endstop switch is wired and configured; set `homing.endstop_trigger` accordingly.
  - Behavior: Moves in `homing.direction` at `homing.speed` until the endstop triggers, then stops and sets the logical zero position. This is the most robust and repeatable option.

- SENSORLESS
  - Requirements: No switch needed. You must provide a suitable `homing.current` threshold that matches your mechanics and motor (defaults are provided per `servo_type`).
  - Behavior: Moves in `homing.direction` at `homing.speed` until the motor current exceeds `homing.current` (stall detected), then stops and sets zero. Sensitivity depends on friction/load and may need tuning.

- VIRTUAL
  - Requirements: No physical detection. Relies on the controller’s internal "return to zero" routine (0_Mode/No_Limit). Best used when a known zero was set previously and mechanics allow a safe return without hard stops.
  - Behavior: Moves to the controller’s stored zero without reading an endstop or stall. Intended for quick reference moves or after a known alignment. Does not detect or learn mechanical limits.

**Example:**
```yaml
stepper:
  - platform: servo42d_rs485
    mode: POSITION
    homing:
      mode: ENDSTOP
```

#### `homing.at_startup`

Automatically perform homing sequence during component initialization.

- **Type:** `bool`
- **Required:** ❌ Optional
- **Default:** `false`

**Example:**
```yaml
stepper:
  - platform: servo42d_rs485
    mode: POSITION
    homing:
      mode: ENDSTOP
      at_startup: true  # Home immediately on boot
```

#### `homing.direction`

Direction to move during homing sequence.

- **Type:** [`homing_direction`](#homing_direction-type)
- **Required:** ❌ Optional
- **Default:** `CW`

**Example:**
```yaml
stepper:
  - platform: servo42d_rs485
    mode: POSITION
    homing:
      direction: CCW  # Move toward endstop in negative direction
```

#### `homing.speed`

Speed used during homing movement. Accepted type depends on `homing.mode`:

**When `homing.mode: VIRTUAL`:**
- **Type:** [`zeroing_speed`](#zeroing_speed-type)
- **Default:** `MEDIUM` (level 2)

> [!NOTE]
> The controller firmware only supports five discrete speed levels for virtual homing. These map to internal speed settings (0–4) and cannot be specified as RPM or steps/s.

**In all other cases (`ENDSTOP`, `SENSORLESS`):**
- **Type:** [`speed`](#speed-type)
- **Default:** `1 RPM`

**Examples:**
```yaml
# VIRTUAL mode - use discrete levels
stepper:
  - platform: servo42d_rs485
    mode: POSITION
    homing:
      mode: VIRTUAL
      speed: FAST

# ENDSTOP/SENSORLESS mode - use standard speed units
stepper:
  - platform: servo42d_rs485
    mode: POSITION
    homing:
      mode: ENDSTOP
      speed: 50 RPM  # Slow for accuracy
```

#### `homing.endstop_trigger`

Endstop switch trigger logic level.

- **Type:** `enum`
- **Required:** ⚠️ Required if `mode: ENDSTOP`
- **Values:**
  - `HIGH` - Endstop triggers when signal goes HIGH
  - `LOW` - Endstop triggers when signal goes LOW

**Example:**
```yaml
stepper:
  - platform: servo42d_rs485
    mode: POSITION
    homing:
      mode: ENDSTOP
      endstop_trigger: LOW  # Normally-open switch
```

#### `homing.current`

Current threshold for sensorless homing (stallguard detection).

- **Type:** [`current`](#current-type)
- **Required:** ❌ Optional
- **Default:** Depends on `servo_type`:
  - `SERVO28D`: `600mA` (0.6A)
  - `SERVO35D`: `800mA` (0.8A)
  - `SERVO42D`: `1600mA` (1.6A)
  - `SERVO57D`: `3200mA` (3.2A)

> [!NOTE]
> **Only effective when `homing.mode: SENSORLESS`**
> 
> Motor moves in `homing.direction` until current exceeds this threshold, indicating a physical obstruction (endstop/hard stop). Higher values = more force before detection, lower values = more sensitive but may trigger prematurely.
> 
> This setting is ignored in `ENDSTOP` and `VIRTUAL` modes.

**Examples:**
```yaml
stepper:
  - platform: servo42d_rs485
    servo_type: SERVO42D
    mode: POSITION
    homing:
      mode: SENSORLESS
      current: 1.5A          # Trigger at 1.5A
  direction: CCW
      
  - platform: servo42d_rs485
    servo_type: SERVO57D
    mode: POSITION
    homing:
      mode: SENSORLESS
      current: 2500mA        # Trigger at 2.5A
      # current omitted - would use default 3.2A
  direction: CCW
```

### Speed Mode Configuration

Additional fields available only when [mode](#mode) is `SPEED`.

> [!WARNING]
> **Default Behavior on Startup**
>
> With all defaults, the motor will immediately start rotating at `1 RPM` in clockwise direction when the component initializes. This is determined by:
> - `initial_speed`: `1 RPM` (from [Basic Configuration](#initial_speed))
>
> If you need the motor to remain stationary at startup, explicitly set `initial_speed: 0 RPM` or use an action to control when movement begins.



## Actions

Actions are organized by their availability in different operating modes. Some actions work in all modes, while others are specific to Position Mode or Speed Mode.

### Basic Actions

Actions available in both operating modes.

#### `stepper.enable` / `stepper.disable`

Enable or disable the motor. Same action is used for the `sleep_when_done` configuration.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.

**C++ API:**
```cpp
void enable();
void disable();
```

**Examples:**
```yaml
on_...:
  - stepper.enable: my_stepper
  - stepper.disable: my_stepper
```



#### `stepper.emergency_stop`

Emergency stop - immediately halt motor with maximum deceleration.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.

**C++ API:**
```cpp
void emergency_stop();
```

> [!WARNING]
> At speeds above about 1000 RPM, this can be mechanically harsh. Use regular [`stepper.stop`](#stepperstop) with controlled deceleration when possible.

> [!NOTE]
> Same as [`stepper.stop`](#stepperstop), but with deceleration set to `inf` (instant stop). The stepper will also be disabled after stopping, and acceleration cannot be changed. [`stepper.release_protection`](#stepperrelease_protection) may be called to re-enable normal operation after an emergency stop.

**Example:**
```yaml
on_...:
  - stepper.emergency_stop: my_stepper
```



#### `stepper.calibrate`

Start motor calibration sequence. Used to map measured magnetic field to encoder positions. Should be done at least once after motor installation.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.

**C++ API:**
```cpp
void calibrate();
```
> [!NOTE]
> Motor will move during calibration. Make sure that the stepper moves freely and is not obstructed. Stepper will restart after calibration.

**Example:**
```yaml
on_...:
  - stepper.calibrate: my_stepper
```

#### `stepper.release_protection`

Release motor protection state after error condition. Is part of `stepper.home` action. Most of the cases that is the right action to recover from an error.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.

**C++ API:**
```cpp
void release_protection();
```

**Example:**
```yaml
on_...:
  - stepper.release_protection: my_stepper
```



#### `stepper.restart`

Restart the motor controller. Part of initial setup, and is also called when `homing.mode: VIRTUAL` is used, `homing.at_startup: true` is set and `stepper.set_zero` was at least once called before.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.

**C++ API:**
```cpp
void restart();
```

**Example:**
```yaml
on_...:
  - stepper.restart: my_stepper
```



#### `stepper.set_work_mode`

Change the motor control mode at runtime.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **mode** (**Required**, enum): Work mode, one of `SR_OPEN`, `SR_CLOSE`, `SR_VFOC`.

**C++ API:**
```cpp
void set_work_mode(WorkMode mode);
```

**Example:**
```yaml
on_...:
  - stepper.set_work_mode:
      id: my_stepper
      mode: SR_VFOC
```



#### `stepper.set_working_current`

Change the working current at runtime.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **current** (**Required**, [`current`](#current-type)): Current in mA. May be smaller than [`max` of `working_current`](#working_current).

**C++ API:**
```cpp
void set_working_current(float current_milliamps);
```

**Example:**
```yaml
on_...:
  - stepper.set_working_current:
      id: my_stepper
      current: 2000  # mA
```



#### `stepper.set_holding_current_percent`

Change the holding current percentage at runtime. Only works in `SR_OPEN` and `SR_CLOSE` modes.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **percent** (**Required**, [percent](https://esphome.io/guides/configuration-types.html#config-percentage)): Percentage of working current (10-90).

**C++ API:**
```cpp
void set_holding_current_percent(uint8_t percent);
```

**Example:**
```yaml
on_...:
  - stepper.set_holding_current_percent:
      id: my_stepper
      percent: 40  # 10-90%
```



#### `stepper.set_microstepping`

Change microstepping (step mode) at runtime. `steps_per_revolution` is automatically adjusted accordingly.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **subdivision** (**Required**, uint16): Microstepping / step mode (`1-256`), e.g., `1`=full, `2`=half, `4`=quarter.

**C++ API:**
```cpp
void set_microstepping(uint16_t subdivision);
```

**Example:**
```yaml
on_...:
  - stepper.set_microstepping:
      id: my_stepper
      subdivision: 32  # 1-256
```



#### `stepper.set_speed`

Set the maximum speed of the stepper at runtime.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **speed** (**Required**, [`templatable_with_unit`](#templatable_with_unit-type) as [`speed`](#speed-type)): The speed to drive the stepper at. Supports units like `steps/s`, `RPM`, `rev/s`, `deg/s`, `rad/s`. Value can be a number, string with unit, or lambda.

**C++ API:**
```cpp
struct Speed {
    float value;
    SpeedUnit unit;
};

void set_speed(Speed speed);
```

**Examples:**
```yaml
on_...:
  # Plain number with unit
  - stepper.set_speed:
      id: my_stepper
      speed: 250 steps/s
  
  # Lambda with unit
  - stepper.set_speed:
      id: my_stepper
      speed:
        value: !lambda "return id(speed_sensor).state;"
        unit: RPM
```



#### `stepper.set_acceleration`

Set the acceleration of the stepper at runtime.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **acceleration** (**Required**, [`templatable_with_unit`](#templatable_with_unit-type) as [`acceleration`](#acceleration-type)): The acceleration to use when starting to move. Supports units like `steps/s²`, `RPM/s`, `rev/s²`, `deg/s²`, `rad/s²`. Value can be a number, string with unit, or lambda.

**C++ API:**
```cpp
struct Acceleration {
    float value;
    AccelerationUnit unit;
};

void set_acceleration(Acceleration acceleration);
```

> [!NOTE]
> Unlike the [ESPHome Stepper Component](https://esphome.io/components/stepper/), this component does not support separate acceleration and deceleration values. Both use the same internal value, so calling either action will affect both acceleration and deceleration.

**Examples:**
```yaml
on_...:
  # Plain number with unit
  - stepper.set_acceleration:
      id: my_stepper
      acceleration: 250 steps/s²
  
  # Lambda with unit
  - stepper.set_acceleration:
      id: my_stepper
      acceleration:
        value: !lambda "return id(accel_sensor).state;"
        unit: RPM/s
```



#### `stepper.stop`

Stop the current motor movement. Available in both Position and Speed modes.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **acceleration** (*Optional*, [`templatable_with_unit`](#templatable_with_unit-type) as [`acceleration`](#acceleration-type)): Deceleration to use when stopping the motor. Supports units like `steps/s²`, `RPM/s`, `rev/s²`, `deg/s²`, `rad/s²`. Value can be a number, string with unit, or lambda.

**C++ API:**
```cpp
struct Acceleration {
    float value;
    AccelerationUnit unit;
};

void stop(optional<Acceleration> deceleration);
```

> [!NOTE]
> `acceleration` will be changed by `stepper.stop` and keeps its last-used value. If a value was never set before, the component default (from `initial_acceleration`) is used.

> [!WARNING]
> At speeds above about 1000 RPM, avoid stopping too abruptly. Use a non-zero `acceleration` (deceleration) for smoother, safer stops to protect mechanics and couplings.

**Examples:**
```yaml
on_...:
  # Plain number with unit
  - stepper.stop:
      id: my_stepper
      acceleration: 500 steps/s²
  
  # Lambda with unit
  - stepper.stop:
      id: my_stepper
      acceleration:
        value: !lambda "return id(decel_sensor).state;"
        unit: RPM/s
```



#### `stepper.key_lock` / `stepper.key_unlock`

Lock or unlock the motor display buttons.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.

**C++ API:**
```cpp
void key_lock();
void key_unlock();
```

**Examples:**
```yaml
on_...:
  - stepper.key_lock: my_stepper
  - stepper.key_unlock: my_stepper
```



### Position Mode Actions

Actions available only when [`mode`](#mode) is `POSITION`.

#### `stepper.set_target`

Set the target position of the motor. The stepper will move towards the target position and stop once reached.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **target** (**Required**, [`templatable_with_unit`](#templatable_with_unit-type) as [`position`](#position-type)): The target position. Supports units like `steps`, `revolutions`, `degrees`, `radians`. Value can be a number, string with unit, or lambda.

**C++ API:**
```cpp
struct Position {
    float value;
    PositionUnit unit;
};

void set_target(Position target);
```

**Examples:**
```yaml
on_...:
  # Plain number (uses default unit: steps)
  - stepper.set_target:
      id: my_stepper
      target: 3200
  
  # String with unit
  - stepper.set_target:
      id: my_stepper
      target: 2.5 rev
  
  # Dict format with unit
  - stepper.set_target:
      id: my_stepper
      target:
        value: 720
        unit: deg
  
  # Lambda (uses default unit: steps)
  - stepper.set_target:
      id: my_stepper
      target: !lambda "return id(target_sensor).state;"
  
  # Lambda with explicit unit
  - stepper.set_target:
      id: my_stepper
      target:
        value: !lambda "return id(target_sensor).state;"
        unit: rev
```


#### `stepper.report_position`

Report the current position to a specific value. Sets an offset for future movements. To store a position for virtual homing, use [`stepper.set_zero`](#stepperset_zero) instead.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **position** (**Required**, [`templatable_with_unit`](#templatable_with_unit-type) as [`position`](#position-type)): The position to report. Supports units like `steps`, `revolutions`, `degrees`, `radians`. Value can be a number, string with unit, or lambda.

**C++ API:**
```cpp
struct Position {
    float value;
    PositionUnit unit;
};

void report_position(Position position);
```

**Examples:**
```yaml
on_...:
  # Plain number (uses default unit: steps)
  - stepper.report_position:
      id: my_stepper
      position: 0
  
  # String with unit
  - stepper.report_position:
      id: my_stepper
      position: 1 rev
  
  # Dict format
  - stepper.report_position:
      id: my_stepper
      position:
        value: 90
        unit: deg
```



#### `stepper.home`

Execute homing sequence. Behavior depends on `homing.mode` configuration:
- `SENSORLESS`: Uses stall detection (sensorless homing)
- `ENDSTOP`: Uses endstop and GoHome command (real homing)
- `VIRTUAL`: Restarts motor to return to stored zero position

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.

**C++ API:**
```cpp
void home(bool no_restart = false);
```

**Example:**
```yaml
on_...:
  - stepper.home: my_stepper
```



#### `stepper.set_zero`

Store the current position as persistent zero point for virtual homing. This must be called once before using virtual homing. The value is stored within the motor controller and remains after power-cycles. May only be used when `homing.mode` is `VIRTUAL`.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.

**C++ API:**
```cpp
void set_zero();
```

**Example:**
```yaml
on_...:
  - stepper.set_zero: my_stepper
```



### Speed Mode Actions

Actions available only when [`mode`](#mode) is `SPEED`.

#### `stepper.run_continuous`

Run the motor continuously at specified speed. Used in speed mode and for continuous movements.

**Configuration:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **speed** (*Optional*, [`templatable_with_unit`](#templatable_with_unit-type) as [`speed`](#speed-type)): Target speed (signed; sign determines direction). Supports units like `steps/s`, `RPM`, `rev/s`, `deg/s`, `rad/s`. Value can be a number, string with unit, or lambda.
- **acceleration** (*Optional*, [`templatable_with_unit`](#templatable_with_unit-type) as [`acceleration`](#acceleration-type)): Acceleration for speed-mode change. Supports units like `steps/s²`, `RPM/s`, `rev/s²`, `deg/s²`, `rad/s²`. Value can be a number, string with unit, or lambda.

**C++ API:**
```cpp
struct Speed {
    float value;
    SpeedUnit unit;
};

struct Acceleration {
    float value;
    AccelerationUnit unit;
};

void run_continuous(optional<Speed> speed,
                    optional<Acceleration> acceleration);
```

> [!NOTE]
> At least one of `speed` or `acceleration` must be provided per call. Omitted values keep their last-used value. If a value was never set before, the component default (e.g., from `initial_speed` or `initial_acceleration`) is used. The sign of `speed` determines direction (positive => clockwise, negative => counter-clockwise).

**Examples:**
```yaml
on_...:
  # Set all parameters (clockwise)
  - stepper.run_continuous:
      id: my_stepper
      acceleration: 1000 steps/s²
      speed: 1000 steps/s
  
  # Change only speed (counter-clockwise)
  - stepper.run_continuous:
      id: my_stepper
    speed: -60 RPM
  
  # Use lambda for dynamic speed
  - stepper.run_continuous:
      id: my_stepper
      speed:
        value: !lambda "return id(speed_sensor).state;"
        unit: RPM
```
