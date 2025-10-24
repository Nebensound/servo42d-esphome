# Unit System Documentation (Developer Reference)

## Core Concepts

### Terminology

**STEPS = MICROSTEPS = PULSES**

In ESPHome stepper convention (and this component):
- **"steps"** always refers to **microsteps/pulses**, NOT full steps
- This is consistent across all ESPHome stepper components
- In C++ code, all variables use "steps"
  - Exception: Only the subdivision parameter uses "microsteps" terminology

**Motor Hardware:**
- NEMA17: 200 full steps/revolution (1.8° step angle)
- Microstep subdivision: 1, 2, 4, 8, 16, 32, 64, 128, 256
- Example: 200 full steps × 16 microsteps = **3200 steps/revolution**

**Encoder Hardware:**
- **Ticks**: Raw encoder counts from hardware (16384 ticks per revolution)
- Separate from steps - used for high-precision position feedback
- Conversion: `steps = (ticks × steps_per_revolution) / 16384`

**Unit Summary:**
- `steps` (ESPHome) = `microsteps` = `pulses` = `full_steps × microsteps`
- `ticks` = Raw encoder counts (16384 per revolution, hardware-specific)
- **Primary term:** "steps" (used in all C++ code and API)
- **Alternative terms:** "microsteps" and "pulses" (only for clarification in documentation)
- **Encoder term:** "ticks" (raw encoder feedback, separate concept)

**NO CONVERSION** occurs in the component for steps. Values are passed through directly.
**CONVERSION** happens between ticks (encoder) and steps (motor position).

### Configuration Requirements

```yaml
steps_per_revolution: 3200  # Total microsteps per revolution (full_steps × microsteps)
microsteps: 16              # Must match motor setting
```

**Critical:** Configuration values must match motor:
1. Component config: `microsteps: 16`
2. Motor setting: `MStep: 16` (via motor menu)
3. Component config: `steps_per_revolution: 3200` (200 full steps × 16 microsteps)

**Note:** `steps_per_revolution` represents the **total steps per revolution**, not full steps!

### Example Calculation

```
NEMA17 with 16 microsteps:
- Full steps (motor hardware) = 200
- Microsteps (driver subdivision) = 16
- steps_per_revolution = 200 × 16 = 3200 (config value!)
- set_target(100) = 100 steps = 11.25° rotation
- set_target(3200) = 3200 steps = 360° rotation (1 full revolution)
```

### Speed Units (max_speed, homing_speed, run_continuous)

| Unit | Aliases | Description | Conversion to steps | Requires Config |
|------|---------|-------------|---------------------|-----------------|
| `steps/s` | `step/s`, `pulses/s`, `pulse/s` | Steps per second (native) | `value` (no conversion) | Always available |
| `steps/min` | `step/min`, `pulses/min`, `pulse/min` | Steps per minute | `value / 60` | Always available |
| `RPM` | `rpm`, `revolutions/min`, `revolution/min` | Revolutions per minute | `value × steps_per_revolution / 60` | `steps_per_revolution` |
| `RPS` | `rps`, `revolutions/s`, `revolution/s` | Revolutions per second | `value × steps_per_revolution` | `steps_per_revolution` |
| `degrees/s` | `degree/s`, `deg/s`, `°/s` | Degrees per second | `value × steps_per_revolution / 360` | `steps_per_revolution` |
| `degrees/min` | `degree/min`, `deg/min`, `°/min` | Degrees per minute | `value × steps_per_revolution / 360 / 60` | `steps_per_revolution` |

**Note:** "steps" = microsteps (ESPHome standard terminology, used in all C++ code)

### Acceleration Units (acceleration, deceleration)

| Unit | Aliases | Description | Conversion to steps/s² | Requires Config |
|------|---------|-------------|------------------------|-----------------|
| `steps/s^2` | `steps/s*s`, `steps/s/s`, `steps/ss`, `steps/(s*s)` | Steps per second squared (native) | `value` (no conversion) | Always available |
| `steps/min^2` | `steps/min*min`, `steps/min/min`, `steps/minmin`, `steps/(min*min)` | Steps per minute squared | `value / 3600` | Always available |
| `revolutions/s^2` | `revolutions/s*s`, `revolutions/s/s`, `revolutions/ss`, `revolutions/(s*s)` | Revolutions per second squared | `value × steps_per_revolution` | `steps_per_revolution` |
| `revolutions/min^2` | `revolutions/min*min`, `revolutions/min/min`, `revolutions/minmin`, `revolutions/(min*min)` | Revolutions per minute squared | `value × steps_per_revolution / 3600` | `steps_per_revolution` |
| `degrees/s^2` | `degrees/s*s`, `degrees/s/s`, `degrees/ss`, `degrees/(s*s)`, `deg/s^2`, `°/s^2` | Degrees per second squared | `value × steps_per_revolution / 360` | `steps_per_revolution` |
| `degrees/min^2` | `degrees/min*min`, `degrees/min/min`, `degrees/minmin`, `degrees/(min*min)`, `deg/min^2`, `°/min^2` | Degrees per minute squared | `value × steps_per_revolution / 360 / 3600` | `steps_per_revolution` |

**Note:** "steps" = microsteps (ESPHome standard terminology, used in all C++ code)

### Position Units (set_target, report_position)

| Unit | Aliases | Description | Conversion to steps | Requires Config |
|------|---------|-------------|---------------------|-----------------|
| `steps` | `step`, `pulses`, `pulse` | Steps (native) | `value` (no conversion) | Always available |
| `revolutions` | `revolution`, `rev` | Full revolutions | `value × steps_per_revolution` | `steps_per_revolution` |
| `degrees` | `degree`, `deg`, `°` | Degrees | `value × steps_per_revolution / 360` | `steps_per_revolution` |

**Note:** "steps" = microsteps (ESPHome standard terminology, used in all C++ code)

### Encoder Units (Read-only, hardware feedback)

| Unit | Description | Value Range | API Function |
|------|-------------|-------------|-------------|
| `ticks` | Raw encoder counts | -∞ to +∞ (int64) | `get_encoder_ticks()` |
| `steps` | Converted to motor steps | -∞ to +∞ (int32) | `get_encoder_steps()` |
| `degrees` | Converted to degrees | 0.0-359.99... (float) | `get_encoder_degrees()` |
| `radians` | Converted to radians | 0.0-2π (float) | `get_encoder_radians()` |

**Note:** Encoder has 16384 ticks per revolution (hardware constant). Conversion to steps uses `steps_per_revolution` configuration.

### Angle Units (virtual_home_angle)

| Unit | Description | Range | Note |
|------|-------------|-------|------|
| `degrees` | Degrees (always, no suffix needed) | 0-359 | Always available, no conversion |

### Configuration Examples

```yaml
# Example 1: Direct microsteps
stepper:
  - platform: servo42d_rs485
    full_steps_per_revolution: 200
    microsteps: 16
    max_speed: 1600 steps/s        # 1600 microsteps/s
    acceleration: 800 steps/s^2

# Example 2: Full steps-based
stepper:
  - platform: servo42d_rs485
    full_steps_per_revolution: 200
    microsteps: 16
    max_speed: 100 full_steps/s    # = 1600 microsteps/s (100×16)
    acceleration: 50 full_steps/s^2 # = 800 microsteps/s^2 (50×16)

# Example 3: Revolution-based
stepper:
  - platform: servo42d_rs485
    full_steps_per_revolution: 200
    microsteps: 16
    max_speed: 2 revolutions/s     # = 6400 microsteps/s (200×16×2)
    acceleration: 1 revolutions/s^2 # = 3200 microsteps/s^2

# Example 4: Degree-based
stepper:
  - platform: servo42d_rs485
    full_steps_per_revolution: 200
    microsteps: 16
    max_speed: 90 degrees/s        # = 800 microsteps/s (3200÷360×90)
    acceleration: 180 degrees/s^2   # = 1600 microsteps/s^2
```

### Virtual Homing

`virtual_home_angle` always uses **degrees (0-359)**, regardless of unit system:

```yaml
use_virtual_home: true
virtual_home_angle: 180  # Always in degrees
homing_direction: NEAREST
```

## Common Configurations

### High-Speed (lower microsteps)
```yaml
full_steps_per_revolution: 200
microsteps: 8
max_speed: 20 revolutions/s  # = 32000 microsteps/s (200×8×20)
```

### High-Precision (higher microsteps)
```yaml
full_steps_per_revolution: 200
microsteps: 256
max_speed: 1 revolution/s    # = 51200 microsteps/s (200×256×1)
```

### Standard (balanced)
```yaml
full_steps_per_revolution: 200
microsteps: 16
max_speed: 2 revolutions/s   # = 6400 microsteps/s (200×16×2)
```
