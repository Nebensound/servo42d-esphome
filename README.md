# Servo42D RS485 Stepper Component

The `servo42d_rs485` stepper platform allows you to control MKS ServoXXD closed-loop stepper motors with RS485 communication via Modbus RTU.

```yaml
# Base setup shared by both profiles

stepper:
  - platform: servo42d_rs485
    id: my_stepper
    address: 0x01
    steps_per_revolution: 3200
    microsteps: 16
    control_mode: SR_VFOC
    servo_type: SERVO42D
    working_current: 1.6A
    holding_current_percent: 40%
    en_pin_active: ALWAYS
    auto_screen_off: false
    lock_keys_at_startup: false
    update_interval: 500ms
    mode: ... # position | speed
```
> [!NOTE]
> This component requires the Modbus component to be set up as well.


## Base Configuration

- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): Specify the ID of the stepper so that you can control it.
- **modbus_id** (*Optional*, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the Modbus controller. Only needed when you have multiple Modbus controllers.
- **address** (*Optional*, int): The Modbus device address. Defaults to `0x01`. Range: 1-247.
- **steps_per_revolution** (**Required**, float): The number of steps for one full rotation. Example: `3200` (200 steps × 16 microsteps).
  > [!TIP]
  > Set this precisely; wrong values will cause incorrect position and speed calculations.
- **microsteps** (*Optional*, int): Microstepping (aka step mode). Typical values: `1`=full, `2`=half, `4`=quarter, then `8`, `16`, `32`, … Range `1-256`. Defaults to `16`.

- **servo_type** (*Optional*, enum): Motor model used. One of `SERVO28D`, `SERVO35D`, `SERVO42D`, `SERVO57D`. Defaults to `SERVO42D`.
- **control_mode** (*Optional*, enum): Motor control mode. One of `SR_OPEN`, `SR_CLOSE`, `SR_VFOC`. Defaults to `SR_VFOC`.
  - `SR_OPEN`: Open-loop mode, stepper behaves like a regular stepper motor. Working current is `working_current`, holding current is `holding_current_percent` of working current.
  - `SR_CLOSE`: Closed-loop mode, same as `SR_OPEN` but with position feedback from encoder to prevent missed steps.
  - `SR_VFOC`: FOC mode (recommended), same as `SR_CLOSE` but current may be adaptet to the steppers needs up to the max `working_current`. `holding_current_percent` is ignored in this mode.
- **initial_speed** (*Optional*, float): Initial/target speed for motor operations. Supports units: `steps/s`, `RPM`, `revolutions/s`, `degrees/s`. Defaults to `1 RPM`. Must be ≤ `max_speed`
- **max_speed** (*Optional*, float): Maximum speed the contoller will ask for. Defaults to `control_mode` based Hardware limits: `SR_OPEN`: 400 RPM, `SR_CLOSE`: 1500 RPM, `SR_VFOC`: 3000 RPM.
- **acceleration** (*Optional*, float): The acceleration in steps/s^2 (steps per seconds squared) to use when changing the speed (for example when starting) of the stepper. The default is `inf` which means infinite acceleration, so the stepper will try to drive with the full speed immediately. This value is helpful if that first motion of the motor is too jerky for what it’s moving. If you make this a small number, it will take the motor a moment to get up to speed. `revolutions/s^2` may also be used.
  > [!NOTE]
  > Unlike the [ESPHome Stepper Component](https://esphome.io/components/stepper/), this component does not support separate `deceleration` parameter. Acceleration and deceleration use the same value (`acceleration = deceleration`).
- **working_current** (*Optional*, [Current](https://esphome.io/guides/configuration-types.html#config-current)): Working current. Accepts units: `mA` or `A` (e.g., `1500`, `1500mA`, `1.5A`). Defaults and maximums depend on `servo_type`:
  - Defaults: `0.6A` (28D), `0.8A` (35D), `1.6A` (42D), `3.2A` (57D)
  - Max: up to `3.0A` (28D/35D/42D), up to `5.2A` (57D)
- **holding_current_percent** (*Optional*, [Percentage](https://esphome.io/guides/configuration-types.html#config-percentage)): Holding current as percentage of working current (10-90%). Accepts: `40`, `40%`, or `0.4`. Only effective in `SR_OPEN` and `SR_CLOSE` modes.
- **en_pin_active** (*Optional*, enum): EN pin behavior. One of `LOW`, `HIGH`, `ALWAYS`. Defaults to `ALWAYS`.
- **auto_screen_off** (*Optional*, boolean): Automatically turn off motor display after 15 seconds. Defaults to `true`.
- **lock_keys_at_startup** (*Optional*, boolean): Lock motor display buttons at startup. Defaults to `false`.
- **update_interval** (*Optional*, [Time](https://esphome.io/guides/configuration-types.html#config-time)): Status polling interval. Defaults to `500ms`.
- **mode** (*Optional*, enum): Operating mode of the stepper. One of `position` or `speed`. Determines which actions and configurations are available. Defaults to `position`.

## Speed Mode

```yaml

stepper:
  - platform: servo42d_rs485
    id: my_stepper
    modbus_id: modbus1
    address: 0x01
    control_mode: SR_VFOC          # Hardware limit: 3000 RPM
    steps_per_revolution: 3200     # Needed for unit conversion
    initial_speed: 600 RPM                 # Initial/target speed for actions
    # max_speed is auto-set based on control_mode (3000 RPM for SR_VFOC)
    # You can override it to a lower value if needed:
    # max_speed: 2000 RPM
```


### Configuration
- **initial_direction** (*Optional*, enum): Direction for initial continuous movement. One of `CW` or `CCW`. Defaults to `CW`.
- All other from [Base Configuration](#base-configuration).

In following actions are exclisively used in speed mode:
- [`stepper.run_continuous`](#stepperrun_continuous) 


## Position Mode

```yaml

stepper:
  - platform: servo42d_rs485
    id: my_stepper
    modbus_id: modbus1
    address: 0x01
    control_mode: SR_VFOC           # Hardware limit: 3000 RPM
    steps_per_revolution: 3200
    initial_speed: 1000 steps/s      # Initial/target speed
    # max_speed: auto-set to 3000 RPM based on control_mode
    acceleration: 500 steps/s^2

    # Homing / 0_Mode (nested configuration)
    homing:
      mode: virtual              # limit | no_limit | virtual
      at_startup: false          # run homing on boot
      speed: 600 rpm             # or steps/s
      direction: NEAREST         # CW | CCW | NEAREST (NEAREST only for virtual)
      virtual_home_angle: 0°     # only for virtual
      zero_mode_speed_level: 2   # 0..4
```

### Configuration

- **sleep_when_done** (*Optional*, [Time](https://esphome.io/guides/configuration-types.html#config-time) or boolean): Put the motor to sleep after reaching the target and waiting for the set amount of time. Defaults to `false` or `inf` which may deactivate this function. `true` or any other [Time](https://esphome.io/guides/configuration-types.html#config-time) value may deactivate the motor after that amount of [Time](https://esphome.io/guides/configuration-types.html#config-time). `true` may equal a delay of `0ms`.
- **homing** (*Optional*, object): Homing configuration
  - **mode** (**Required**, enum):
    - `endstop`: Real homing using an endstop (limit switch).
    - `sensorless`: Sensorless homing using stall detection.
    - `virtual`: Return-to-zero using stored angle (0_Mode, no endstop).
      > [!NOTE]
      > Position to move to may be set at least once with `stepper.set_zero` before using virtual homing. After that is may be stored permanently within th controller of the stepper.
  - **direction** (*Optional*, enum): `CW` clockwise, `CCW` counter-clockwise and `NEAREST`. Default: `CW`. `NEAREST` may only be used with `mode: virtual`.
  - **speed**: (*Optional*, string): Homing speed. Supports units: `RPM` or `steps/s`. Default: `1 RPM`.
    > [!NOTE]
    > In `mode: virtual`, the speed may only be provided in five discrete levels. Use `VERY_SLOW`, `SLOW`, `MEDIUM`, `FAST` or `VERY_FAST` in this mode to set the speed.
  - **endstop_trigger** (*Optional*, enum): Endstop may be `LOW` or `HIGH` to be recognized as triggered. May only be used for `mode: endstop`. Default: `HIGH`.
  - **current** (*Optional*, [Current](https://esphome.io/guides/configuration-types.html#config-current)): Constant ccurrent used while Homing. Accepts units: `mA` or `A` (e.g., `1500`, `1500mA`, `1.5A`). May only be used for `mode: sensorless`. Default depends on `servo_type`: `0.6A` (28D), `0.8A` (35D), `1.6A` (42D), `3.2A` (57D).
  - **at_startup** (*Optional*, boolean): Run homing at startup. Default: `false`.

  - **zero_mode_speed_level** (*Optional*, int): 0–4. Maps to `zero_mode_speed_level`. Default: `2`.
  - **current** (*Optional*, [Current](https://esphome.io/guides/configuration-types.html#config-current)): Only for `no_limit` mode; homing/0_Mode current. Maps to `homing_current`.
- All other from [Base Configuration](#base-configuration).

In following actions are exclusively used in position mode:
- [`stepper.set_target`](#stepperset_target)
- [`stepper.report_position`](#stepperreport_position)
- [`stepper.home`](#stepperhome)
- [`stepper.set_zero`](#stepperset_zero)


## `stepper.set_target`

Set the target position of the motor. The stepper will move towards the target position and stop once reached.

```yaml
on_...:
  - stepper.set_target:
      id: my_stepper
      target: 1000
```

### Configuration

- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **target** (**Required**, int, [templatable](https://esphome.io/guides/configuration-types.html#config-templatable)): The target position in steps.

## `stepper.report_position`

Report the current position to a specific value (in steps). Sets an offset for future movements. To store a position for virtual homing, use [`stepper.set_zero`](#stepperset_zero) instead.

```yaml
on_...:
  - stepper.report_position:
      id: my_stepper
      position: 0
```

### Configuration

- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **position** (**Required**, int, [templatable](https://esphome.io/guides/configuration-types.html#config-templatable)): The position to report in steps.

## `stepper.home`
```yaml
on_...:
  - stepper.home: my_stepper
```
Execute homing sequence. Behavior depends on `homing.mode` configuration:
- `sensorless`: Uses stall detection (sensorless homing)
- `endstop`: Uses endstop and GoHome command (real homing)
- `virtual`: Restarts motor to return to stored zero position

## `stepper.set_zero`

Store the current position as persistent zero point for virtual homing. This must be called once before using virtual homing. The value is stored within the motor controller and remains after power-cycles.

```yaml
on_...:
  - stepper.set_zero: my_stepper
```

## `stepper.run_continuous`

Run the motor continuously at specified speed.

```yaml
on_...:
  - stepper.run_continuous:
      id: my_stepper
      direction: CW
      acceleration: 1000 steps/s²
      speed: 1000 steps/s
```

### Configuration

- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **speed** (*Optional*, [templatable](https://esphome.io/guides/configuration-types.html#config-templatable), float): Target speed in `steps/s`. Supports units like `steps/s`, `RPM`, `revolutions/s`, `degrees/s`.
- **direction** (*Optional*, [templatable](https://esphome.io/guides/configuration-types.html#config-templatable), enum): Direction, one of clockwise `CW`, counter-clockwise `CCW`.
- **acceleration** (*Optional*, [templatable](https://esphome.io/guides/configuration-types.html#config-templatable), float/string): Acceleration for speed-mode change in `steps/s^2`. `revolutions/s^2` is also supported.

> [!NOTE]
> Exactly one of `speed`, `direction`, or `acceleration` must be provided per call. Omitted values keep their last-used value. If a value was never set before, the component default is used.

## `stepper.stop`

Stop the current motor movement.

```yaml
on_...:
  - stepper.stop:
      id: my_stepper
      acceleration: 500 steps/s^2
```

### Configuration
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **acceleration** (*Optional*, [templatable](https://esphome.io/guides/configuration-types.html#config-templatable), float/string): Deceleration to use when stopping the motor in `steps/s^2`. `revolutions/s^2` is also supported.

> [!NOTE]
> `acceleration` keeps its last-used value. If a value was never set before, the component default is used.

> [!WARNING]
> At speeds above about 1000 RPM, avoid stopping too abruptly. Use a non-zero `acceleration` (deceleration) for smoother, safer stops to protect mechanics and couplings.

## `stepper.emergency_stop`

Emergency stop - immediately halt motor with maximum deceleration.

```yaml
on_...:
  - stepper.emergency_stop: my_stepper
```
### Configuration
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.

> [!WARNING]
> At speeds above about 1000 RPM, avoid stopping too abruptly. Use a non-zero `acceleration` (deceleration) for smoother, safer stops to protect mechanics and couplings.

## `stepper.enable` / `stepper.disable`

Enable or disables the motor. Same action is used für the `sleep_when_done` configuration.

```yaml
on_...:
  - stepper.enable: my_stepper
  - stepper.disable: my_stepper
```
### Configuration
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.

## `stepper.calibrate`

Start motor calibration sequence. Motor will move during calibration. Make sure that the stepper moves freely and is not obstructed.

```yaml
on_...:
  - stepper.calibrate: my_stepper
```

## `stepper.release_protection`

Release motor protection state after error condition. Is part of `stepper.home` action. Most of the cases that is th e right action to recover from an error.

```yaml
on_...:
  - stepper.release_protection: my_stepper
```

### `stepper.restart` Action

Restart the motor controller. Part of inital setup, and is also called when `stepper.homing.mode: virtual` is used, `stepper.homing.at_startup: true` is set and `stepper.set_zero` was at least once called before.

```yaml
on_...:
  - stepper.restart: my_stepper
```

## `stepper.set_work_mode`

Change the motor control mode at runtime.

```yaml
on_...:
  - stepper.set_work_mode:
      id: my_stepper
      mode: SR_vFOC
```

### Configuration
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **mode** (**Required**, enum): Work mode, one of `SR_OPEN`, `SR_CLOSE`, `SR_vFOC`.

## `stepper.set_working_current`

Change the working current at runtime.

```yaml
on_...:
  - stepper.set_working_current:
      id: my_stepper
      current: 2000  # mA
```

### Configuration

- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **current** (**Required**, int): Current in mA (0-3000 for SERVO42D, 0-5200 for SERVO57D).

## `stepper.set_holding_current_percent`

Change the holding current percentage at runtime. Only works in `SR_OPEN` and `SR_CLOSE` modes.

```yaml
on_...:
  - stepper.set_holding_current_percent:
      id: my_stepper
      percent: 40  # 10-90%
```

### Configuration

- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **percent** (**Required**, int): Percentage of working current (10-90).

## `stepper.set_microstepping`

Change microstepping (step mode) at runtime. `stepper.steps_per_revolution` is automatically adjusted accordingly. Sof if `stepper.set_target` ist used with microstepping `16` to move to position `1600`, after changing microstepping to `32`, the command `stepper.set_target` to position `1600` will move tht Steper to half the angle compared to before.

```yaml
on_...:
  - stepper.set_microstepping:
      id: my_stepper
      subdivision: 32  # 1-256
```

### Configuration

- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **subdivision** (**Required**, int): Microstepping / step mode (`1-256`), e.g., `1`=full, `2`=half, `4`=quarter.

## `stepper.key_lock` / `stepper.key_unlock`

Lock or unlock the motor display buttons.

```yaml
on_...:
  - stepper.key_lock: my_stepper
  - stepper.key_unlock: my_stepper
```
### Configuration
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.

## Hardware Setup

### Supported Models

- MKS Servo28D
- MKS Servo35D
- MKS Servo42D
- MKS Servo57D

All motors must be D-series with RS485 communication. C-series motors and CAN-bus variants are not supported.

### Wiring

follow the [ESPHome Modbus Component](https://esphome.io/components/modbus.html) wiring instructions. Connect the motor's RS485 A/B lines to the corresponding Modbus transceiver A/B lines. Ensure proper power supply for the motor as per its specifications.

### Motor Configuration

> [!IMPORTANT]
> Before using this component, you must configure your motor for Modbus communication using the built-in display-menu and the three keys. Set the setting done 

Required settings via motor menu:
- **Mb_RTU**: `Enable` (Enable MODBUS-RTU communication)
- **UartAddr**: Set device address 1-247 (must match ESPHome `address` config)
- **UartBaud**: Set baud rate (recommended: `9600`)

**Navigation:** Press `Menu` → Use `Next` to select → Press `Enter` to edit → Use `Next` to change → Press `Enter` to confirm

<!-- Examples moved into Speed Mode and Position Mode sections above -->
## See Also

- [ESPHome Modbus Component](https://esphome.io/components/modbus.html)
- [ESPHome Stepper Component](https://esphome.io/components/stepper/)
- [MKS Servo42&57D RS485 User Manual V1.0.5](docs/MKS%20SERVO42%2657D_RS485%20User%20Manual%20V1.0.5.pdf)
- Developer mapping (YAML → C++ → Modbus): `docs/CONFIG_TO_MODBUS.md`