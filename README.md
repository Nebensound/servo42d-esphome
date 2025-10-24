# Servo42D RS485 Stepper

The `servo42d_rs485` stepper platform allows you to control MKS Servo42D/57D closed-loop stepper motors with RS485 communication via Modbus RTU.

```yaml
# Example configuration entry
uart:
  id: uart_bus
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 9600
  parity: EVEN

modbus:
  id: modbus1
  uart_id: uart_bus

stepper:
  - platform: servo42d_rs485
    id: my_stepper
    modbus_id: modbus1
    address: 0x01
    steps_per_revolution: 3200
    max_speed: 1000 steps/s
    acceleration: 500 steps/s^2
```

## Configuration Variables

- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **modbus_id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the Modbus controller.
- **address** (*Optional*, int): The Modbus device address. Defaults to `0x01`. Must match the motor's configured address (1-247).
- **max_speed** (**Required**, string): The maximum speed in steps per second. Can use units: `steps/s`, `RPM`, `revolutions/s`.
- **steps_per_revolution** (*Optional*, float): The number of steps for one full rotation. Defaults to `3200` (200 steps × 16 microsteps).
- **microsteps** (*Optional*, int): The microstepping subdivision (1-256). Defaults to `16`.
- **acceleration** (*Optional*, string): The acceleration rate. Can use units: `steps/s^2`, `revolutions/s^2`. Defaults to `inf`.
- **deceleration** (*Optional*, string): The deceleration rate. Can use units: `steps/s^2`, `revolutions/s^2`. Defaults to `inf`.
- **sleep_when_done** (*Optional*, boolean): Whether the motor should sleep after reaching target. Defaults to `false`.

### Motor Control Configuration

- **control_mode** (*Optional*, enum): Motor control mode. One of `SR_OPEN`, `SR_CLOSE`, `SR_vFOC`. Defaults to `SR_vFOC`.
  - `SR_OPEN`: Open-loop mode, max 400 RPM
  - `SR_CLOSE`: Closed-loop mode, max 1500 RPM
  - `SR_vFOC`: Vector FOC mode, max 3000 RPM (recommended)
- **working_current** (*Optional*, current): Arbeitsstrom in mA – akzeptiert Einheiten `mA` oder `A` (z. B. `1500`, `1500mA`, `1.5A`). Bereich: 0–3000 (Servo42D), bis 5200 (Servo57D).
- **holding_current_percent** (*Optional*, percent): Haltestrom in Prozent vom Arbeitsstrom (10–90%). Akzeptiert z. B. `40`, `40%` oder `0.4`. Nur wirksam in `SR_OPEN` und `SR_CLOSE`.

### Homing Configuration

- **home_at_startup** (*Optional*, boolean): Automatically home the motor on startup. Defaults to `false`.
- **use_virtual_home** (*Optional*, boolean): Use virtual homing (without endstop). Defaults to `false`.
  - If `false`: Uses real homing with endstop switch (command 0x91)
  - If `true`: Uses virtual homing to specified angle (command 0x9A)
- **virtual_home_angle** (*Optional*, degrees): Zielwinkel für virtuelles Homing (0–3°). Akzeptiert `90`, `90deg`, `90°`. Wird nur verwendet, wenn `use_virtual_home: true`.
- **homing_speed** (*Optional*, int): Homing speed in RPM. Defaults to `500`.
- **homing_direction** (*Optional*, enum): Homing direction. One of `CW`, `CCW`, `NEAREST`. Defaults to `CW`.
  - `CW`: Clockwise (for real homing with endstop)
  - `CCW`: Counter-clockwise (for real homing with endstop)
  - `NEAREST`: Shortest path (for virtual homing only)

### Display Configuration

- **auto_screen_off** (*Optional*, boolean): Automatically turn off motor display after 15 seconds. Defaults to `false`.
- **lock_keys_at_startup** (*Optional*, boolean): Lock motor display buttons at startup. Defaults to `false`.
- **post_arrival_hold_ms** (*Optional*, [Time](https://esphome.io/guides/configuration-types.html#config-time)): Zusätzliche Haltezeit bevor der Motor bei `sleep_when_done: true` automatisch deaktiviert wird. Akzeptiert Einheiten wie `ms`, `s`, `min` (z. B. `500ms`, `1s`, `2min`). Standard: `0ms`.

Beispiel:

```yaml
stepper:
  - platform: servo42d_rs485
    id: my_stepper
    sleep_when_done: true
    post_arrival_hold_ms: 750ms  # halte 0,75s nach Ankunft, dann EN aus
```

### Polling Configuration

- **update_interval** (*Optional*, [Time](https://esphome.io/guides/configuration-types.html#config-time)): Status polling interval. Defaults to `500ms`.
- All other options from [Polling Component](https://esphome.io/components/sensor/index.html#config-polling-component).

## Actions

### `stepper.enable`

Enable the motor.

```yaml
on_...:
  - stepper.enable: my_stepper
```

### `stepper.disable`

Disable the motor.

```yaml
on_...:
  - stepper.disable: my_stepper
```

### `stepper.run_continuous`

Run the motor continuously at specified speed.

```yaml
on_...:
  - stepper.run_continuous:
      id: my_stepper
      rpm: 500
      direction: CW  # or CCW
```

**Configuration variables:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **rpm** (**Required**, int): Speed in RPM (1-3000, depending on control_mode).
- **direction** (**Required**, enum): Direction, one of `CW`, `CCW`.

### `stepper.stop`

Stop continuous motor movement.

```yaml
on_...:
  - stepper.stop: my_stepper
```

### `stepper.emergency_stop`

Emergency stop - immediately halt motor with maximum deceleration.

```yaml
on_...:
  - stepper.emergency_stop: my_stepper
```

### `stepper.home`

Execute homing sequence. Uses virtual or real homing based on `use_virtual_home` configuration.

```yaml
on_...:
  - stepper.home: my_stepper
```

### `stepper.reset_position`

Reset the current position to zero (software baseline reset).

```yaml
on_...:
  - stepper.reset_position: my_stepper
```

### `stepper.calibrate`

Start motor calibration sequence. Motor will move during calibration.

```yaml
on_...:
  - stepper.calibrate: my_stepper
```

### `stepper.release_protection`

Release motor protection state after error condition.

```yaml
on_...:
  - stepper.release_protection: my_stepper
```

### `stepper.restart`

Restart the motor controller.

```yaml
on_...:
  - stepper.restart: my_stepper
```

### `stepper.set_work_mode`

Change the motor control mode at runtime.

```yaml
on_...:
  - stepper.set_work_mode:
      id: my_stepper
      mode: SR_vFOC  # or SR_OPEN, SR_CLOSE
```

**Configuration variables:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **mode** (**Required**, enum): Work mode, one of `SR_OPEN`, `SR_CLOSE`, `SR_vFOC`.

### `stepper.set_working_current`

Change the working current at runtime.

```yaml
on_...:
  - stepper.set_working_current:
      id: my_stepper
      current: 2000  # mA
```

**Configuration variables:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **current** (**Required**, int): Current in mA (0-3000 for SERVO42D, 0-5200 for SERVO57D).

### `stepper.set_holding_current_percent`

Change the holding current percentage at runtime. Only works in `SR_OPEN` and `SR_CLOSE` modes.

```yaml
on_...:
  - stepper.set_holding_current_percent:
      id: my_stepper
      percent: 40  # 10-90%
```

**Configuration variables:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **percent** (**Required**, int): Percentage of working current (10-90).

### `stepper.set_microstepping`

Change the microstepping subdivision at runtime.

```yaml
on_...:
  - stepper.set_microstepping:
      id: my_stepper
      subdivision: 32  # 1-256
```

**Configuration variables:**
- **id** (**Required**, [ID](https://esphome.io/guides/configuration-types.html#config-id)): The ID of the stepper.
- **subdivision** (**Required**, int): Microstepping subdivision (1-256).

### `stepper.key_lock`

Lock the motor display buttons to prevent manual operation.

```yaml
on_...:
  - stepper.key_lock: my_stepper
```

### `stepper.key_unlock`

Unlock the motor display buttons to allow manual operation.

```yaml
on_...:
  - stepper.key_unlock: my_stepper
```

## Example Configurations

### Basic Setup

```yaml
uart:
  id: uart_bus
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 9600
  parity: EVEN

modbus:
  id: modbus1
  uart_id: uart_bus

stepper:
  - platform: servo42d_rs485
    id: my_stepper
    modbus_id: modbus1
    address: 0x01
    steps_per_revolution: 3200
    max_speed: 1000 steps/s
    acceleration: 500 steps/s^2

# Position sensor
sensor:
  - platform: template
    name: "Motor Position"
    lambda: return id(my_stepper).current_position;
    update_interval: 100ms

# Control buttons
button:
  - platform: template
    name: "Move Forward"
    on_press:
      - stepper.set_target:
          id: my_stepper
          target: !lambda "return id(my_stepper).current_position + 1000;"
```

### Advanced Configuration with Virtual Homing

```yaml
stepper:
  - platform: servo42d_rs485
    id: my_stepper
    modbus_id: modbus1
    address: 0x01
    
    # Motor configuration
    control_mode: SR_vFOC
    working_current: 1.5A
    steps_per_revolution: 3200
    microsteps: 16
    max_speed: 2 revolutions/s
    acceleration: 1.5 revolutions/s^2
    
    # Virtual homing (no endstop needed)
    home_at_startup: true
    use_virtual_home: true
    virtual_home_angle: 180deg
    homing_speed: 1000
    homing_direction: NEAREST
    
    # Display settings
    auto_screen_off: true
    lock_keys_at_startup: false
```

### Multiple Motors

```yaml
modbus:
  id: modbus1
  uart_id: uart_bus

stepper:
  - platform: servo42d_rs485
    id: motor_x
    modbus_id: modbus1
    address: 0x01
    steps_per_revolution: 3200
    max_speed: 1000 steps/s
    
  - platform: servo42d_rs485
    id: motor_y
    modbus_id: modbus1
    address: 0x02
    steps_per_revolution: 3200
    max_speed: 1500 steps/s
```

---

## Additional Information

### Hardware Requirements

- ESP32 or ESP8266 with UART capability
- RS485 transceiver (e.g., MAX485) or direct TTL connection  
- **MKS Servo42D or Servo57D motor with RS485 communication** (D-series only)

**Important**: This component is specifically designed for the **D-series** motors (Servo42D/57D) with **RS485 communication**. It does **not** work with:
- Servo42C (older version without RS485)
- D-series motors with CAN communication

### Hardware Connection

**RS485 Connection:**
```
ESP32    MAX485    Servo42D/57D
GPIO17 → DI        A+
GPIO16 ← RO        B-
5V     → VCC       
GND    → GND       GND
```

**Direct TTL Connection (if supported by motor firmware):**
```
ESP32     Servo42D/57D
GPIO17 → TX
GPIO16 ← RX  
GND    → GND
```

### Motor Setup (Required Before First Use)

⚠️ **Important**: Before using this component, you must configure your Servo42D/57D motor for Modbus communication using the motor's built-in display.

**Navigation**: Use the three buttons (Next/Enter/Menu) to navigate the menu.

**Required Settings via Motor Menu:**
- **Mode**: Select `SR_OPEN`, `SR_CLOSE`, or `SR_vFOC` (recommended: `SR_vFOC`)
- **Mb_RTU**: Set to `Enable` (Enable MODBUS-RTU communication)
- **UartAddr**: Set device address 1-247 (must match ESPHome `address` config)
- **UartBaud**: Set baud rate, recommended `9600`
- **MStep**: Set subdivision/microsteps, recommended `16`

**Serial Mode Comparison:**

| Mode | Max RPM | Current Control | Encoder Feedback | Best For |
|------|---------|-----------------|------------------|----------|
| SR_OPEN | 400 | Fixed | No | Simple applications |
| SR_CLOSE | 1500 | Fixed | Yes | Precise positioning, 3D printing |
| SR_vFOC | 3000 | Adaptive | Yes | High performance ⭐ Recommended |

**Navigation:** Press `Menu` → Use `Next` to select option → Press `Enter` to view/edit → Use `Next` to change value → Press `Enter` to confirm

**Without proper motor configuration, the ESPHome component will not be able to communicate with the motor!**

### Troubleshooting

**No Response from Motor:**
- ❗ **Most common**: Motor not configured for Modbus communication (check display menu settings)
- Check wiring and connections (A+/B- polarity for RS485)
- Verify Modbus address matches ESPHome configuration
- Ensure correct baud rate (motor and ESPHome must match)
- Check if motor is in correct serial mode via display menu

**Motor Not Moving:**
- Check motor display for error messages
- Verify power supply voltage and current capacity
- Ensure `max_speed` is appropriate for motor specifications
- Check mechanical load isn't too high

**Communication Errors:**
- Verify RS485 transceiver wiring (DI/RO connections)
- Check motor display shows RS485 mode (not CAN mode)
- Check for proper bus termination if using long cables
- Ensure only one device per Modbus address

**Position Drift or Inaccuracy:**
- Verify proper mechanical coupling (no slipping)
- Check for electromagnetic interference near motor
- Ensure motor load is within specifications
- Verify `steps_per_revolution` setting matches motor configuration

**Debug Logging:**

```yaml
logger:
  level: DEBUG
  logs:
    servo42d_rs485.stepper: DEBUG
    modbus: DEBUG
```

### Documentation References

- **[MKS SERVO42&57D RS485 User Manual V1.0.5](docs/MKS%20SERVO42%2657D_RS485%20User%20Manual%20V1.0.5.pdf)** - Official motor documentation
- [ESPHome Modbus Component](https://esphome.io/components/modbus.html) - Modbus RTU documentation
- [ESPHome Stepper Component](https://esphome.io/components/stepper/) - General stepper information

### Repository

**Source**: https://github.com/Nebensound/servo42d-esphome  
**Sponsor**: [Nebensound](https://github.com/Nebensound)

### License

This project is licensed under the EUPL-1.2 - see the [LICENSE](LICENSE) file for details.


- [@jowgn](https://github.com/jowgn)

## Sponsors

This project is supported by:
- [@Nebensound](https://github.com/Nebensound)