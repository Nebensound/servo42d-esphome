# ESPHome Servo42D RS485 Component

An ESPHome component for controlling MKS Servo42D/57D closed-loop stepper motors via Modbus RTU (RS485 communication).

## Overview

This component provides ESPHome integration for MKS Servo42D/57D closed-loop stepper motors with **RS485 communication**. These are the **D-series** motors with built-in drivers, encoders, and Modbus RTU communication, making them ideal for precise positioning applications.

**Important**: This component is specifically designed for the **D-series** motors (Servo42D/57D) with **RS485 communication**. It does **not** work with:
- Servo42C (older version without RS485)
- D-series motors with CAN communication

## Features

- **Modbus RTU Communication**: Direct control via RS485/TTL
- **Closed-Loop Control**: Built-in encoder feedback for precise positioning
- **Flexible Units**: Configure speeds in steps/s, RPM, or revolutions/s
- **Microstepping Support**: Up to 256 microsteps per full step
- **Position Mode Control**: Absolute positioning with acceleration/deceleration
- **Real-time Monitoring**: Read motor status, speed, and encoder position
- **Energy Saving**: Automatic motor sleep when positioning is complete

## Motor Setup (Required Before First Use)

⚠️ **Important**: Before using this component, you must configure your Servo42D/57D motor for Modbus communication:

### 1. Motor Configuration via Built-in Display
Use the motor's built-in display and three buttons to configure:

**Navigation**: Use the three buttons (UP/DOWN/SELECT) to navigate the menu

### 2. Required Settings
Configure the following parameters via the motor's menu system:

```
Communication Mode: RS485 Modbus RTU
Baud Rate: 9600 (recommended)  
Device Address: 1-247 (must match ESPHome config)
Subdivision (Microsteps): 16 (recommended)
```

**Motor Configuration Menu**: 
Use the three buttons (Next/Enter/Menu) to navigate and configure:

**Required Settings via Menu:**
- **Mode**: Select serial interface mode (see options below)
- **Mb_RTU**: Set to `Enable` (Enable MODBUS-RTU communication)
- **UartAddr**: Set device address 1-247 (must match ESPHome config)
- **UartBaud**: Set baud rate, recommended `9600`
- **MStep**: Set subdivision/microsteps, recommended `16`

**Serial Mode Options (choose one):**
- **`SR_OPEN`**: Serial Open mode, runs without encoder feedback
  - Max RPM: 400 | Current: Fixed at Ma setting | Use: Simple applications without position feedback
- **`SR_CLOSE`**: Serial Closed-loop mode, runs with encoder feedback  
  - Max RPM: 1500 | Current: Fixed at Ma setting | Use: Precise positioning, recommended for 3D printing
- **`SR_vFOC`**: Serial FOC mode, runs with encoder and advanced control ⭐ **Recommended**
  - Max RPM: 3000 | Current: Self-adaptive (max Ma) | Use: High performance applications, best precision

**Navigation:** Press `Menu` → Use `Next` to select option → Press `Enter` to view/edit → Use `Next` to change value → Press `Enter` to confirm

### 3. Verify Configuration
- Check display shows current settings
- Ensure motor is in RS485 mode (not CAN)
- Note the device address for ESPHome configuration

**Without proper motor configuration, the ESPHome component will not be able to communicate with the motor!**

## Hardware Requirements

- ESP32 or ESP8266 with UART capability
- RS485 transceiver (e.g., MAX485) or direct TTL connection  
- **MKS Servo42D or Servo57D motor with RS485 communication** (D-series only)

## Installation

Add this repository to your ESPHome external components:

```yaml
external_components:
  - source: github://Arvernus/servo42d-esphome
    components: [servo42d_rs485]
```

## Basic Configuration

```yaml
# UART configuration for Modbus
uart:
  id: uart_bus
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 9600
  parity: NONE
  stop_bits: 1

# Modbus controller
modbus:
  uart_id: uart_bus

# Servo42D/57D stepper motor (RS485)
stepper:
  - platform: servo42d_rs485
    id: my_stepper
    address: 0x01                    # Modbus device address
    steps_per_revolution: 3200       # Steps for full rotation (default: 3200)
    microsteps: 16                   # Microstepping (1-256, default: 16)
    max_speed: 1000 steps/s          # Maximum speed
    acceleration: 500 steps/s^2      # Acceleration rate
    deceleration: 500 steps/s^2      # Deceleration rate
    sleep_when_done: false           # Motor sleep after positioning
    update_interval: 100ms           # Status update frequency
```

## Configuration Options

### Required Parameters

- **`id`**: Component identifier
- **`address`**: Modbus device address (1-247)
- **`max_speed`**: Maximum motor speed

### Optional Parameters

- **`steps_per_revolution`** (default: 3200): Steps for one full rotation
- **`microsteps`** (default: 16): Microstepping division (1-256)
- **`acceleration`** (default: inf): Acceleration rate
- **`deceleration`** (default: inf): Deceleration rate  
- **`sleep_when_done`** (default: false): Enable motor sleep after positioning
- **`update_interval`** (default: 100ms): Status polling frequency

### Speed and Acceleration Units

The component supports multiple units for speed and acceleration:

**Speed Units:**
- `steps/s` or `step/s` - Steps per second
- `steps/min` or `step/min` - Steps per minute  
- `RPM`, `rpm`, or `revolutions/min` - Revolutions per minute
- `RPS`, `rps`, or `revolutions/s` - Revolutions per second

**Acceleration Units:**
- `steps/s^2` - Steps per second squared
- `revolutions/s^2` - Revolutions per second squared
- Also supports `/min^2` variants

## Usage Examples

### Simple Movement

```yaml
# Move to absolute position
- stepper.set_target:
    id: my_stepper
    target: 1600  # Half revolution (if 3200 steps/rev)

# Relative movement
- stepper.report_position:
    id: my_stepper
    position: 0
- stepper.set_target:
    id: my_stepper
    target: !lambda "return id(my_stepper).current_position + 800;"
```

### Speed Control

```yaml
stepper:
  - platform: servo42d_rs485
    id: my_stepper
    max_speed: 2 revolutions/s      # 2 RPS
    acceleration: 1.5 revolutions/s^2
```

### Home Position Setup

```yaml
# The motor automatically homes on startup
# You can also trigger homing manually:
- stepper.report_position:
    id: my_stepper
    position: 0
```

## Advanced Configuration

### Multiple Motors

```yaml
stepper:
  - platform: servo42d_rs485
    id: motor_x
    address: 0x01
    max_speed: 1000 steps/s
    
  - platform: servo42d_rs485  
    id: motor_y
    address: 0x02
    max_speed: 1500 steps/s
```

### High-Speed Configuration

```yaml
stepper:
  - platform: servo42d_rs485
    id: high_speed_motor
    steps_per_revolution: 3200
    microsteps: 8              # Lower microsteps for higher speed
    max_speed: 10 revolutions/s
    acceleration: 5 revolutions/s^2
    sleep_when_done: true      # Save power
```

## Hardware Connection

### RS485 Connection
```
ESP32    MAX485    Servo42D/57D (D-Series)
GPIO17 → DI        A+
GPIO16 ← RO        B-
5V     → VCC       
GND    → GND       GND
```

### Direct TTL Connection (if supported by motor firmware)
```
ESP32     Servo42D/57D
GPIO17 → TX
GPIO16 ← RX  
GND    → GND
```

## Troubleshooting

### Common Issues

1. **No Response from Motor**
   - ❗ **Most common**: Motor not configured for Modbus communication (check display menu settings)
   - Check wiring and connections (A+/B- polarity)  
   - Verify Modbus address matches ESPHome configuration
   - Ensure correct baud rate (motor and ESPHome must match)
   - Check if motor is in correct work mode (Position Mode 5 via display menu)

2. **Motor Not Moving**
   - Check motor display for error messages
   - Verify power supply voltage and current capacity
   - Ensure `max_speed` is appropriate for motor specifications
   - Check mechanical load isn't too high

3. **Communication Errors**
   - Verify RS485 transceiver wiring (DI/RO connections)
   - Check motor display shows RS485 mode (not CAN mode)
   - Check for proper bus termination if using long cables
   - Ensure only one device per Modbus address

4. **Position Drift or Inaccuracy**
   - Verify proper mechanical coupling (no slipping)
   - Check for electromagnetic interference near motor
   - Ensure motor load is within specifications
   - Verify `steps_per_revolution` setting matches motor configuration

### Debug Logging

Enable debug logging for troubleshooting:

```yaml
logger:
  level: DEBUG
  logs:
    servo42d_rs485.stepper: DEBUG
    modbus: DEBUG
```

## Documentation

The [`docs/`](docs/) directory contains:
- Official motor manual (MKS SERVO42&57D_RS485 User Manual V1.0.5.pdf)
- Additional technical documentation as needed

For the most up-to-date information, refer to the official MKS documentation.

## License

This project is licensed under the EUPL-1.2 - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## Author

- [@jowgn](https://github.com/jowgn)