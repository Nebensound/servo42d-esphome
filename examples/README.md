# Servo42D RS485 Examples

This directory contains example configurations for the `servo42d_rs485` ESPHome component.

## Setup

### WiFi Credentials

1. Copy the secrets template:
   ```bash
   cp examples/secrets.yaml.template examples/secrets.yaml
   ```

2. Edit `examples/secrets.yaml` with your WiFi credentials:
   ```yaml
   wifi_ssid: "YourActualWiFiSSID"
   wifi_password: "YourActualPassword"
   ota_password: "YourOTAPassword"
   fallback_ap_password: "fallback123"
   ```

3. The `secrets.yaml` file is in `.gitignore` and will not be committed.

## Prerequisites

Before using these examples, ensure your MKS Servo42D/57D motor is properly configured:

1. **Enable MODBUS-RTU Mode** (Critical!)
   - Use the motor's built-in display and buttons
   - Navigate to register `0x008E` 
   - Set value to `1` (Enable)
   - Without this, the motor won't respond to MODBUS commands!

2. **Set Communication Parameters**
   - Baud rate: `38400` (recommended) or `115200`
   - Parity: `EVEN`
   - Stop bits: `1`
   - Motor address: `1` (or any unique value for multi-motor setups)

3. **Configure Work Mode**
   - Set to `SR_vFOC` (mode 5) for best performance with serial control
   - Or `SR_CLOSE` (mode 4) for closed-loop position control

4. **Wire RS485 Connection**
   - ESP TX -> RS485 module DI (Data Input)
   - ESP RX -> RS485 module RO (Receiver Output)
   - RS485 A/B -> Motor A/B terminals
   - Connect GND between all devices

## Examples

### 1. `basic_stepper.yaml`
Minimal configuration to get started. Shows:
- Basic UART setup for RS485
- Single motor control
- Required configuration only

**Use this if:** You're setting up your first motor and want to verify communication.

### 2. `advanced_positioning.yaml`
Demonstrates advanced features including:
- Home Assistant integration
- Position control with number inputs
- Homing and calibration buttons
- Status monitoring sensors
- Emergency stop functionality

**Use this if:** You want full Home Assistant integration with UI controls.

### 3. `multi_motor.yaml`
Shows how to control multiple motors on one RS485 bus:
- Three motors with different addresses
- Coordinated homing sequences
- Proper timing between motor commands

**Use this if:** You're building a multi-axis system (CNC, 3D printer, robot arm, etc.)

## Testing Your Setup

1. Copy one of the example files
2. Adjust GPIO pins to match your hardware
3. Create a `secrets.yaml` file for WiFi credentials (advanced examples only)
4. Compile with: `esphome compile your_config.yaml`
5. Upload to your ESP32: `esphome upload your_config.yaml`
6. Monitor logs: `esphome logs your_config.yaml`

## Troubleshooting

**Motor doesn't respond:**
- ✅ Check if MODBUS-RTU is enabled (register 0x008E = 1)
- ✅ Verify baud rate matches between ESP and motor
- ✅ Check RS485 wiring (A to A, B to B)
- ✅ Ensure motor address matches configuration

**Communication errors:**
- ✅ Reduce baud rate to 38400
- ✅ Check for proper grounding
- ✅ Verify parity is set to EVEN
- ✅ Try shorter RS485 cable

**Position inaccuracies:**
- ✅ Perform motor calibration first
- ✅ Use SR_vFOC or SR_CLOSE mode
- ✅ Ensure proper current setting for your load

## More Information

See the main [README.md](../README.md) for complete API documentation and component details.
