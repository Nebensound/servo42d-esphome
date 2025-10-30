# Layer 4: Transport Abstraction - Detailed Specification

**Parent Document:** [02-cpp-interface.md](./02-cpp-interface.md)  
**Status:** ✅ FINALIZED – Layer 4 implementation details

**Navigation:**

- [← Previous: Layer 3 (CommandQueue)](./02c-layer3-command-queue.md)
- [← Back to Overview](./02-cpp-interface.md#layer-4-transport-layer-modbus-specific)

---

## Architecture

**Register-basierte Transport-Abstraktion**

MKS SERVO42D/57D Hardware: Identische Register-Codes für Modbus-RTU und Serial Protocol. Unterschiede nur im Transport-Format (Modbus Function Codes 0x04/0x06/0x10 vs. Serial FA/FB Frames).

```
Layer 4a: Commands (servoxxd_command.h/.cpp)
   ├─ ReadRegisterCommand(addr, count)
   ├─ WriteRegisterCommand(addr, value)
   └─ WriteMultipleRegistersCommand(addr, values)
   
Layer 4b: Transport Interface (servoxxd_transport.h)
   └─ ITransport (abstract)
   
Layer 4c: Implementations
   ├─ ModbusTransport
   └─ SerialTransport (future)
```

---

## Layer 4a: Commands

**Files:** `servoxxd_command.h/.cpp`

### BaseCommand

```cpp
class BaseCommand {
 protected:
  ITransport* transport_;      // NOT ModbusDevice!
  uint16_t register_address_;
  uint32_t timeout_ms_;
  CommandState state_;         // PENDING → EXECUTING → COMPLETED/FAILED/TIMEOUT
  
  std::function<void(bool)> completion_callback_;
  std::function<void(const std::vector<uint8_t>&)> data_callback_;

 public:
  virtual void execute() = 0;                          // Send via transport
  virtual void process_response(const std::vector<uint8_t>&) = 0;  // Parse response
  virtual const char* get_command_name() const = 0;
  
  bool is_timeout(uint32_t current_time_ms) const;
  // ... state management
};
```

### Concrete Commands

- `ReadRegisterCommand` - Liest Register via `transport_->send_read()`
- `WriteRegisterCommand` - Schreibt Register via `transport_->send_write()`
- `WriteMultipleRegistersCommand` - Schreibt mehrere Register via `transport_->send_write_multiple()`

---

## Layer 4b: Transport Interface

```cpp
class ITransport {
  virtual void send_read(uint16_t addr, uint16_t count) = 0;
  virtual void send_write(uint16_t addr, uint16_t value) = 0;
  virtual void send_write_multiple(uint16_t addr, const std::vector<uint16_t>& vals) = 0;
  
  virtual void set_response_callback(std::function<void(const std::vector<uint8_t>&)>) = 0;
  virtual void set_error_callback(std::function<void(uint8_t)>) = 0;
};
```

---

## Layer 4c: ModbusTransport

```cpp
class ModbusTransport : public ITransport {
  modbus::ModbusDevice* device_;
  
  void send_read(uint16_t addr, uint16_t count) override;    // Modbus 0x04
  void send_write(uint16_t addr, uint16_t value) override;   // Modbus 0x06
  void send_write_multiple(...) override;                    // Modbus 0x10
  
  void on_modbus_data(const std::vector<uint8_t>& data);
  void on_modbus_error(uint8_t func, uint8_t exc);
};
```

Übersetzt ITransport-Calls in Modbus Function Codes, routet ESPHome-Callbacks.

---

## Integration

- **CommandQueue:** Verwendet `ITransport*`, setzt Callbacks
- **ServoXxd:** Erstellt `ModbusTransport` in `setup()`, routet ESPHome-Callbacks
- **Testing:** `MockTransport` statt `MockModbusDevice`

---


## Complete Register/Function/Command Mapping

Below is a comprehensive, English-language table of all relevant register codes, functions, serial commands, and Modbus registers for the MKS SERVO42&57D. This table is based on the official hardware manual (see reference below).

| Register | Function | Serial Command | Modbus Register | Modbus Function |
|----------|----------|----------------|-----------------|-----------------|
| 0x30 | Read encoder carry | `FA 01 30 CRC` | 0x0030 | 0x04 (Read) |
| 0x31 | Read encoder addition | `FA 01 31 CRC` | 0x0031 | 0x04 (Read) |
| 0x32 | Read real-time speed (RPM) | `FA 01 32 CRC` | 0x0032 | 0x04 (Read) |
| 0x33 | Read pulse count | `FA 01 33 CRC` | 0x0033 | 0x04 (Read) |
| 0x34 | Read IO port status | `FA 01 34 CRC` | 0x0034 | 0x04 (Read) |
| 0x39 | Read angle error | `FA 01 39 CRC` | 0x0039 | 0x04 (Read) |
| 0x3A | Read enable pin status | `FA 01 3A CRC` | 0x003A | 0x04 (Read) |
| 0x3B | Read homing status | `FA 01 3B CRC` | 0x003B | 0x04 (Read) |
| 0x3D | Release locked-rotor protection | `FA 01 3D CRC` | 0x003D | 0x06 (Write) |
| 0x3E | Read protection status | `FA 01 3E CRC` | 0x003E | 0x04 (Read) |
| 0x3F | Restore default parameters | `FA 01 3F CRC` | 0x003F | 0x06 (Write) |
| 0x41 | Restart motor | `FA 01 41 CRC` | 0x0041 | 0x06 (Write) |
| 0x80 | Calibrate encoder | `FA 01 80 00 CRC` | 0x0080 | 0x06 (Write) |
| 0x82 | Set work mode | `FA 01 82 [mode] CRC` | 0x0082 | 0x06 (Write) |
| 0x83 | Set working current | `FA 01 83 [current] CRC` | 0x0083 | 0x06 (Write) |
| 0x84 | Set subdivision | `FA 01 84 [micstep] CRC` | 0x0084 | 0x06 (Write) |
| 0x85 | Set En pin active | `FA 01 85 [enable] CRC` | 0x0085 | 0x06 (Write) |
| 0x86 | Set direction | `FA 01 86 [dir] CRC` | 0x0086 | 0x06 (Write) |
| 0x87 | Set auto screen off | `FA 01 87 [enable] CRC` | 0x0087 | 0x06 (Write) |
| 0x88 | Set locked-rotor protection | `FA 01 88 [enable] CRC` | 0x0088 | 0x06 (Write) |
| 0x89 | Set subdivision interpolation | `FA 01 89 [enable] CRC` | 0x0089 | 0x06 (Write) |
| 0x8A | Set baud rate | `FA 01 8A [baud] CRC` | 0x008A | 0x06 (Write) |
| 0x8B | Set slave address | `FA 01 8B [addr] CRC` | 0x008B | 0x06 (Write) |
| 0x8C | Set slave respond/active | `FA 01 8C [resp] [active] CRC` | 0x008C | 0x06 (Write) |
| 0x8D | Set group address | `FA 01 8D [addr] CRC` | 0x008D | 0x06 (Write) |
| 0x8E | Set MODBUS-RTU enable | `FA 01 8E [enable] CRC` | 0x008E | 0x06 (Write) |
| 0x8F | Set key lock/unlock | `FA 01 8F [enable] CRC` | 0x008F | 0x06 (Write) |
| 0x90 | Set home parameters | `FA 01 90 [params...] CRC` | 0x0090 | 0x10 (Write Multiple) |
| 0x91 | Go home | `FA 01 91 CRC` | 0x0091 | 0x06 (Write) |
| 0x92 | Set current axis to zero | `FA 01 92 CRC` | 0x0092 | 0x06 (Write) |
| 0x94 | Set noLimit home params | `FA 01 94 [params...] CRC` | 0x0094 | 0x06 (Write) |
| 0x9A | Set 0_Mode params | `FA 01 9A [params...] CRC` | 0x009A | 0x10 (Write Multiple) |
| 0x9B | Set holding current % | `FA 01 9B [holdMa] CRC` | 0x009B | 0x06 (Write) |
| 0x9E | Set limit port remap | `FA 01 9E [enable] CRC` | 0x009E | 0x06 (Write) |
| 0xF1 | Query motor status | `FA 01 F1 CRC` | 0x00F1 | 0x04 (Read) |
| 0xF3 | Enable/disable motor | `FA 01 F3 [en] CRC` | 0x00F3 | 0x06 (Write) |
| 0xF4 | Position mode 3 (rel. axis) | `FA 01 F4 [params...] CRC` | 0x00F4 | 0x10 (Write Multiple) |
| 0xF5 | Position mode 4 (abs. axis) | `FA 01 F5 [params...] CRC` | 0x00F5 | 0x10 (Write Multiple) |
| 0xF6 | Speed mode | `FA 01 F6 [params...] CRC` | 0x00F6 | 0x10 (Write Multiple) |
| 0xF7 | Emergency stop | `FA 01 F7 CRC` | 0x00F7 | 0x06 (Write) |
| 0xFD | Position mode 1 (rel. pulses) | `FA 01 FD [params...] CRC` | 0x00FD | 0x10 (Write Multiple) |
| 0xFE | Position mode 2 (abs. pulses) | `FA 01 FE [params...] CRC` | 0x00FE | 0x10 (Write Multiple) |
| 0xFF | Save/Clean speed mode param | `FA 01 FF [flag] CRC` | 0x00FF | 0x06 (Write) |

**Reference:**
MKS SERVO42&57D_RS485 User Manual V1.0.5 — see `docs/servo_hardware_doc/AI/MKS_SERVO42D57D_RS485_User_Manual_V1.0.5.txt`

---

[← Layer 3](./02c-layer3-command-queue.md) | [↑ Overview](./02-cpp-interface.md#layer-4-transport-layer-modbus-specific)
