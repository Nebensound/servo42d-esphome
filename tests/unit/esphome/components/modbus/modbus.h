#pragma once

// Mock ESPHome Modbus header for unit testing

#include <cstdint>
#include <vector>

namespace esphome
{
  namespace modbus
  {

    // Modbus function codes
    constexpr uint8_t FUNC_PRESET_MULTIPLE_REGISTERS = 0x10;

    class ModbusDevice
    {
    public:
      virtual ~ModbusDevice() = default;

      // Mock methods for unit testing
      virtual void set_address(uint8_t address) { address_ = address; }
      virtual uint8_t get_address() const { return address_; }

      // Mock send method (for CommandQueue integration)
      // Signature matches ESPHome's ModbusDevice: 5 parameters (function, address, count/value, data_len, data)
      virtual void send(uint8_t function_code, uint16_t start_address, uint16_t count_or_value,
                        uint8_t data_len = 0, const uint8_t *data = nullptr)
      {
        // Mock implementation for testing
        (void)data_len;
        (void)data;
      }

      virtual void send_raw(const std::vector<uint8_t> &payload)
      {
        // Mock implementation for testing
      }

      // Virtual modbus callback methods
      virtual void on_modbus_data(const std::vector<uint8_t> &data) {}
      virtual void on_modbus_error(uint8_t function_code, uint8_t exception_code) {}

    protected:
      uint8_t address_{0};
    };

  } // namespace modbus
} // namespace esphome
