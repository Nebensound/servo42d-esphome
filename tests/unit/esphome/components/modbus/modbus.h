#pragma once

// Mock ESPHome Modbus header for unit testing

#include <cstdint>
#include <vector>

namespace esphome
{
  namespace modbus
  {

    class ModbusDevice
    {
    public:
      virtual ~ModbusDevice() = default;

      // Mock methods for unit testing
      virtual void set_address(uint8_t address) { address_ = address; }
      virtual uint8_t get_address() const { return address_; }

      // Virtual modbus callback methods
      virtual void on_modbus_data(const std::vector<uint8_t> &data) {}
      virtual void on_modbus_error(uint8_t function_code, uint8_t exception_code) {}

    protected:
      uint8_t address_{0};
    };

  } // namespace modbus
} // namespace esphome
