#pragma once

#include "base_command.h"
#include <vector>
#include <cstdint>

namespace esphome
{
  namespace servo42d_rs485
  {
    namespace servo42d_rs485
    {

      class ReadCommand : public BaseCommand
      {
      public:
        ReadCommand(uint16_t register_address, uint16_t count, uint32_t timeout_ms = 1000);

        bool execute(modbus::ModbusDevice *device) override;
        void process_response(const std::vector<uint8_t> &data) override;
        ModbusFunction get_function_code() const override { return ModbusFunction::READ_INPUT_REGISTERS; }
        const char *get_command_name() const override { return "ReadCommand"; }

        uint16_t get_quantity() const { return quantity_; }
        const std::vector<uint16_t> &get_values() const { return values_; }

      private:
        uint16_t quantity_;
        std::vector<uint16_t> values_; // Store read values
      };
      class WriteCommand : public BaseCommand
      {
      public:
      public:
        WriteCommand(uint16_t register_address, uint16_t value, uint32_t timeout_ms = 1000);

        bool execute(modbus::ModbusDevice *device) override;
        void process_response(const std::vector<uint8_t> &data) override;
        ModbusFunction get_function_code() const override { return ModbusFunction::WRITE_SINGLE_REGISTER; }
        const char *get_command_name() const override { return "WriteCommand"; }

        uint16_t get_value() const { return value_; }

      private:
        uint16_t value_;
      };

      class MultiWriteCommand : public BaseCommand
      {
      public:
        MultiWriteCommand(uint16_t register_address, const std::vector<uint16_t> &values, uint32_t timeout_ms = 1000);

        bool execute(modbus::ModbusDevice *device) override;
        void process_response(const std::vector<uint8_t> &data) override;
        ModbusFunction get_function_code() const override { return ModbusFunction::WRITE_MULTIPLE_REGISTERS; }
        const char *get_command_name() const override { return "MultiWriteCommand"; }

        const std::vector<uint16_t> &get_values() const { return values_; }
        uint16_t get_quantity() const { return values_.size(); }

      private:
        std::vector<uint16_t> values_;
      };

    } // namespace servo42d_rs485
  } // namespace esphome