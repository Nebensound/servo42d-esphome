#include "servo42d_modbus_commands.h"
#include "esphome/components/modbus/modbus.h"
#include "esphome/core/log.h"

namespace esphome
{
  namespace servo42d_rs485
  {

    using esphome::modbus::ModbusDevice;

    static const char *TAG = "servo42d_rs485.modbus_commands";

    // ReadCommand Implementation
    ReadCommand::ReadCommand(uint16_t register_address, uint16_t quantity, uint32_t timeout_ms)
        : BaseCommand(register_address, timeout_ms), quantity_(quantity)
    {
    }

    bool ReadCommand::execute(ModbusDevice *device)
    {
      ESP_LOGV(TAG, "Executing ReadCommand: register=0x%04X, quantity=%u",
               get_register_address(), quantity_);

      set_state(CommandState::EXECUTING);
      reset_timeout();

      device->send(static_cast<uint8_t>(get_function_code()),
                   get_register_address(), quantity_);
      return true;
    }

    void ReadCommand::process_response(const std::vector<uint8_t> &data)
    {
      ESP_LOGV(TAG, "ReadCommand response: %zu bytes", data.size());

      // Expected response: data only (byte_count already removed by Modbus library)
      uint16_t expected_bytes = quantity_ * 2; // Each register is 2 bytes
      if (data.size() != expected_bytes)
      {
        ESP_LOGW(TAG, "ReadCommand: Invalid response size. Expected %u, got %zu",
                 expected_bytes, data.size());
        set_state(CommandState::FAILED);
        trigger_completion(false);
        return;
      }

      // Parse and store register values (byte_count already removed by Modbus library)
      values_.clear();
      for (size_t i = 0; i < quantity_; i++)
      {
        uint16_t value = (static_cast<uint16_t>(data[i * 2]) << 8) | data[i * 2 + 1];
        values_.push_back(value);
      }

      set_state(CommandState::COMPLETED);
      trigger_data_callback(data);
      trigger_completion(true);
    }

    // WriteCommand Implementation
    WriteCommand::WriteCommand(uint16_t register_address, uint16_t value, uint32_t timeout_ms)
        : BaseCommand(register_address, timeout_ms), value_(value)
    {
    }

    bool WriteCommand::execute(ModbusDevice *device)
    {
      ESP_LOGV(TAG, "Executing WriteCommand: register=0x%04X, value=0x%04X",
               get_register_address(), value_);

      set_state(CommandState::EXECUTING);
      reset_timeout();

      std::vector<uint8_t> payload = {
          static_cast<uint8_t>(value_ >> 8),  // High byte
          static_cast<uint8_t>(value_ & 0xFF) // Low byte
      };

      device->send(static_cast<uint8_t>(get_function_code()),
                   get_register_address(), 1, payload.size(), payload.data());
      return true;
    }

    void WriteCommand::process_response(const std::vector<uint8_t> &data)
    {
      ESP_LOGV(TAG, "WriteCommand response: %zu bytes", data.size());

      // Write command echo: should return same register address and value
      if (data.size() != 4)
      {
        ESP_LOGW(TAG, "WriteCommand: Invalid response size. Expected 4, got %zu", data.size());
        set_state(CommandState::FAILED);
        trigger_completion(false);
        return;
      }

      uint16_t returned_address = (data[0] << 8) | data[1];
      uint16_t returned_value = (data[2] << 8) | data[3];

      if (returned_address != get_register_address() || returned_value != value_)
      {
        ESP_LOGW(TAG, "WriteCommand: Response mismatch. Expected addr=0x%04X val=0x%04X, got addr=0x%04X val=0x%04X",
                 get_register_address(), value_, returned_address, returned_value);
        set_state(CommandState::FAILED);
        trigger_completion(false);
        return;
      }

      set_state(CommandState::COMPLETED);
      trigger_data_callback(data);
      trigger_completion(true);
    }

    // MultiWriteCommand Implementation
    MultiWriteCommand::MultiWriteCommand(uint16_t register_address, const std::vector<uint16_t> &values, uint32_t timeout_ms)
        : BaseCommand(register_address, timeout_ms), values_(values)
    {
    }

    bool MultiWriteCommand::execute(ModbusDevice *device)
    {
      ESP_LOGV(TAG, "Executing MultiWriteCommand: register=0x%04X, count=%zu",
               get_register_address(), values_.size());

      set_state(CommandState::EXECUTING);
      reset_timeout();

      // Build payload: byte_count + register_values
      std::vector<uint8_t> payload;
      payload.push_back(values_.size() * 2); // Byte count

      for (uint16_t value : values_)
      {
        payload.push_back(value >> 8);   // High byte
        payload.push_back(value & 0xFF); // Low byte
      }

      device->send(static_cast<uint8_t>(get_function_code()),
                   get_register_address(), values_.size(),
                   payload.size(), payload.data());
      return true;
    }

    void MultiWriteCommand::process_response(const std::vector<uint8_t> &data)
    {
      ESP_LOGV(TAG, "MultiWriteCommand response: %zu bytes", data.size());

      // Response: starting_address (2 bytes) + quantity (2 bytes)
      if (data.size() != 4)
      {
        ESP_LOGW(TAG, "MultiWriteCommand: Invalid response size. Expected 4, got %zu", data.size());
        set_state(CommandState::FAILED);
        trigger_completion(false);
        return;
      }

      uint16_t returned_address = (data[0] << 8) | data[1];
      uint16_t returned_quantity = (data[2] << 8) | data[3];

      // Some devices (like MKS Servo42D) echo start address correctly but report quantity=0
      // even when the write succeeds. Accept qty==0 as success if address matches.
      if (returned_address != get_register_address())
      {
        ESP_LOGW(TAG, "MultiWriteCommand: Response mismatch. Expected addr=0x%04X, got addr=0x%04X",
                 get_register_address(), returned_address);
        set_state(CommandState::FAILED);
        trigger_completion(false);
        return;
      }

      if (returned_quantity != values_.size())
      {
        if (returned_quantity == 0)
        {
          ESP_LOGV(TAG, "MultiWriteCommand: Device returned qty=0, accepting as success (addr OK)");
        }
        else
        {
          ESP_LOGW(TAG, "MultiWriteCommand: Response qty mismatch. Expected %zu, got %u",
                   values_.size(), returned_quantity);
          set_state(CommandState::FAILED);
          trigger_completion(false);
          return;
        }
      }

      set_state(CommandState::COMPLETED);
      trigger_data_callback(data);
      trigger_completion(true);
    }

  } // namespace servo42d_rs485
} // namespace esphome