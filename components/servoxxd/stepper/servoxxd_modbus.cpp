#include "servoxxd_modbus.h"
#include "esphome/core/log.h"

namespace esphome
{
  namespace servoxxd
  {

    static const char *const TAG = "servoxxd.modbus";

    // ============================================================================
    // ReadCommand (0x04 - Read Input Registers)
    // ============================================================================

    void ReadCommand::execute(modbus::ModbusDevice *device)
    {
      if (!device)
      {
        ESP_LOGE(TAG, "ReadCommand::execute - device is null");
        state_ = CommandState::FAILED;
        invoke_completion_callback(false);
        return;
      }

      ESP_LOGV(TAG, "ReadCommand: Reading %d registers from 0x%04X", quantity_, register_address_);

      // Send Modbus 0x04 (Read Input Registers) request
      // Format: [device_addr, function_code, start_addr_hi, start_addr_lo, quantity_hi, quantity_lo, crc]
      device->send(0x04, register_address_, quantity_);

      // State transition handled by caller (CommandQueue sets EXECUTING + start_time)
    }

    bool ReadCommand::process_response(const std::vector<uint8_t> &data)
    {
      // Response format: [byte_count, reg1_hi, reg1_lo, reg2_hi, reg2_lo, ...]
      // byte_count = quantity * 2 (each register is 2 bytes)

      if (data.empty())
      {
        ESP_LOGE(TAG, "ReadCommand: Empty response data");
        state_ = CommandState::FAILED;
        invoke_completion_callback(false);
        return false;
      }

      uint8_t byte_count = data[0];
      uint16_t expected_byte_count = quantity_ * 2;

      if (byte_count != expected_byte_count)
      {
        ESP_LOGE(TAG, "ReadCommand: Invalid byte count. Expected %d, got %d",
                 expected_byte_count, byte_count);
        state_ = CommandState::FAILED;
        invoke_completion_callback(false);
        return false;
      }

      // Validate data length (byte_count + 1 for the byte_count field itself)
      if (data.size() < (size_t)(byte_count + 1))
      {
        ESP_LOGE(TAG, "ReadCommand: Incomplete data. Expected %d bytes, got %d",
                 byte_count + 1, data.size());
        state_ = CommandState::FAILED;
        invoke_completion_callback(false);
        return false;
      }

      // Extract register values (skip first byte which is byte_count)
      std::vector<uint8_t> register_data(data.begin() + 1, data.begin() + 1 + byte_count);

      ESP_LOGV(TAG, "ReadCommand: Successfully read %d registers (%d bytes)",
               quantity_, byte_count);

      // Call data callback with extracted register bytes
      invoke_data_callback(register_data);

      state_ = CommandState::COMPLETED;
      invoke_completion_callback(true);
      return true;
    }

    // ============================================================================
    // WriteCommand (0x06 - Write Single Register)
    // ============================================================================

    void WriteCommand::execute(modbus::ModbusDevice *device)
    {
      if (!device)
      {
        ESP_LOGE(TAG, "WriteCommand::execute - device is null");
        state_ = CommandState::FAILED;
        invoke_completion_callback(false);
        return;
      }

      ESP_LOGV(TAG, "WriteCommand: Writing 0x%04X to register 0x%04X", value_, register_address_);

      // Send Modbus 0x06 (Write Single Register) request
      // Format: [device_addr, function_code, reg_addr_hi, reg_addr_lo, value_hi, value_lo, crc]
      device->send(0x06, register_address_, value_);

      // State transition handled by caller (CommandQueue sets EXECUTING + start_time)
    }

    bool WriteCommand::process_response(const std::vector<uint8_t> &data)
    {
      // Response format: [reg_addr_hi, reg_addr_lo, value_hi, value_lo] (echo of request)
      // Modbus 0x06 response echoes the register address and value written

      if (data.size() < 4)
      {
        ESP_LOGE(TAG, "WriteCommand: Invalid response length. Expected 4 bytes, got %d", data.size());
        state_ = CommandState::FAILED;
        invoke_completion_callback(false);
        return false;
      }

      // Extract echoed register address
      uint16_t echoed_address = (data[0] << 8) | data[1];

      // Extract echoed value
      uint16_t echoed_value = (data[2] << 8) | data[3];

      // Validate echo matches what we sent
      if (echoed_address != register_address_)
      {
        ESP_LOGE(TAG, "WriteCommand: Address mismatch. Sent 0x%04X, received 0x%04X",
                 register_address_, echoed_address);
        state_ = CommandState::FAILED;
        invoke_completion_callback(false);
        return false;
      }

      if (echoed_value != value_)
      {
        ESP_LOGE(TAG, "WriteCommand: Value mismatch. Sent 0x%04X, received 0x%04X",
                 value_, echoed_value);
        state_ = CommandState::FAILED;
        invoke_completion_callback(false);
        return false;
      }

      ESP_LOGV(TAG, "WriteCommand: Successfully wrote 0x%04X to register 0x%04X",
               value_, register_address_);

      state_ = CommandState::COMPLETED;
      invoke_completion_callback(true);
      return true;
    }

    // ============================================================================
    // MultiWriteCommand (0x10 - Write Multiple Registers)
    // ============================================================================

    void MultiWriteCommand::execute(modbus::ModbusDevice *device)
    {
      if (!device)
      {
        ESP_LOGE(TAG, "MultiWriteCommand::execute - device is null");
        state_ = CommandState::FAILED;
        invoke_completion_callback(false);
        return;
      }

      if (values_.empty())
      {
        ESP_LOGE(TAG, "MultiWriteCommand: No values to write");
        state_ = CommandState::FAILED;
        invoke_completion_callback(false);
        return;
      }

      uint16_t quantity = values_.size();
      ESP_LOGV(TAG, "MultiWriteCommand: Writing %d registers starting at 0x%04X",
               quantity, register_address_);

      // Build payload for Modbus 0x10 (Write Multiple Registers)
      // Format: [device_addr, function_code, start_addr_hi, start_addr_lo,
      //          quantity_hi, quantity_lo, byte_count,
      //          reg1_hi, reg1_lo, reg2_hi, reg2_lo, ..., crc]

      std::vector<uint8_t> payload;

      // Function code
      payload.push_back(0x10);

      // Register address (big-endian)
      payload.push_back((register_address_ >> 8) & 0xFF);
      payload.push_back(register_address_ & 0xFF);

      // Quantity of registers (big-endian)
      payload.push_back((quantity >> 8) & 0xFF);
      payload.push_back(quantity & 0xFF);

      // Byte count (quantity * 2)
      uint8_t byte_count = quantity * 2;
      payload.push_back(byte_count);

      // Register values (big-endian)
      for (uint16_t value : values_)
      {
        payload.push_back((value >> 8) & 0xFF);
        payload.push_back(value & 0xFF);
      }

      // Send raw Modbus request
      device->send_raw(payload);

      // State transition handled by caller (CommandQueue sets EXECUTING + start_time)
    }

    bool MultiWriteCommand::process_response(const std::vector<uint8_t> &data)
    {
      // Response format: [reg_addr_hi, reg_addr_lo, quantity_hi, quantity_lo]
      // Modbus 0x10 response confirms the starting address and quantity written

      if (data.size() < 4)
      {
        ESP_LOGE(TAG, "MultiWriteCommand: Invalid response length. Expected 4 bytes, got %d",
                 data.size());
        state_ = CommandState::FAILED;
        invoke_completion_callback(false);
        return false;
      }

      // Extract echoed register address
      uint16_t echoed_address = (data[0] << 8) | data[1];

      // Extract echoed quantity
      uint16_t echoed_quantity = (data[2] << 8) | data[3];

      uint16_t expected_quantity = values_.size();

      // Validate echo matches what we sent
      if (echoed_address != register_address_)
      {
        ESP_LOGE(TAG, "MultiWriteCommand: Address mismatch. Sent 0x%04X, received 0x%04X",
                 register_address_, echoed_address);
        state_ = CommandState::FAILED;
        invoke_completion_callback(false);
        return false;
      }

      if (echoed_quantity != expected_quantity)
      {
        ESP_LOGE(TAG, "MultiWriteCommand: Quantity mismatch. Sent %d, received %d",
                 expected_quantity, echoed_quantity);
        state_ = CommandState::FAILED;
        invoke_completion_callback(false);
        return false;
      }

      ESP_LOGV(TAG, "MultiWriteCommand: Successfully wrote %d registers starting at 0x%04X",
               echoed_quantity, register_address_);

      state_ = CommandState::COMPLETED;
      invoke_completion_callback(true);
      return true;
    }

  } // namespace servoxxd
} // namespace esphome
