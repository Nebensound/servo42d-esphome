#include "servoxxd_modbus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome
{
  namespace servoxxd
  {

    static const char *const TAG = "servoxxd.modbus";

    // Helper: Determine if command is a read operation
    static inline bool is_read_command(Command cmd)
    {
      uint8_t code = static_cast<uint8_t>(cmd);
      // Read commands are in range 0x30-0x3F (except 0x3F which is RESTART)
      return (code >= 0x30 && code < 0x3F);
    }

    ModbusTransport::ModbusTransport(modbus::ModbusDevice *device, uint8_t slave_address)
        : device_(device), slave_address_(slave_address) {}

    Result ModbusTransport::execute_command(Command cmd, const std::vector<uint8_t> &data)
    {
      if (state_ != State::IDLE)
      {
        ESP_LOGW(TAG, "Transport busy, cannot execute command 0x%02X", static_cast<uint8_t>(cmd));
        return {false, ErrorCode::BUSY};
      }

      // Determine if this is a read or write operation
      if (is_read_command(cmd))
      {
        // Delegate to read_command() - data parameter ignored for reads
        return read_command(cmd);
      }

      // Write operation - validate data size and choose Modbus function
      uint16_t register_address = command_to_register(cmd);

      // MKS SERVO supports both Function 0x06 and 0x10:
      // - Function 0x06 (Write Single Register): 2 bytes (most commands)
      // - Function 0x10 (Write Multiple Registers): 4+ bytes (home params, 0_Mode, motion)

      if (data.size() == 2)
      {
        // Write single register (Function 0x06)
        // Third parameter: value to write (16-bit)
        uint16_t register_value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
        device_->send(0x06, register_address, register_value, 0, nullptr);

        pending_function_code_ = 0x06;
        pending_register_address_ = register_address;
        pending_data_ = data; // Store for response validation
      }
      else if (data.size() >= 4)
      {
        // Write multiple registers (Function 0x10)
        // Third parameter: register count (not response length!)
        uint16_t register_count = (data.size() + 1) / 2;
        uint8_t padded_size = register_count * 2;

        // Pad data if necessary (Modbus registers are 16-bit)
        std::vector<uint8_t> padded_data(data.begin(), data.end());
        if (padded_data.size() % 2 != 0)
        {
          padded_data.push_back(0x00);
        }

        device_->send(0x10, register_address, register_count, padded_size, padded_data.data());

        pending_function_code_ = 0x10;
        pending_register_address_ = register_address;
        pending_data_.clear(); // 0x10 doesn't echo data, only count
      }
      else
      {
        ESP_LOGE(TAG, "Invalid data size %d for command 0x%02X (must be 2 or 4+ bytes)",
                 data.size(), static_cast<uint8_t>(cmd));
        return {false, ErrorCode::TIMEOUT};
      }

      state_ = State::WAITING_WRITE;
      pending_command_ = cmd;
      timeout_start_ms_ = millis();

      ESP_LOGD(TAG, "Execute command 0x%02X, register 0x%04X, %d bytes",
               static_cast<uint8_t>(cmd), register_address, data.size());

      return {true, ErrorCode::OK};
    }

    Result ModbusTransport::read_command(Command cmd)
    {
      if (state_ != State::IDLE)
      {
        ESP_LOGW(TAG, "Transport busy, cannot read command 0x%02X", static_cast<uint8_t>(cmd));
        return {false, ErrorCode::BUSY};
      }

      uint16_t register_address = command_to_register(cmd);

      // Determine register count based on command type
      uint16_t register_count = 1;
      switch (cmd)
      {
      case Command::READ_ENCODER_CARRY:
        register_count = 3; // 6 bytes: carry (4) + value (2)
        break;
      case Command::READ_PULSE_COUNT:
        register_count = 2; // 4 bytes
        break;
      case Command::READ_CURRENT_SPEED:
      case Command::READ_MOTOR_STATUS:
      case Command::READ_PROTECTION_STATUS:
      case Command::READ_IO_STATUS:
      case Command::READ_ZERO_RETURN_STATUS:
        register_count = 1; // 2 bytes
        break;
      default:
        register_count = 1;
        break;
      }

      // Send read input registers (Function 0x04)
      // Response length: address[1] + function[1] + byte_count[1] + data[register_count*2] + CRC[2]
      uint16_t expected_response_length = 5 + (register_count * 2);
      device_->send(0x04, register_address, register_count, 0, nullptr);

      state_ = State::WAITING_READ;
      pending_command_ = cmd;
      pending_function_code_ = 0x04;
      pending_register_address_ = register_address;
      pending_data_.resize(1);
      pending_data_[0] = static_cast<uint8_t>(register_count); // Store expected register count
      timeout_start_ms_ = millis();

      ESP_LOGD(TAG, "Read command 0x%02X, register 0x%04X, count %d, expecting %d bytes",
               static_cast<uint8_t>(cmd), register_address, register_count, expected_response_length);

      return {true, ErrorCode::OK};
    }

    bool ModbusTransport::is_busy() const
    {
      return state_ != State::IDLE;
    }

    bool ModbusTransport::is_waiting_write() const
    {
      return state_ == State::WAITING_WRITE;
    }

    void ModbusTransport::update()
    {
      if (state_ == State::IDLE)
      {
        return;
      }

      // Check for timeout
      if (check_timeout())
      {
        ESP_LOGW(TAG, "Command 0x%02X timed out", static_cast<uint8_t>(pending_command_));
        state_ = State::IDLE;
        if (error_callback_)
        {
          error_callback_(pending_command_, ErrorCode::TIMEOUT);
        }
        return;
      }

      // Note: ESPHome's ModbusDevice handles response parsing via callbacks
      // Upper layers (CommandQueue) should call on_modbus_data() / on_modbus_error()
      // This implementation provides a simplified synchronous-style wrapper
    }

    void ModbusTransport::set_response_callback(
        std::function<void(Command, const std::vector<uint8_t> &)> cb)
    {
      response_callback_ = cb;
    }

    void ModbusTransport::set_error_callback(
        std::function<void(Command, ErrorCode)> cb)
    {
      error_callback_ = cb;
    }

    bool ModbusTransport::check_timeout()
    {
      return (millis() - timeout_start_ms_) > timeout_ms_;
    }

    void ModbusTransport::handle_response(const std::vector<uint8_t> &data)
    {
      // Validate Modbus response based on function code in response
      // ESPHome Modbus gives us: [SlaveAddr][Function][Bytes][value...][CRC16]
      // But CRC and address are already validated by ESPHome, so we get: [Function][payload...]
      //
      // Function 0x04 (Read):  [0x04][byte_count][data...]
      // Function 0x06 (Write): [0x06][addr_hi][addr_lo][value_hi][value_lo]
      // Function 0x10 (Write): [0x10][addr_hi][addr_lo][count_hi][count_lo]

      if (data.size() < 1)
      {
        ESP_LOGW(TAG, "Response too short: %d bytes", data.size());
        state_ = State::IDLE;
        if (error_callback_)
        {
          error_callback_(pending_command_, ErrorCode::PROTOCOL_ERROR);
        }
        return;
      }

      uint8_t function_code = data[0];

      // Check for Modbus exception response (error bit 0x80 set)
      if (function_code & 0x80)
      {
        uint8_t exception_code = (data.size() >= 2) ? data[1] : 0x00;
        ESP_LOGE(TAG, "Modbus exception: Function 0x%02X, Exception Code 0x%02X",
                 function_code & 0x7F, exception_code);
        state_ = State::IDLE;
        if (error_callback_)
        {
          error_callback_(pending_command_, ErrorCode::MODBUS_ERROR);
        }
        return;
      }

      // Verify function code matches what we sent
      if (function_code != pending_function_code_)
      {
        ESP_LOGW(TAG, "Function code mismatch: sent 0x%02X, received 0x%02X",
                 pending_function_code_, function_code);
        state_ = State::IDLE;
        if (error_callback_)
        {
          error_callback_(pending_command_, ErrorCode::PROTOCOL_ERROR);
        }
        return;
      }

      if (function_code == 0x04)
      {
        // Read Input Registers (Function 0x04)
        // Response format: [0x04][byte_count][data...]
        if (data.size() < 2)
        {
          ESP_LOGW(TAG, "Read response too short: %d bytes (expected at least 2)", data.size());
          state_ = State::IDLE;
          if (error_callback_)
          {
            error_callback_(pending_command_, ErrorCode::PROTOCOL_ERROR);
          }
          return;
        }

        uint8_t byte_count = data[1];
        uint8_t expected_register_count = pending_data_.empty() ? 1 : pending_data_[0];
        size_t expected_bytes = expected_register_count * 2;

        if (byte_count != expected_bytes)
        {
          ESP_LOGW(TAG, "Read response byte count mismatch: expected %d bytes, received %d",
                   expected_bytes, byte_count);
          state_ = State::IDLE;
          if (error_callback_)
          {
            error_callback_(pending_command_, ErrorCode::PROTOCOL_ERROR);
          }
          return;
        }

        // Extract data payload (skip function code and byte count)
        std::vector<uint8_t> payload(data.begin() + 2, data.end());
        if (payload.size() != byte_count)
        {
          ESP_LOGW(TAG, "Read response payload size mismatch: expected %d, got %d",
                   byte_count, payload.size());
          state_ = State::IDLE;
          if (error_callback_)
          {
            error_callback_(pending_command_, ErrorCode::PROTOCOL_ERROR);
          }
          return;
        }

        // Response data is valid - pass to callback
        state_ = State::IDLE;
        if (response_callback_)
        {
          response_callback_(pending_command_, payload);
        }
        ESP_LOGD(TAG, "Read response validated for command 0x%02X: %d bytes",
                 static_cast<uint8_t>(pending_command_), payload.size());
      }
      else if (function_code == 0x06 || function_code == 0x10)
      {
        // Write Single Register (0x06) or Write Multiple Registers (0x10)
        // Response format: [0x06/0x10][addr_hi][addr_lo][value/count_hi][value/count_lo]

        if (data.size() < 5)
        {
          ESP_LOGW(TAG, "Write response too short: %d bytes (expected 5)", data.size());
          state_ = State::IDLE;
          if (error_callback_)
          {
            error_callback_(pending_command_, ErrorCode::PROTOCOL_ERROR);
          }
          return;
        }

        // Extract and validate register address (skip function code)
        uint16_t response_address = (static_cast<uint16_t>(data[1]) << 8) | data[2];
        if (response_address != pending_register_address_)
        {
          ESP_LOGW(TAG, "Write response address mismatch: sent 0x%04X, received 0x%04X",
                   pending_register_address_, response_address);
          state_ = State::IDLE;
          if (error_callback_)
          {
            error_callback_(pending_command_, ErrorCode::PROTOCOL_ERROR);
          }
          return;
        }

        // Function-specific validation
        if (function_code == 0x06)
        {
          // Validate echoed value for Function 0x06
          if (pending_data_.size() == 2)
          {
            uint16_t response_value = (static_cast<uint16_t>(data[3]) << 8) | data[4];
            uint16_t sent_value = (static_cast<uint16_t>(pending_data_[0]) << 8) | pending_data_[1];

            if (response_value != sent_value)
            {
              ESP_LOGW(TAG, "Function 0x06 value mismatch: sent 0x%04X, received 0x%04X",
                       sent_value, response_value);
              state_ = State::IDLE;
              if (error_callback_)
              {
                error_callback_(pending_command_, ErrorCode::PROTOCOL_ERROR);
              }
              return;
            }
          }
        }
        else if (function_code == 0x10)
        {
          // Function 0x10: Response contains register count
          uint16_t response_count = (static_cast<uint16_t>(data[3]) << 8) | data[4];
          ESP_LOGV(TAG, "Function 0x10 validated: address 0x%04X, count %d",
                   response_address, response_count);
        }

        // Write response validated successfully
        state_ = State::IDLE;
        if (response_callback_)
        {
          response_callback_(pending_command_, std::vector<uint8_t>{}); // Empty data for writes
        }
        ESP_LOGD(TAG, "Write response validated for command 0x%02X (function 0x%02X)",
                 static_cast<uint8_t>(pending_command_), function_code);
      }
      else
      {
        ESP_LOGW(TAG, "Unexpected function code 0x%02X in handle_response", function_code);
        state_ = State::IDLE;
        if (error_callback_)
        {
          error_callback_(pending_command_, ErrorCode::PROTOCOL_ERROR);
        }
      }
    }

    void ModbusTransport::handle_error_response(uint8_t function_code, uint8_t exception_code)
    {
      // Modbus error response received - clear transport state immediately!
      // This prevents the 4-second timeout wait when motor rejects a command
      ESP_LOGW(TAG, "Modbus error for command 0x%02X: function=0x%02X, exception=%d",
               static_cast<uint8_t>(pending_command_), function_code, exception_code);

      state_ = State::IDLE;

      // Invoke error callback
      if (error_callback_)
      {
        error_callback_(pending_command_, ErrorCode::MODBUS_ERROR);
      }
    }

  } // namespace servoxxd
} // namespace esphome
