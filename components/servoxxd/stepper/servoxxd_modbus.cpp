#include "servoxxd_modbus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome
{
  namespace servoxxd
  {

    static const char *const TAG = "servoxxd.modbus";

    ModbusTransport::ModbusTransport(modbus::ModbusDevice *device, uint8_t slave_address)
        : device_(device), slave_address_(slave_address) {}

    Result ModbusTransport::execute_command(Command cmd, const std::vector<uint8_t> &data)
    {
      if (state_ != State::IDLE)
      {
        ESP_LOGW(TAG, "Transport busy, cannot execute command 0x%02X", static_cast<uint8_t>(cmd));
        return {false, ErrorCode::BUSY};
      }

      uint16_t register_address = command_to_register(cmd);

      // Modbus send() is void - we assume success and handle errors via callbacks
      // NOTE: ESPHome's send() for Function 0x06 ignores the third parameter (number_of_entities)
      // and expects the value in the payload parameter as 2 bytes (big-endian)!
      if (data.empty())
      {
        // Write single register with value 0 (enable/disable, stop, etc.)
        uint8_t payload[2] = {0x00, 0x00};
        device_->send(0x06, register_address, 0, 2, payload);
      }
      else if (data.size() <= 2)
      {
        // Write single register (Function 0x06)
        // Value must be passed as payload, not as number_of_entities!
        uint8_t payload[2];
        if (data.size() == 2)
        {
          payload[0] = data[0]; // High byte
          payload[1] = data[1]; // Low byte
        }
        else
        {
          payload[0] = 0x00;    // High byte = 0
          payload[1] = data[0]; // Low byte = value
        }
        device_->send(0x06, register_address, 0, 2, payload);
      }
      else
      {
        // Write multiple registers (Function 0x10)
        // Convert byte array to uint16 array for Modbus
        uint16_t register_count = (data.size() + 1) / 2;
        device_->send(0x10, register_address, register_count,
                      static_cast<uint8_t>(data.size()), data.data());
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
      case Command::READ_HOMING_STATUS:
        register_count = 1; // 2 bytes
        break;
      default:
        register_count = 1;
        break;
      }

      // Send read input registers (Function 0x04)
      device_->send(0x04, register_address, register_count, 0, nullptr);

      state_ = State::WAITING_READ;
      pending_command_ = cmd;
      timeout_start_ms_ = millis();

      ESP_LOGD(TAG, "Read command 0x%02X, register 0x%04X, count %d",
               static_cast<uint8_t>(cmd), register_address, register_count);

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

    void ModbusTransport::handle_read_response(const std::vector<uint8_t> &data)
    {
      state_ = State::IDLE;

      if (response_callback_)
      {
        response_callback_(pending_command_, data);
      }

      ESP_LOGD(TAG, "Read response for command 0x%02X: %d bytes",
               static_cast<uint8_t>(pending_command_), data.size());
    }

    void ModbusTransport::handle_write_response()
    {
      // For write commands, successful send means operation completed
      state_ = State::IDLE;

      // Invoke response callback with empty data for write operations
      if (response_callback_)
      {
        response_callback_(pending_command_, std::vector<uint8_t>{});
      }

      ESP_LOGD(TAG, "Write response for command 0x%02X",
               static_cast<uint8_t>(pending_command_));
    }

  } // namespace servoxxd
} // namespace esphome
