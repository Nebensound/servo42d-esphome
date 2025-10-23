#include "servo42d_base_command.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome
{
  namespace servo42d_rs485
  {

    static const char *TAG = "servo42d_rs485.command";

    BaseCommand::BaseCommand(uint16_t register_address, uint32_t timeout_ms)
        : register_address_(register_address), state_(CommandState::PENDING),
          timeout_ms_(timeout_ms), start_time_(0)
    {
    }

    bool BaseCommand::is_timeout() const
    {
      return (millis() - start_time_) > timeout_ms_;
    }

    void BaseCommand::reset_timeout()
    {
      start_time_ = millis();
    }

    void BaseCommand::on_error(uint8_t function_code, uint8_t exception_code)
    {
      ESP_LOGW(TAG, "Command %s failed: function=0x%02X, exception=0x%02X",
               get_command_name(), function_code, exception_code);
      state_ = CommandState::FAILED;
      trigger_completion(false);
    }

    void BaseCommand::trigger_completion(bool success)
    {
      if (completion_callback_)
      {
        completion_callback_(this, success);
      }
    }

    void BaseCommand::trigger_data_callback(const std::vector<uint8_t> &data)
    {
      if (data_callback_)
      {
        data_callback_(data);
      }
    }

  } // namespace servo42d_rs485
} // namespace esphome
