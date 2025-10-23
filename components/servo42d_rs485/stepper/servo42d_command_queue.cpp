#include "servo42d_command_queue.h"
#include "esphome/components/modbus/modbus.h"
#include "esphome/core/log.h"

namespace esphome
{
  namespace servo42d_rs485
  {

    using esphome::modbus::ModbusDevice;

    static const char *TAG = "servo42d_rs485.queue";

    CommandQueue::CommandQueue()
    {
    }

    void CommandQueue::enqueue(std::unique_ptr<BaseCommand> command)
    {
      if (!command)
      {
        ESP_LOGW(TAG, "Attempted to enqueue null command");
        return;
      }

      ESP_LOGV(TAG, "Enqueuing command: %s (register=0x%04X)",
               command->get_command_name(), command->get_register_address());

      queue_.push_back(std::move(command));
    }

    void CommandQueue::process_next()
    {
      // Complete current command if finished
      if (current_command_ && current_command_->is_finished())
      {
        bool success = current_command_->get_state() == CommandState::COMPLETED;
        ESP_LOGV(TAG, "Command %s finished: %s",
                 current_command_->get_command_name(), success ? "SUCCESS" : "FAILED");
        current_command_ = nullptr;
      }

      // Start next command if available and no current command
      if (!current_command_ && !queue_.empty())
      {
        current_command_ = queue_.front().get();
        queue_.pop_front();
      }
    }

    void CommandQueue::execute_next(ModbusDevice *device)
    {
      if (!current_command_ || !device)
      {
        return;
      }

      if (current_command_->get_state() == CommandState::PENDING)
      {
        ESP_LOGV(TAG, "Executing command: %s", current_command_->get_command_name());
        bool success = current_command_->execute(device);
        if (!success)
        {
          ESP_LOGW(TAG, "Command execution failed: %s", current_command_->get_command_name());
          current_command_->set_state(CommandState::FAILED);
        }
        else
        {
          current_command_->set_state(CommandState::EXECUTING);
        }
      }
    }

    void CommandQueue::clear()
    {
      queue_.clear();
      current_command_ = nullptr;
      ESP_LOGD(TAG, "Queue cleared");
    }

    bool CommandQueue::is_empty() const
    {
      return queue_.empty() && !current_command_;
    }

    size_t CommandQueue::size() const
    {
      return queue_.size() + (current_command_ ? 1 : 0);
    }

    BaseCommand *CommandQueue::get_current_command()
    {
      return current_command_;
    }

    bool CommandQueue::has_executing_command() const
    {
      return current_command_ != nullptr;
    }

    void CommandQueue::handle_response(const std::vector<uint8_t> &data)
    {
      if (!current_command_)
      {
        ESP_LOGW(TAG, "Received response but no command is executing");
        return;
      }

      ESP_LOGV(TAG, "Handling response for command: %s", current_command_->get_command_name());
      current_command_->process_response(data);
    }

    void CommandQueue::handle_error(uint8_t function_code, uint8_t exception_code)
    {
      if (!current_command_)
      {
        ESP_LOGW(TAG, "Received error but no command is executing");
        return;
      }

      ESP_LOGW(TAG, "Command error: %s (func=0x%02X, exception=0x%02X)",
               current_command_->get_command_name(), function_code, exception_code);

      current_command_->on_error(function_code, exception_code);
      current_command_->set_state(CommandState::FAILED);
    }

    void CommandQueue::handle_timeout()
    {
      if (!current_command_)
      {
        return;
      }

      ESP_LOGW(TAG, "Command timeout: %s", current_command_->get_command_name());
      current_command_->set_state(CommandState::TIMEOUT);
    }

  } // namespace servo42d_rs485
} // namespace esphome
