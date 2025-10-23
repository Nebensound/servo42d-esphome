#include "command_queue.h"
#include "esphome/components/modbus/modbus.h"
#include "esphome/core/log.h"

namespace esphome
{
  namespace servo42d_rs485
  {

    static const char *TAG = "servo42d_rs485.queue";

    CommandQueue::CommandQueue() : device_(nullptr)
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

      // Start processing if no command is currently executing
      if (!current_command_)
      {
        process_next();
      }
    }

    void CommandQueue::process_next()
    {
      // Complete current command if finished
      if (current_command_ && current_command_->is_finished())
      {
        bool success = current_command_->get_state() == CommandState::COMPLETED;
        ESP_LOGV(TAG, "Command %s finished: %s",
                 current_command_->get_command_name(), success ? "SUCCESS" : "FAILED");
        current_command_.reset();
      }

      // Start next command if available and no current command
      if (!current_command_ && !queue_.empty())
      {
        current_command_ = std::move(queue_.front());
        queue_.pop_front();
        execute_current_command();
      }
    }

    void CommandQueue::execute_current_command()
    {
      if (!current_command_)
      {
        return;
      }

      if (!device_)
      {
        ESP_LOGE(TAG, "No device set for command execution");
        current_command_->set_state(CommandState::FAILED);
        return;
      }

      ESP_LOGV(TAG, "Executing command: %s", current_command_->get_command_name());

      // Set completion callback to automatically process next command
      current_command_->set_completion_callback([this](BaseCommand *cmd, bool success)
                                                {
                                                  ESP_LOGV(TAG, "Command completed: %s (%s)",
                                                           cmd->get_command_name(), success ? "SUCCESS" : "FAILED");
                                                  // Process next command on next loop iteration
                                                });

      if (!current_command_->execute(device_))
      {
        ESP_LOGW(TAG, "Failed to execute command: %s", current_command_->get_command_name());
        current_command_->set_state(CommandState::FAILED);
      }
    }

    void CommandQueue::clear()
    {
      ESP_LOGV(TAG, "Clearing command queue (%zu commands)", queue_.size());
      queue_.clear();

      if (current_command_)
      {
        ESP_LOGV(TAG, "Cancelling current command: %s", current_command_->get_command_name());
        current_command_.reset();
      }
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
      return current_command_.get();
    }

    bool CommandQueue::has_executing_command() const
    {
      return current_command_ && current_command_->get_state() == CommandState::EXECUTING;
    }

    void CommandQueue::handle_response(const std::vector<uint8_t> &data)
    {
      if (!current_command_)
      {
        ESP_LOGW(TAG, "Received response but no command is executing");
        return;
      }

      if (current_command_->get_state() != CommandState::EXECUTING)
      {
        ESP_LOGW(TAG, "Received response but command is not in executing state");
        return;
      }

      ESP_LOGV(TAG, "Processing response for command: %s", current_command_->get_command_name());
      current_command_->process_response(data);
    }

    void CommandQueue::handle_error(uint8_t function_code, uint8_t exception_code)
    {
      if (!current_command_)
      {
        ESP_LOGW(TAG, "Received error but no command is executing");
        return;
      }

      ESP_LOGW(TAG, "Handling error for command: %s (func=0x%02X, exc=0x%02X)",
               current_command_->get_command_name(), function_code, exception_code);
      current_command_->on_error(function_code, exception_code);
    }

    void CommandQueue::check_timeouts()
    {
      if (current_command_)
      {
        current_command_->check_timeout();
      }
    }

  }
}

} // namespace servo42d_rs485
} // namespace esphome
} // namespace esphome