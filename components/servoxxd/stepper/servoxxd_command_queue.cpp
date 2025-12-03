#include "servoxxd_command_queue.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome
{
  namespace servoxxd
  {

    static const char *const TAG = "servoxxd.queue";

    CommandQueue::CommandQueue(ITransport *transport, uint32_t timeout_ms)
        : transport_(transport), timeout_ms_(timeout_ms)
    {
      ESP_LOGCONFIG(TAG, "CommandQueue initialized: timeout=%ums", timeout_ms_);
      
      // Register callbacks with transport layer
      if (transport_)
      {
        transport_->set_response_callback(
            [this](Command cmd, const std::vector<uint8_t> &data)
            {
              this->on_response(cmd, data);
            });
        
        transport_->set_error_callback(
            [this](Command cmd, ErrorCode error)
            {
              this->on_error(cmd, error);
            });
        
        ESP_LOGD(TAG, "Registered callbacks with transport layer");
      }
    }

    void CommandQueue::update()
    {
      // From spec: "Called each loop iteration to detect stuck commands"
      check_timeout();

      // Opportunistic execution: start next command if idle
      if (!execution_guard_)
      {
        execute_next();
      }
    }

    void CommandQueue::enqueue(Command cmd, const std::vector<uint8_t> &data,
                               CommandCallback callback, bool priority)
    {
      // Check for EMERGENCY_STOP - clear all pending commands
      if (cmd == Command::EMERGENCY_STOP)
      {
        ESP_LOGW(TAG, "EMERGENCY_STOP: clearing %zu pending commands", queue_.size());
        clear();
      }

      QueuedCommand queued_cmd(cmd, data, callback);

      if (priority)
      {
        // From spec: "Priority commands insert at front (after executing)"
        if (execution_guard_ && !queue_.empty())
        {
          ESP_LOGD(TAG, "Enqueuing priority command 0x%02X (after executing)",
                   static_cast<uint8_t>(cmd));
          queue_.insert(queue_.begin() + 1, queued_cmd);
        }
        else
        {
          ESP_LOGD(TAG, "Enqueuing priority command 0x%02X (front)",
                   static_cast<uint8_t>(cmd));
          queue_.push_front(queued_cmd);
        }
      }
      else
      {
        ESP_LOGD(TAG, "Enqueuing command 0x%02X (queue size: %zu)",
                 static_cast<uint8_t>(cmd), queue_.size());
        queue_.push_back(queued_cmd);
      }

      // Try to execute immediately if idle
      execute_next();
    }

    void CommandQueue::enqueue_read(Command cmd, CommandCallback callback)
    {
      // From spec: "Deduplication: If identical read command exists, merge callbacks"
      auto existing = find_duplicate_read(cmd);
      if (existing != queue_.end())
      {
        ESP_LOGD(TAG, "Deduplicating read command 0x%02X (merging callbacks)",
                 static_cast<uint8_t>(cmd));
        existing->callbacks.push_back(callback);
        return;
      }

      // No data needed for read commands
      enqueue(cmd, std::vector<uint8_t>{}, callback, false);
    }

    void CommandQueue::on_response(Command cmd, const std::vector<uint8_t> &data)
    {
      if (queue_.empty() || !execution_guard_)
      {
        ESP_LOGW(TAG, "Unexpected response for command 0x%02X (no executing command)",
                 static_cast<uint8_t>(cmd));
        return;
      }

      auto &current_cmd = queue_.front();
      if (current_cmd.command != cmd)
      {
        ESP_LOGW(TAG, "Response mismatch: expected 0x%02X, got 0x%02X",
                 static_cast<uint8_t>(current_cmd.command), static_cast<uint8_t>(cmd));
        return;
      }

      ESP_LOGD(TAG, "Command 0x%02X completed (%zu bytes)",
               static_cast<uint8_t>(cmd), data.size());

      // Invoke all callbacks (for deduplicated commands)
      for (auto &cb : current_cmd.callbacks)
      {
        if (cb)
        {
          cb(true, data);
        }
      }

      // Remove completed command
      queue_.pop_front();
      commands_completed_++;

      // CRITICAL: Clear execution guard to enable next command
      execution_guard_ = false;

      // Tail-recursive processing: execute next command
      execute_next();
    }

    void CommandQueue::on_error(Command cmd, ErrorCode error)
    {
      if (queue_.empty() || !execution_guard_)
      {
        ESP_LOGW(TAG, "Unexpected error for command 0x%02X (no executing command)",
                 static_cast<uint8_t>(cmd));
        return;
      }

      auto &current_cmd = queue_.front();
      if (current_cmd.command != cmd)
      {
        ESP_LOGW(TAG, "Error mismatch: expected 0x%02X, got 0x%02X",
                 static_cast<uint8_t>(current_cmd.command), static_cast<uint8_t>(cmd));
        return;
      }

      ESP_LOGE(TAG, "Command 0x%02X failed: error %d",
               static_cast<uint8_t>(cmd), static_cast<int>(error));

      // Invoke all callbacks with failure
      for (auto &cb : current_cmd.callbacks)
      {
        if (cb)
        {
          cb(false, std::vector<uint8_t>{});
        }
      }

      // Remove failed command
      queue_.pop_front();
      commands_failed_++;

      // CRITICAL: Clear execution guard to enable recovery
      execution_guard_ = false;

      // Tail-recursive processing: continue with next command
      execute_next();
    }

    void CommandQueue::clear()
    {
      // From spec: "Clear all pending (non-executing) commands"
      if (queue_.empty())
      {
        return;
      }

      // If executing, skip first command (it must complete/timeout naturally)
      size_t start_index = execution_guard_ ? 1 : 0;

      ESP_LOGW(TAG, "Clearing %zu pending commands", queue_.size() - start_index);

      // Invoke callbacks for all cleared commands
      for (size_t i = start_index; i < queue_.size(); i++)
      {
        for (auto &cb : queue_[i].callbacks)
        {
          if (cb)
          {
            cb(false, std::vector<uint8_t>{});
          }
        }
      }

      // Remove pending commands (keep executing command if present)
      if (start_index > 0)
      {
        queue_.erase(queue_.begin() + 1, queue_.end());
      }
      else
      {
        queue_.clear();
      }
    }

    void CommandQueue::execute_next()
    {
      // From spec: "Check execution guard - return immediately if set"
      if (execution_guard_)
      {
        return; // Command already executing
      }

      if (queue_.empty())
      {
        return; // No commands to execute
      }

      auto &cmd = queue_.front();

      // Transition to EXECUTING
      cmd.state = CommandState::EXECUTING;
      cmd.sent_time = millis();

      // Set execution guard (single-flight guarantee)
      execution_guard_ = true;

      // Send via transport
      send_command(cmd);

      ESP_LOGD(TAG, "Executing command 0x%02X (queue depth: %zu)",
               static_cast<uint8_t>(cmd.command), queue_.size());

      commands_sent_++;
    }

    void CommandQueue::send_command(QueuedCommand &cmd)
    {
      // Determine if read or write based on Command enum
      if (is_read_command(cmd.command))
      {
        transport_->read_command(cmd.command);
      }
      else
      {
        transport_->execute_command(cmd.command, cmd.data);
      }
    }

    void CommandQueue::check_timeout()
    {
      if (queue_.empty() || !execution_guard_)
      {
        return; // No executing command
      }

      auto &current_cmd = queue_.front();
      uint32_t elapsed = millis() - current_cmd.sent_time;

      if (elapsed > timeout_ms_)
      {
        ESP_LOGW(TAG, "Command 0x%02X timed out after %ums",
                 static_cast<uint8_t>(current_cmd.command), elapsed);

        // Invoke callbacks with failure
        for (auto &cb : current_cmd.callbacks)
        {
          if (cb)
          {
            cb(false, std::vector<uint8_t>{});
          }
        }

        // Remove timed-out command
        queue_.pop_front();
        commands_timeout_++;

        // CRITICAL: Clear execution guard to prevent queue stall
        execution_guard_ = false;

        // Tail-recursive processing: continue with next command
        execute_next();
      }
    }

    std::deque<CommandQueue::QueuedCommand>::iterator
    CommandQueue::find_duplicate_read(Command cmd)
    {
      // From spec: "Deduplication only for read commands (same Command enum)"
      if (!is_read_command(cmd))
      {
        return queue_.end();
      }

      for (auto it = queue_.begin(); it != queue_.end(); ++it)
      {
        if (it->command == cmd && it->state == CommandState::PENDING)
        {
          return it;
        }
      }

      return queue_.end();
    }

  } // namespace servoxxd
} // namespace esphome
