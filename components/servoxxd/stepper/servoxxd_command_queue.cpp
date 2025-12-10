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
            [this](const Command &cmd)
            {
              this->on_response(cmd);
            });

        transport_->set_error_callback(
            [this](const Command &cmd, ErrorCode error)
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
      if (queue_.empty() || queue_.front().state != CommandState::EXECUTING)
      {
        execute_next();
      }
    }

    void CommandQueue::enqueue(const Command &cmd, CommandCallback callback, Priority priority, uint32_t delay_before_next_ms, std::optional<bool> deduplicate)
    {
      // Determine deduplication: Auto-decision based on priority if not specified
      bool should_deduplicate = deduplicate.value_or(priority == Priority::BACKGROUND || priority == Priority::IDLE);

      // Deduplication: Check if identical command exists (only if enabled)
      if (should_deduplicate)
      {
        auto existing = find_duplicate(cmd);
        if (existing != queue_.end())
        {
          // Update existing command with new data
          existing->command.payload = cmd.payload;
          existing->priority = std::min(priority, existing->priority);
          existing->delay_before_next_ms = delay_before_next_ms;
          existing->callback = callback;
          return;
        }
      }

      QueuedCommand queued_cmd(cmd, callback, priority, delay_before_next_ms, millis());

      ESP_LOGD(TAG, "Enqueuing command 0x%02X (prio=%d, queue size: %zu)",
               static_cast<uint8_t>(cmd.command_type), static_cast<uint8_t>(priority), queue_.size());
      queue_.push_back(queued_cmd);

      // Try to execute immediately if idle
      execute_next();
    }

    void CommandQueue::on_response(const Command &response_cmd)
    {
      if (queue_.empty() || queue_.front().state != CommandState::EXECUTING)
      {
        ESP_LOGW(TAG, "Unexpected response for command 0x%02X (no executing command)",
                 static_cast<uint8_t>(response_cmd.command_type));
        return;
      }

      auto &current_cmd = queue_.front();
      if (current_cmd.command.command_type != response_cmd.command_type)
      {
        ESP_LOGW(TAG, "Response mismatch: expected 0x%02X, got 0x%02X",
                 static_cast<uint8_t>(current_cmd.command.command_type), static_cast<uint8_t>(response_cmd.command_type));
        return;
      }

      ESP_LOGD(TAG, "Command 0x%02X completed (%zu bytes)",
               static_cast<uint8_t>(response_cmd.command_type), response_cmd.response.size());

      // Invoke callback
      if (current_cmd.callback)
      {
        current_cmd.callback(true, response_cmd);
      }

      // Set delay for next command if specified
      if (current_cmd.delay_before_next_ms > 0)
      {
        delay_until_ms_ = millis() + current_cmd.delay_before_next_ms;
        ESP_LOGD(TAG, "  Delaying next command for %ums", current_cmd.delay_before_next_ms);
      }
      else
      {
        delay_until_ms_ = 0;
      }

      // Remove completed command
      queue_.pop_front();

      // Tail-recursive processing: execute next command
      execute_next();
    }

    void CommandQueue::on_error(const Command &error_cmd, ErrorCode error)
    {
      if (queue_.empty() || queue_.front().state != CommandState::EXECUTING)
      {
        ESP_LOGW(TAG, "Unexpected error for command 0x%02X (no executing command)",
                 static_cast<uint8_t>(error_cmd.command_type));
        return;
      }

      auto &current_cmd = queue_.front();
      if (current_cmd.command.command_type != error_cmd.command_type)
      {
        ESP_LOGW(TAG, "Error mismatch: expected 0x%02X, got 0x%02X",
                 static_cast<uint8_t>(current_cmd.command.command_type), static_cast<uint8_t>(error_cmd.command_type));
        return;
      }

      ESP_LOGE(TAG, "Command 0x%02X failed: error %d",
               static_cast<uint8_t>(error_cmd.command_type), static_cast<int>(error));

      // Invoke callback with failure
      if (current_cmd.callback)
      {
        Command empty_cmd(error_cmd.command_type);
        current_cmd.callback(false, empty_cmd);
      }

      // Remove failed command
      queue_.pop_front();

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
      size_t start_index = (!queue_.empty() && queue_.front().state == CommandState::EXECUTING) ? 1 : 0;

      ESP_LOGW(TAG, "Clearing %zu pending commands", queue_.size() - start_index);

      // Invoke callbacks for all cleared commands
      for (size_t i = start_index; i < queue_.size(); i++)
      {
        if (queue_[i].callback)
        {
          Command empty_cmd(queue_[i].command.command_type);
          queue_[i].callback(false, empty_cmd);
        }
      }

      // Remove pending commands (keep executing command if present)
      // Note: Can't use erase() or resize() because Command has const members
      if (start_index > 0)
      {
        // Keep only the executing command (first element) - rebuild queue
        std::deque<QueuedCommand> new_queue;
        new_queue.push_back(queue_[0]);
        queue_ = std::move(new_queue);
      }
      else
      {
        queue_.clear();
      }
    }

    void CommandQueue::execute_next()
    {
      // Early exit: No commands to execute
      if (queue_.empty())
      {
        return;
      }

      // From spec: "Single-flight guarantee - only one EXECUTING command at a time"
      if (queue_.front().state == CommandState::EXECUTING)
      {
        return; // Commandtype already executing
      }

      // Check if we need to delay before executing next command
      if (delay_until_ms_ > 0 && millis() < delay_until_ms_)
      {
        return; // Still waiting for delay to elapse
      }

      // Prepare next command (sorts by effective time with age-based penalties)
      prepare_next_command();

      auto &cmd = queue_.front();

      // Transition to EXECUTING (single-flight guarantee)
      cmd.state = CommandState::EXECUTING;
      cmd.sent_time = millis();

      // Send via transport - execute_command handles both read (0x04) and write (0x06/0x10)
      transport_->execute_command(cmd.command);

      ESP_LOGD(TAG, "Executing command 0x%02X (prio=%d, age=%ums, queue depth: %zu)",
               static_cast<uint8_t>(cmd.command.command_type), static_cast<uint8_t>(cmd.priority),
               millis() - cmd.enqueued_time, queue_.size());
    }

    void CommandQueue::prepare_next_command()
    {
      // Time-penalty based scheduling: Find command with lowest effective_time
      if (queue_.empty())
        return;

      uint32_t now = millis();
      auto best = queue_.begin();
      uint32_t best_effective_time = calculate_effective_time(*best, now);

      // Find command with smallest effective time (executes first)
      for (auto it = queue_.begin() + 1; it != queue_.end(); ++it)
      {
        uint32_t effective_time = calculate_effective_time(*it, now);
        if (effective_time < best_effective_time)
        {
          best = it;
          best_effective_time = effective_time;
        }
      }

      // Move best command to front if not already there
      if (best != queue_.begin())
      {
        // Can't use erase() with const members - rebuild queue with best element first
        auto index = std::distance(queue_.begin(), best);
        std::deque<QueuedCommand> new_queue;
        
        new_queue.push_back(*best); // Best command first
        for (size_t i = 0; i < queue_.size(); ++i)
        {
          if (i != static_cast<size_t>(index))
          {
            new_queue.push_back(queue_[i]);
          }
        }
        queue_ = std::move(new_queue);

        // Log if non-FIFO reordering happened
        if (queue_[0].priority == Priority::CRITICAL)
        {
          ESP_LOGW(TAG, "Moving CRITICAL command 0x%02X to front (unexpected position)",
                   static_cast<uint8_t>(queue_[0].command.command_type));
        }
        else if (queue_[0].priority == Priority::BACKGROUND)
        {
          ESP_LOGD(TAG, "Prioritizing BACKGROUND command 0x%02X (age=%ums, effective=%u)",
                   static_cast<uint8_t>(queue_[0].command.command_type),
                   now - queue_[0].enqueued_time,
                   best_effective_time);
        }
      }
    }

    uint32_t CommandQueue::calculate_effective_time(const QueuedCommand &cmd, uint32_t now)
    {
      switch (cmd.priority)
      {
      case Priority::CRITICAL:
        return 0; // Always first (smallest value)

      case Priority::NORMAL:
        return cmd.enqueued_time; // FIFO after CRITICAL

      case Priority::BACKGROUND:
        return cmd.enqueued_time + BACKGROUND_PENALTY_MS;

      case Priority::IDLE:
      default:
        return cmd.enqueued_time + IDLE_PENALTY_MS;
      }
    }

    void CommandQueue::check_timeout()
    {
      if (queue_.empty() || queue_.front().state != CommandState::EXECUTING)
      {
        return; // No executing command
      }

      auto &current_cmd = queue_.front();
      uint32_t elapsed = millis() - current_cmd.sent_time;

      if (elapsed > timeout_ms_)
      {
        ESP_LOGW(TAG, "Command 0x%02X timed out after %ums (timeout=%ums)",
                 static_cast<uint8_t>(current_cmd.command.command_type), elapsed, timeout_ms_);

        // Invoke callback with failure
        if (current_cmd.callback)
        {
          Command empty_cmd(current_cmd.command.command_type);
          current_cmd.callback(false, empty_cmd);
        }

        // Remove timed-out command
        queue_.pop_front();

        // Tail-recursive processing: continue with next command
        execute_next();
      }
    }

    std::deque<CommandQueue::QueuedCommand>::iterator
    CommandQueue::find_duplicate(const Command &cmd)
    {
      // Find duplicate command for deduplication
      // Requirements:
      // 1. Same Command (type AND payload)
      // 2. PENDING state (not EXECUTING or completed)
      // 3. Skip first command if it's EXECUTING

      if (queue_.empty())
      {
        return queue_.end();
      }

      size_t start_index = (queue_.front().state == CommandState::EXECUTING) ? 1 : 0;

      for (size_t i = start_index; i < queue_.size(); ++i)
      {
        auto &queued = queue_[i];
        // Match both command type and payload for true duplicate detection
        if (queued.command.command_type == cmd.command_type &&
            queued.command.payload == cmd.payload &&
            queued.state == CommandState::PENDING)
        {
          return queue_.begin() + i;
        }
      }

      return queue_.end();
    }

  } // namespace servoxxd
} // namespace esphome
