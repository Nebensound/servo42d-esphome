#include "servoxxd_command_queue.h"
#include "servoxxd.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome
{
  namespace servoxxd_modbus
  {

    static const char *const TAG = "servoxxd_modbus.queue";

    CommandQueue::CommandQueue(ServoXxdModbus *parent, uint32_t timeout_ms, uint8_t max_retries)
        : parent_(parent), timeout_ms_(timeout_ms), max_retries_(max_retries)
    {
      ESP_LOGCONFIG(TAG, "CommandQueue initialized: timeout=%ums, max_retries=%u", timeout_ms_, max_retries_);
    }

    void CommandQueue::enqueue_read(uint16_t address, uint16_t count, CommandCallback callback, bool priority)
    {
      // From spec: "Deduplication: If identical read command exists, merge callbacks"
      auto existing = find_duplicate_read(address, count);
      if (existing != queue_.end())
      {
        ESP_LOGD(TAG, "Deduplicating read command: addr=0x%04X, count=%u (merging callbacks)", address, count);
        existing->callbacks.push_back(callback);
        return;
      }

      // Create new read command (function code 0x04)
      Command cmd(0x04, address, count, callback);

      if (priority)
      {
        // From spec: "Priority commands insert at front (after in-flight)"
        // If executing, insert at position 1 (after executing command at position 0)
        // If not executing, insert at position 0
        if (is_executing_ && !queue_.empty())
        {
          ESP_LOGD(TAG, "Enqueuing priority read: addr=0x%04X, count=%u (after executing command)", address, count);
          queue_.insert(queue_.begin() + 1, cmd); // Insert after executing command
        }
        else
        {
          ESP_LOGD(TAG, "Enqueuing priority read: addr=0x%04X, count=%u (front of queue)", address, count);
          queue_.push_front(cmd);
        }
      }
      else
      {
        ESP_LOGD(TAG, "Enqueuing read: addr=0x%04X, count=%u (queue size: %zu)", address, count, queue_.size());
        queue_.push_back(cmd);
      }

      // Try to execute immediately if idle
      execute_next();
    }

    void CommandQueue::enqueue_write(uint16_t address, uint16_t value, CommandCallback callback, bool priority)
    {
      // Create write command (function code 0x06) - no deduplication for writes
      Command cmd(0x06, address, value, callback);

      if (priority)
      {
        // From spec: "Priority commands insert at front (after in-flight)"
        if (is_executing_ && !queue_.empty())
        {
          ESP_LOGD(TAG, "Enqueuing priority write: addr=0x%04X, value=0x%04X (after executing command)", address, value);
          queue_.insert(queue_.begin() + 1, cmd);
        }
        else
        {
          ESP_LOGD(TAG, "Enqueuing priority write: addr=0x%04X, value=0x%04X (front of queue)", address, value);
          queue_.push_front(cmd);
        }
      }
      else
      {
        ESP_LOGD(TAG, "Enqueuing write: addr=0x%04X, value=0x%04X (queue size: %zu)", address, value, queue_.size());
        queue_.push_back(cmd);
      }

      // Try to execute immediately if idle
      execute_next();
    }

    void CommandQueue::enqueue_multi_write(uint16_t address, const std::vector<uint16_t> &values,
                                           CommandCallback callback, bool priority)
    {
      // Create multi-write command (function code 0x10) - no deduplication
      Command cmd(0x10, address, values, callback);

      if (priority)
      {
        // From spec: "Priority commands insert at front (after in-flight)"
        if (is_executing_ && !queue_.empty())
        {
          ESP_LOGD(TAG, "Enqueuing priority multi-write: addr=0x%04X, count=%zu (after executing command)", address,
                   values.size());
          queue_.insert(queue_.begin() + 1, cmd);
        }
        else
        {
          ESP_LOGD(TAG, "Enqueuing priority multi-write: addr=0x%04X, count=%zu (front of queue)", address,
                   values.size());
          queue_.push_front(cmd);
        }
      }
      else
      {
        ESP_LOGD(TAG, "Enqueuing multi-write: addr=0x%04X, count=%zu (queue size: %zu)", address, values.size(),
                 queue_.size());
        queue_.push_back(cmd);
      }

      // Try to execute immediately if idle
      execute_next();
    }

    void CommandQueue::update()
    {
      // From spec: "Called each loop iteration to detect stuck commands"
      check_timeout();

      // From spec: "Opportunistic execution: start next command if queue idle"
      execute_next();
    }

    void CommandQueue::on_response(const std::vector<uint8_t> &data)
    {
      // From spec: "Completes command, clears guard, calls execute_next()"

      if (!is_executing_)
      {
        ESP_LOGW(TAG, "Received response but no command executing (unexpected response)");
        return;
      }

      if (queue_.empty())
      {
        ESP_LOGW(TAG, "Received response but queue is empty (should not happen)");
        is_executing_ = false;
        return;
      }

      // Get executing command (should be front of queue)
      Command &cmd = queue_.front();

      if (cmd.state != CommandState::EXECUTING)
      {
        ESP_LOGW(TAG, "Received response but front command not in EXECUTING state (state=%d)", (int)cmd.state);
        is_executing_ = false;
        return;
      }

      // Transition to COMPLETED
      cmd.state = CommandState::COMPLETED;
      commands_completed_++;

      ESP_LOGD(TAG, "Command completed: fc=0x%02X, addr=0x%04X, time=%ums", cmd.function_code, cmd.address,
               (uint32_t)(millis() - cmd.sent_time));

      // Invoke all callbacks with success=true
      for (auto &callback : cmd.callbacks)
      {
        if (callback)
        {
          callback(true, data);
        }
      }

      // Remove completed command from queue
      queue_.pop_front();

      // Clear execution guard
      is_executing_ = false;

      // Process next command
      execute_next();
    }

    void CommandQueue::on_error(uint8_t function_code, uint8_t exception_code)
    {
      // From spec: "Fails command, clears guard, continues"

      if (!is_executing_)
      {
        ESP_LOGW(TAG, "Received error but no command executing (unexpected error)");
        return;
      }

      if (queue_.empty())
      {
        ESP_LOGW(TAG, "Received error but queue is empty (should not happen)");
        is_executing_ = false;
        return;
      }

      // Get executing command
      Command &cmd = queue_.front();

      ESP_LOGE(TAG, "Command failed: fc=0x%02X, addr=0x%04X, exception=0x%02X, retry=%u/%u", cmd.function_code,
               cmd.address, exception_code, cmd.retry_count, max_retries_);

      // Check if we should retry
      if (cmd.retry_count < max_retries_)
      {
        cmd.retry_count++;
        cmd.state = CommandState::PENDING; // Reset to pending for retry
        is_executing_ = false;             // Clear guard for retry
        ESP_LOGD(TAG, "Retrying command: retry %u/%u", cmd.retry_count, max_retries_);
        execute_next(); // Try again immediately
        return;
      }

      // Max retries reached - transition to FAILED
      cmd.state = CommandState::FAILED;
      commands_failed_++;

      // Invoke all callbacks with success=false
      std::vector<uint8_t> empty_data;
      for (auto &callback : cmd.callbacks)
      {
        if (callback)
        {
          callback(false, empty_data);
        }
      }

      // Remove failed command from queue
      queue_.pop_front();

      // Clear execution guard
      is_executing_ = false;

      // Process next command
      execute_next();
    }

    void CommandQueue::clear()
    {
      // From spec: "Clear all pending (non-executing) commands"
      ESP_LOGW(TAG, "Clearing command queue (size: %zu, executing: %d)", queue_.size(), is_executing_);

      // Invoke all callbacks with success=false (except executing command)
      std::vector<uint8_t> empty_data;
      size_t cleared = 0;

      // If executing, skip first command (it must complete naturally)
      auto start_it = queue_.begin();
      if (is_executing_ && !queue_.empty())
      {
        start_it++; // Skip executing command
      }

      // Clear all pending commands
      for (auto it = start_it; it != queue_.end(); ++it)
      {
        for (auto &callback : it->callbacks)
        {
          if (callback)
          {
            callback(false, empty_data);
          }
        }
        cleared++;
      }

      // Erase pending commands (keep executing command if present)
      if (is_executing_ && !queue_.empty())
      {
        queue_.erase(start_it, queue_.end());
      }
      else
      {
        queue_.clear();
      }

      ESP_LOGD(TAG, "Cleared %zu pending commands", cleared);
    }

    void CommandQueue::execute_next()
    {
      // From spec: "checks execution guard, sends next command if idle"

      // Single-flight guarantee: only one command executing at a time
      if (is_executing_)
      {
        return; // Already executing, wait for response/error/timeout
      }

      // Check if queue is empty
      if (queue_.empty())
      {
        return; // Nothing to execute
      }

      // Get next pending command
      Command &cmd = queue_.front();

      // Verify it's in PENDING state (should be, but check anyway)
      if (cmd.state != CommandState::PENDING)
      {
        ESP_LOGW(TAG, "Front command not in PENDING state (state=%d), removing", (int)cmd.state);
        queue_.pop_front();
        execute_next(); // Try next command
        return;
      }

      // Transition to EXECUTING
      cmd.state = CommandState::EXECUTING;
      cmd.sent_time = millis();
      is_executing_ = true; // Set execution guard

      ESP_LOGD(TAG, "Executing command: fc=0x%02X, addr=0x%04X, count/value=%u (queue: %zu)", cmd.function_code,
               cmd.address, cmd.count_or_value, queue_.size());

      // Send command to Modbus device
      send_command(cmd);
      commands_sent_++;
    }

    void CommandQueue::send_command(Command &cmd)
    {
      // From spec: "Format payload based on function code (0x04, 0x06, 0x10)"

      if (!parent_)
      {
        ESP_LOGE(TAG, "Cannot send command: parent is null");
        return;
      }

      // Format Modbus command based on function code
      switch (cmd.function_code)
      {
      case 0x04: // Read Input Registers
      {
        // Function: 0x04, Address: 2 bytes, Count: 2 bytes
        ESP_LOGD(TAG, "Sending Read Input Registers: addr=0x%04X, count=%u", cmd.address, cmd.count_or_value);
        parent_->send(cmd.function_code, cmd.address, cmd.count_or_value);
        break;
      }

      case 0x06: // Write Single Register
      {
        // Function: 0x06, Address: 2 bytes, Value: 2 bytes
        ESP_LOGD(TAG, "Sending Write Single Register: addr=0x%04X, value=0x%04X", cmd.address, cmd.count_or_value);
        parent_->send(cmd.function_code, cmd.address, cmd.count_or_value);
        break;
      }

      case 0x10: // Write Multiple Registers
      {
        // Function: 0x10, Address: 2 bytes, Count: 2 bytes, Byte count: 1 byte, Values: N*2 bytes
        ESP_LOGD(TAG, "Sending Write Multiple Registers: addr=0x%04X, count=%zu", cmd.address,
                 cmd.multi_values.size());

        // Build raw payload for multi-write
        std::vector<uint8_t> payload;
        payload.push_back(cmd.function_code);                // Function code
        payload.push_back((cmd.address >> 8) & 0xFF);        // Address high byte
        payload.push_back(cmd.address & 0xFF);               // Address low byte
        payload.push_back((cmd.count_or_value >> 8) & 0xFF); // Count high byte (number of registers)
        payload.push_back(cmd.count_or_value & 0xFF);        // Count low byte
        payload.push_back(cmd.count_or_value * 2);           // Byte count (2 bytes per register)

        // Add register values (big-endian)
        for (uint16_t value : cmd.multi_values)
        {
          payload.push_back((value >> 8) & 0xFF); // Value high byte
          payload.push_back(value & 0xFF);        // Value low byte
        }

        parent_->send_raw(payload);
        break;
      }

      default:
        ESP_LOGE(TAG, "Unknown function code: 0x%02X", cmd.function_code);
        break;
      }
    }

    void CommandQueue::check_timeout()
    {
      // From spec: "Called periodically to detect stuck commands"

      if (!is_executing_)
      {
        return; // No command executing, nothing to timeout
      }

      if (queue_.empty())
      {
        // Should not happen, but handle gracefully
        ESP_LOGW(TAG, "Executing flag set but queue is empty, clearing flag");
        is_executing_ = false;
        return;
      }

      // Get executing command
      Command &cmd = queue_.front();

      // Check if timeout exceeded
      uint32_t elapsed = millis() - cmd.sent_time;
      if (elapsed > timeout_ms_)
      {
        ESP_LOGW(TAG, "Command timeout: fc=0x%02X, addr=0x%04X, elapsed=%ums > timeout=%ums", cmd.function_code,
                 cmd.address, elapsed, timeout_ms_);

        // Transition to TIMEOUT
        cmd.state = CommandState::TIMEOUT;
        commands_timeout_++;

        // Invoke all callbacks with success=false
        std::vector<uint8_t> empty_data;
        for (auto &callback : cmd.callbacks)
        {
          if (callback)
          {
            callback(false, empty_data);
          }
        }

        // Remove timeout command from queue
        queue_.pop_front();

        // Clear execution guard
        is_executing_ = false;

        // Process next command
        execute_next();
      }
    }

    std::deque<CommandQueue::Command>::iterator CommandQueue::find_duplicate_read(uint16_t address, uint16_t count)
    {
      // From spec: "Deduplication only for read commands (same address+count)"

      // Search queue for duplicate read command (including EXECUTING commands)
      for (auto it = queue_.begin(); it != queue_.end(); ++it)
      {
        // Must be read command (0x04)
        if (it->function_code != 0x04)
        {
          continue;
        }

        // Check address and count match
        if (it->address == address && it->count_or_value == count)
        {
          return it; // Found duplicate (could be PENDING or EXECUTING)
        }
      }

      return queue_.end(); // No duplicate found
    }

  } // namespace servoxxd_modbus
} // namespace esphome
