#pragma once

#include "esphome/core/log.h"
#include <vector>
#include <queue>
#include <functional>

namespace esphome
{
  namespace servoxxd_modbus
  {

    /**
     * @brief Command queue for serial Modbus execution
     *
     * This class ensures that Modbus commands are executed sequentially, one at a time,
     * with proper timeout handling and deduplication.
     *
     * **Design Goals:**
     * - Serial execution: Only one command in-flight at a time
     * - Timeout handling: Abort and retry or skip commands on timeout
     * - Deduplication: Coalesce multiple identical commands (e.g., position queries)
     * - Priority: Emergency commands (stop, emergency_stop) jump to front
     * - Callback support: Notify when command completes (success or failure)
     *
     * **Command Types:**
     * - Read commands (function code 0x04): Query registers
     * - Write commands (function code 0x06): Write single register
     * - Multi-write commands (function code 0x10): Write multiple registers
     *
     * **Queue Behavior:**
     * - Normal commands: Added to back of queue
     * - Priority commands: Added to front of queue (after current in-flight)
     * - Deduplication: If identical command exists in queue, merge callbacks
     * - Timeout: If no response within timeout, callback with error, retry or skip
     *
     * TODO: Implementation required
     * - [ ] Command structure (function code, address, data, callback)
     * - [ ] Queue management (enqueue, dequeue, priority, deduplication)
     * - [ ] Execution logic (send command, wait for response, timeout)
     * - [ ] Callback invocation (success, error, timeout)
     * - [ ] Integration with ModbusDevice (send command, receive response)
     * - [ ] Retry logic (configurable retry count)
     * - [ ] Statistics (commands sent, timeouts, errors)
     *
     * @see ServoXxdModbus for parent class with Modbus communication
     * @see StepperEngine for command queue user
     */
    class CommandQueue
    {
    public:
      /**
       * @brief Command callback signature
       *
       * @param success True if command succeeded, false on error/timeout
       * @param data Response data (for read commands), empty for write commands
       */
      using CommandCallback = std::function<void(bool success, const std::vector<uint8_t> &data)>;

      /**
       * @brief Construct a CommandQueue
       *
       * TODO:
       * - Initialize queue
       * - Set default timeout (e.g., 500ms)
       * - Set default retry count (e.g., 3)
       */
      CommandQueue() { /* TODO */ }

      /**
       * @brief Update queue (called from StepperEngine::update)
       *
       * TODO:
       * - Check if command in-flight
       * - If no command in-flight and queue not empty: send next command
       * - Check for timeout on in-flight command
       * - Handle timeout: callback with error, retry or skip
       */
      void update() { /* TODO */ }

      /**
       * @brief Enqueue a read command (function code 0x04)
       *
       * TODO:
       * - Create command structure
       * - Check for duplicate in queue (same address, same function)
       * - If duplicate: merge callbacks
       * - If not duplicate: add to back of queue
       * - If priority: add to front of queue
       */
      void enqueue_read(uint16_t address, uint16_t count, CommandCallback callback, bool priority = false) { /* TODO */ }

      /**
       * @brief Enqueue a write command (function code 0x06)
       *
       * TODO:
       * - Create command structure
       * - Add to queue (no deduplication for writes)
       * - If priority: add to front
       */
      void enqueue_write(uint16_t address, uint16_t value, CommandCallback callback, bool priority = false) { /* TODO */ }

      /**
       * @brief Enqueue a multi-write command (function code 0x10)
       *
       * TODO:
       * - Create command structure
       * - Add to queue (no deduplication for writes)
       * - If priority: add to front
       */
      void enqueue_multi_write(uint16_t address, const std::vector<uint16_t> &values, CommandCallback callback,
                               bool priority = false)
      {
        /* TODO */
      }

      /**
       * @brief Handle command response (called from ServoXxdModbus::on_modbus_data)
       *
       * TODO:
       * - Match response to in-flight command
       * - Parse response data
       * - Invoke callback with success=true
       * - Clear in-flight command
       */
      void on_response(const std::vector<uint8_t> &data) { /* TODO */ }

      /**
       * @brief Handle command error (called from ServoXxdModbus::on_modbus_error)
       *
       * TODO:
       * - Invoke callback with success=false
       * - Retry or skip command based on retry count
       * - Clear in-flight command if not retrying
       */
      void on_error(uint8_t function_code, uint8_t exception_code) { /* TODO */ }

      /**
       * @brief Clear all pending commands
       *
       * TODO:
       * - Invoke all callbacks with success=false
       * - Clear queue
       * - Clear in-flight command
       */
      void clear() { /* TODO */ }

      /**
       * @brief Check if queue is empty
       */
      bool is_empty() const { /* TODO: return queue_.empty(); */ return true; }

      /**
       * @brief Get queue size
       */
      size_t size() const { /* TODO: return queue_.size(); */ return 0; }

    private:
      // TODO: Add member variables:
      // - struct Command {
      //     uint8_t function_code;
      //     uint16_t address;
      //     uint16_t count_or_value;
      //     std::vector<uint16_t> multi_values;
      //     CommandCallback callback;
      //     uint32_t sent_time;
      //     uint8_t retry_count;
      //   };
      // - std::queue<Command> queue_;
      // - Command* in_flight_{nullptr};
      // - uint32_t timeout_ms_{500};
      // - uint8_t max_retries_{3};
      // - Statistics stats_;

      /**
       * @brief Send a command to Modbus
       *
       * TODO:
       * - Format command based on function code
       * - Send via parent->send_modbus_command()
       * - Set sent_time for timeout tracking
       */
      void send_command(/* Command& cmd */) { /* TODO */ }

      /**
       * @brief Check if a command is a duplicate of another
       *
       * TODO:
       * - Compare function code, address
       * - For reads: same if address and count match
       * - For writes: never duplicate
       */
      bool is_duplicate(/* const Command& a, const Command& b */) const { /* TODO */ return false; }
    };

  } // namespace servoxxd_modbus
} // namespace esphome
