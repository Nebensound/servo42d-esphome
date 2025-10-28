#pragma once

#include "esphome/core/log.h"
#include <vector>
#include <deque>
#include <functional>
#include <memory>

namespace esphome
{
  namespace servoxxd_modbus
  {

    // Forward declaration
    class ServoXxdModbus;

    /**
     * @brief Command state for state machine
     *
     * From spec: "Each Modbus command follows a lifecycle state machine"
     */
    enum class CommandState : uint8_t
    {
      PENDING = 0, // Command in queue, not yet sent
      EXECUTING,   // Command sent to hardware, waiting for response
      COMPLETED,   // Response received and processed successfully
      FAILED,      // Modbus error received (exception code)
      TIMEOUT      // No response within timeout period
    };

    /**
     * @brief Command queue for serial Modbus execution
     *
     * From spec (02-cpp-interface.md):
     * - Single-flight execution: Only one command in EXECUTING state at any time
     * - Timeout handling: Abort commands on timeout, advance to next
     * - Deduplication: Coalesce multiple identical read commands
     * - Priority: Emergency commands clear pending queue
     * - Callback support: Notify when command completes (success or failure)
     *
     * **Command Types:**
     * - Read commands (function code 0x04): Query registers
     * - Write commands (function code 0x06): Write single register
     * - Multi-write commands (function code 0x10): Write multiple registers
     *
     * **Queue Behavior:**
     * - Normal commands: Added to back of queue
     * - Priority commands: Clear pending queue (emergency)
     * - Deduplication: If identical read command exists, merge callbacks
     * - Timeout: Callback with error, advance to next command
     *
     * **Single-Flight Guarantee:**
     * - execution_guard_ flag acts as mutex
     * - execute_next() returns immediately if guard is set
     * - Response/error handlers clear guard and call execute_next()
     * - Timeout recovery clears guard to prevent queue stall
     *
     * @see 02-cpp-interface.md section "CommandQueue"
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
       * @brief Construct a new Command Queue object
       *
       * @param parent Parent ServoXxdModbus instance for sending Modbus commands
       * @param timeout_ms Default timeout for commands in milliseconds (default: 1000ms)
       * @param max_retries Maximum retry count for failed commands (default: 3)
       */
      CommandQueue(ServoXxdModbus *parent, uint32_t timeout_ms = 1000, uint8_t max_retries = 3);

      /**
       * @brief Update the command queue (called from loop())
       *
       * From spec: "Called each loop iteration to detect stuck commands"
       * - Check timeout on current executing command
       * - Opportunistic execution: start next command if queue idle
       */
      void update();

      /**
       * @brief Enqueue a read command (function code 0x04)
       *
       * From spec: "Deduplication: If identical read command exists, merge callbacks"
       *
       * @param address Register address to read
       * @param count Number of registers to read
       * @param callback Callback to invoke when command completes
       * @param priority If true, add to front of queue (emergency)
       */
      void enqueue_read(uint16_t address, uint16_t count, CommandCallback callback, bool priority = false);

      /**
       * @brief Enqueue a write command (function code 0x06)
       *
       * @param address Register address to write
       * @param value Value to write (single register)
       * @param callback Callback to invoke when command completes
       * @param priority If true, add to front of queue (emergency)
       */
      void enqueue_write(uint16_t address, uint16_t value, CommandCallback callback, bool priority = false);

      /**
       * @brief Enqueue a multi-write command (function code 0x10)
       *
       * @param address Starting register address
       * @param values Vector of values to write (multiple registers)
       * @param callback Callback to invoke when command completes
       * @param priority If true, add to front of queue (emergency)
       */
      void enqueue_multi_write(uint16_t address, const std::vector<uint16_t> &values, CommandCallback callback,
                               bool priority = false);

      /**
       * @brief Handle command response (called from ServoXxdModbus::on_modbus_data)
       *
       * From spec: "Completes command, clears guard, calls execute_next()"
       *
       * @param data Response data from Modbus device
       */
      void on_response(const std::vector<uint8_t> &data);

      /**
       * @brief Handle command error (called from ServoXxdModbus::on_modbus_error)
       *
       * From spec: "Fails command, clears guard, continues"
       *
       * @param function_code Modbus function code that failed
       * @param exception_code Modbus exception code (error reason)
       */
      void on_error(uint8_t function_code, uint8_t exception_code);

      /**
       * @brief Clear all pending commands
       *
       * From spec: "emergency_stop shall clear all pending (non-executing) commands"
       * - Invoke all callbacks with success=false
       * - Clear queue (except currently executing command)
       * - Does NOT clear executing command (that must complete/timeout naturally)
       */
      void clear();

      /**
       * @brief Check if queue is empty
       */
      bool is_empty() const { return queue_.empty(); }

      /**
       * @brief Get queue size (includes executing command if present)
       */
      size_t size() const { return queue_.size(); }

    private:
      /**
       * @brief Command structure with state machine
       *
       * From spec (02-cpp-interface.md): "Each Modbus command follows a lifecycle state machine"
       */
      struct Command
      {
        uint8_t function_code;                  // 0x04, 0x06, 0x10
        uint16_t address;                       // Register address
        uint16_t count_or_value;                // For reads: count, for write: value
        std::vector<uint16_t> multi_values;     // For multi-write (0x10)
        std::vector<CommandCallback> callbacks; // Multiple callbacks for deduplicated commands
        CommandState state;                     // State machine state
        uint32_t sent_time;                     // millis() when sent (for timeout)
        uint8_t retry_count;                    // Current retry attempt

        Command(uint8_t fc, uint16_t addr, uint16_t count_val, CommandCallback cb)
            : function_code(fc),
              address(addr),
              count_or_value(count_val),
              state(CommandState::PENDING),
              sent_time(0),
              retry_count(0)
        {
          callbacks.push_back(cb);
        }

        Command(uint8_t fc, uint16_t addr, const std::vector<uint16_t> &values, CommandCallback cb)
            : function_code(fc),
              address(addr),
              count_or_value(values.size()),
              multi_values(values),
              state(CommandState::PENDING),
              sent_time(0),
              retry_count(0)
        {
          callbacks.push_back(cb);
        }
      };

      // Queue and execution state
      std::deque<Command> queue_;       // FIFO queue (deque for front insertion)
      bool is_executing_{false};        // Single-flight execution guard
      ServoXxdModbus *parent_{nullptr}; // Parent for sending Modbus commands

      // Configuration
      uint32_t timeout_ms_{1000}; // Default timeout (1 second)
      uint8_t max_retries_{3};    // Maximum retry count

      // Statistics (optional, for debugging)
      uint32_t commands_sent_{0};
      uint32_t commands_completed_{0};
      uint32_t commands_failed_{0};
      uint32_t commands_timeout_{0};

      /**
       * @brief Execute next pending command (if not already executing)
       *
       * From spec: "checks execution guard, sends next command if idle"
       * - Check is_executing_ flag (single-flight guarantee)
       * - If executing: return immediately
       * - If queue empty: return
       * - Get next PENDING command, transition to EXECUTING, send via Modbus
       */
      void execute_next();

      /**
       * @brief Send a command to Modbus device
       *
       * @param cmd Command to send
       * - Format payload based on function code (0x04, 0x06, 0x10)
       * - Call parent->send() with formatted data
       * - Set sent_time for timeout tracking
       */
      void send_command(Command &cmd);

      /**
       * @brief Check for timeout on executing command
       *
       * From spec: "Called periodically in update() to detect stuck commands"
       * - Check if current command exceeded timeout_ms_
       * - If timeout: transition to TIMEOUT state, invoke callbacks, clear guard
       */
      void check_timeout();

      /**
       * @brief Check if a read command is a duplicate
       *
       * From spec: "Deduplication only for read commands (same address+count)"
       * - Compare function code (must be 0x04)
       * - Compare address and count_or_value
       * - Returns iterator to existing command if duplicate, queue_.end() otherwise
       */
      std::deque<Command>::iterator find_duplicate_read(uint16_t address, uint16_t count);
    };

  } // namespace servoxxd_modbus
} // namespace esphome
