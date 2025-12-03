#pragma once

#include "esphome/core/log.h"
#include "servoxxd_transport.h"
#include "servoxxd_commands.h"
#include <vector>
#include <deque>
#include <functional>
#include <memory>

namespace esphome
{
  namespace servoxxd
  {

    /**
     * @brief Command state for state machine
     *
     * From spec: "Each command follows a lifecycle state machine"
     */
    enum class CommandState : uint8_t
    {
      PENDING = 0, // Command in queue, not yet sent
      EXECUTING,   // Command sent to transport, waiting for response
      COMPLETED,   // Response received and processed successfully
      FAILED,      // Transport error received
      TIMEOUT      // No response within timeout period
    };

    /**
     * @brief Command queue for serialized transport execution
     *
     * From spec (02c-layer3-command-queue.md):
     * - Single-flight execution: Only one command in EXECUTING state at any time
     * - Timeout handling: Abort commands on timeout, advance to next
     * - Deduplication: Coalesce multiple identical read commands
     * - Priority: Emergency commands (EMERGENCY_STOP) clear pending queue
     * - Callback support: Notify when command completes (success or failure)
     *
     * **Single-Flight Guarantee:**
     * - execution_guard_ flag acts as mutex
     * - execute_next() returns immediately if guard is set
     * - Response/error handlers clear guard and call execute_next()
     * - Timeout recovery clears guard to prevent queue stall
     *
     * **Integration with Layer 4:**
     * - Uses ITransport interface for protocol-agnostic communication
     * - Commands identified by Command enum, not raw function codes
     * - Transport callbacks forwarded to StepperEngine (Layer 2)
     *
     * @see docs/specification/02c-layer3-command-queue.md
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
      using CommandCallback = std::function<void(bool success, const std::vector<uint8_t> &)>;

      /**
       * @brief Construct a new Command Queue object
       *
       * @param transport Transport layer interface (Layer 4)
       * @param timeout_ms Default timeout for commands in milliseconds (default: 1000ms)
       */
      CommandQueue(ITransport *transport, uint32_t timeout_ms = 1000);

      /**
       * @brief Update the command queue (called from ServoXxd::loop())
       *
       * From spec: "Called each loop iteration to detect stuck commands"
       * - Check timeout on current executing command
       * - Opportunistic execution: start next command if queue idle
       */
      void update();

      /**
       * @brief Enqueue a command
       *
       * @param cmd Command enum value
       * @param data Command payload (encoded by ServoCommandCodec)
       * @param callback Callback to invoke when command completes
       * @param priority If true, add to front of queue (emergency)
       */
      void enqueue(Command cmd, const std::vector<uint8_t> &data, CommandCallback callback,
                   bool priority = false);

      /**
       * @brief Enqueue a read command
       *
       * From spec: "Deduplication: If identical read command exists, merge callbacks"
       *
       * @param cmd Command enum value (READ_*)
       * @param callback Callback to invoke when command completes
       */
      void enqueue_read(Command cmd, CommandCallback callback);

      /**
       * @brief Handle command response (from ITransport callback)
       *
       * From spec: "Completes command, clears guard, calls execute_next()"
       *
       * @param cmd Command that completed
       * @param data Response data from transport
       */
      void on_response(Command cmd, const std::vector<uint8_t> &data);

      /**
       * @brief Handle command error (from ITransport callback)
       *
       * From spec: "Fails command, clears guard, continues"
       *
       * @param cmd Command that failed
       * @param error Error code from transport
       */
      void on_error(Command cmd, ErrorCode error);

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
       */
      struct QueuedCommand
      {
        Command command;                        // Command enum value
        std::vector<uint8_t> data;              // Command payload
        std::vector<CommandCallback> callbacks; // Multiple callbacks for deduplicated commands
        CommandState state;                     // State machine state
        uint32_t sent_time;                     // millis() when sent (for timeout)

        QueuedCommand(Command cmd, const std::vector<uint8_t> &payload, CommandCallback cb)
            : command(cmd),
              data(payload),
              state(CommandState::PENDING),
              sent_time(0)
        {
          callbacks.push_back(cb);
        }
      };

      // Queue and execution state
      std::deque<QueuedCommand> queue_; // FIFO queue (deque for front insertion)
      bool execution_guard_{false};     // Single-flight execution guard
      ITransport *transport_{nullptr};  // Transport layer interface

      // Configuration
      uint32_t timeout_ms_{1000}; // Default timeout (1 second)

      // Statistics (for debugging)
      uint32_t commands_sent_{0};
      uint32_t commands_completed_{0};
      uint32_t commands_failed_{0};
      uint32_t commands_timeout_{0};

      /**
       * @brief Execute next pending command (if not already executing)
       *
       * From spec: "checks execution guard, sends next command if idle"
       * - Check execution_guard_ flag (single-flight guarantee)
       * - If executing: return immediately
       * - If queue empty: return
       * - Get next PENDING command, transition to EXECUTING, send via transport
       */
      void execute_next();

      /**
       * @brief Send a command via transport
       *
       * @param cmd Queued command to send
       * - Determine if read or write based on Command enum
       * - Call transport->execute_command() or transport->read_command()
       * - Set sent_time for timeout tracking
       */
      void send_command(QueuedCommand &cmd);

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
       * From spec: "Deduplication only for read commands (same Command enum)"
       * - Compare Command enum values
       * - Returns iterator to existing command if duplicate, queue_.end() otherwise
       */
      std::deque<QueuedCommand>::iterator find_duplicate_read(Command cmd);

      /**
       * @brief Check if a Command is a read operation
       */
      bool is_read_command(Command cmd) const
      {
        return cmd == Command::READ_ENCODER_CARRY ||
               cmd == Command::READ_ENCODER_ADDITION ||
               cmd == Command::READ_CURRENT_SPEED ||
               cmd == Command::READ_PULSE_COUNT ||
               cmd == Command::READ_IO_STATUS ||
               cmd == Command::READ_ANGLE_ERROR ||
               cmd == Command::READ_MOTOR_STATUS ||
               cmd == Command::READ_HOMING_STATUS ||
               cmd == Command::READ_PROTECTION_STATUS;
      }
    };

  } // namespace servoxxd
} // namespace esphome
