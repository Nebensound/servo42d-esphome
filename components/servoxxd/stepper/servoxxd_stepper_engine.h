#pragma once

#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "servoxxd_command_queue.h"
#include "servoxxd_commands.h"
#include "servoxxd_command_codec.h"
#include "servoxxd_transport.h"
#include "servoxxd_position.h"
#include "servoxxd_speed.h"
#include "servoxxd_acceleration.h"
#include <functional>
#include <cmath>
#include <optional>

namespace esphome
{
  namespace servoxxd
  {

    // Forward declaration
    class ServoXxd;

    // Import types from servoxxd_modbus
    using servoxxd::Acceleration;
    using servoxxd::AccelerationUnit;
    using servoxxd::CommandQueue;
    using servoxxd::Position;
    using servoxxd::PositionUnit;
    using servoxxd::Speed;
    using servoxxd::SpeedUnit;

    /**
     * @brief Core movement and state machine logic for Servo42D motor
     *
     * StepperEngine encapsulates all movement, homing, and stop logic as state machine.
     * It manages the CommandQueue, processes Modbus callbacks, and handles state transitions.
     *
     * Responsibilities:
     * - Processes all movement commands (move_to, stop, home, run_continuous)
     * - Monitors and controls internal states (Idle, Moving, Homing, Error, Disabled)
     * - Manages CommandQueue and processes Modbus callbacks (response, error, timeout)
     * - Regularly updates status values (encoder position, speed, protection) using hybrid strategy
     * - Validates commands based on current state using command validation matrix
     *
     * Design: Strict separation of concerns with ServoXxd parent (configuration, status, helpers)
     */
    class StepperEngine
    {
    public:
      /**
       * @brief State machine states
       *
       * States define what operations are allowed and how the motor responds to commands.
       * See specification 02-cpp-interface.md for complete state transition table.
       */
      enum class State
      {
        Disabled, // Motor disabled, no motion possible
        Idle,     // Motor ready, waiting for commands
        Moving,   // Position movement in progress (Position Mode only)
        Running,  // Continuous rotation in progress (Speed Mode only)
        Homing,   // Homing process in progress
        Stopping, // Controlled stop in progress (with deceleration)
        Error     // Error occurred (Protection, Modbus error, Timeout)
      };

      /**
       * @brief Constructor
       *
       * @param parent Pointer to parent ServoXxd component (configuration, helpers)
       * @param transport Pointer to ITransport for Layer 3 integration
       * @param command_timeout_ms Timeout for commands (default: 1000ms)
       * @param poll_interval_ms Polling interval for status updates (default: 200ms)
       */
      StepperEngine(ServoXxd *parent, ITransport *transport = nullptr,
                    uint32_t command_timeout_ms = 1000, uint32_t poll_interval_ms = 200);

      ~StepperEngine();

      /**
       * @brief Update method called cyclically from main loop
       *
       * Responsibilities:
       * - Process state machine transitions
       * - Execute CommandQueue update (timeouts, next command)
       * - Trigger status polling if interval elapsed
       * - Check for state-specific timeouts
       *
       * Must be called regularly (e.g., every 10-50ms) for responsive operation.
       */
      void update();

      // ============================================================================
      // Movement Commands
      // ============================================================================

      /**
       * @brief Move to absolute target position
       *
       * @param target Target position (absolute, with unit)
       * @param speed Optional movement speed (overrides default), std::nullopt = use default
       * @param accel Optional acceleration (overrides default), std::nullopt = use default
       *
       * Validation:
       * - Only allowed in Idle state (Position Mode only)
       * - During Moving/Stopping: replaces target (override behavior)
       * - Rejected in other states (Error, Disabled, Running, Homing)
       *
       * State transition: Idle → Moving
       */
      void move_to(const Position &target, std::optional<Speed> speed = std::nullopt,
                   std::optional<Acceleration> accel = std::nullopt);

      /**
       * @brief Stop motor with controlled deceleration
       *
       * @param decel Optional deceleration (overrides default), std::nullopt = use default
       *
       * Validation:
       * - Allowed in Moving, Running, Homing, Stopping states
       * - No-op in Idle (already stopped)
       * - Rejected in Disabled, Error states
       *
       * State transition: Moving/Running/Homing → Stopping → Idle (when speed = 0)
       */
      void stop(std::optional<Acceleration> decel = std::nullopt);

      /**
       * @brief Emergency stop - immediate halt without deceleration
       *
       * Allowed in all states. Clears command queue, disables motor, transitions to Error state.
       * Requires release_protection() or restart() to recover.
       *
       * State transition: * → Error
       */
      void emergency_stop();

      /**
       * @brief Start homing sequence
       *
       * Uses homing configuration from parent (mode, direction, speed, etc.)
       *
       * Validation:
       * - Only allowed in Idle state (Position Mode only)
       * - Rejected in other states
       *
       * State transition: Idle → Homing → Idle (when complete)
       */
      void home();

      /**
       * @brief Run motor continuously at specified speed
       *
       * @param speed Continuous rotation speed (positive = CCW, negative = CW), std::nullopt = use last/default
       * @param accel Acceleration for speed ramp, std::nullopt = use last/default
       *
       * Validation:
       * - Only allowed in Idle or Running states (Speed Mode only)
       * - Rejected in other states
       *
       * State transition: Idle → Running
       */
      void run_continuous(std::optional<Speed> speed = std::nullopt,
                          std::optional<Acceleration> accel = std::nullopt);

      // ============================================================================
      // Configuration Commands
      // ============================================================================

      /**
       * @brief Enable motor
       *
       * Validation:
       * - Only allowed in Disabled state
       * - Rejected in other states
       *
       * State transition: Disabled → Idle
       */
      void enable();

      /**
       * @brief Disable motor
       *
       * Validation:
       * - Allowed in Idle, Error states
       * - During motion (Moving/Running/Homing/Stopping): buffered, executed after stop
       * - Rejected in Disabled state (already disabled)
       *
       * State transition: Idle → Disabled (or via Stopping → Idle → Disabled)
       */
      void disable();

      /**
       * @brief Release protection and clear error state
       *
       * Validation:
       * - Allowed in Error state
       * - Also allowed in Disabled, Idle (no-op if no error)
       * - Rejected during motion
       *
       * State transition: Error → Idle (if protection cleared successfully)
       */
      void release_protection();

      /**
       * @brief Restart motor (full reset)
       *
       * Validation:
       * - Allowed in Disabled, Idle, Error states
       * - Rejected during motion
       *
       * Clears all errors, resets state, re-initializes motor.
       */
      void restart();

      /**
       * @brief Set current position as zero reference
       *
       * Validation:
       * - Only allowed in Idle state
       * - Rejected in other states
       */
      void set_zero();

      // ============================================================================
      // Status Queries
      // ============================================================================

      /**
       * @brief Get current motor position
       *
       * Returns last known encoder position (updated via polling or event-triggered queries).
       *
       * @return Current position with unit
       */
      Position get_current_position() const;

      /**
       * @brief Get current state
       *
       * @return Current state machine state
       */
      State get_state() const { return state_; }

      /**
       * @brief Get state name (for logging/debugging)
       *
       * @param state State to convert
       * @return Human-readable state name
       */
      static const char *state_to_string(State state);

      /**
       * @brief Check if motor is currently moving
       *
       * @return True if state is Moving, Running, Homing, or Stopping
       */
      bool is_moving() const;

      // ============================================================================
      // Callbacks
      // ============================================================================

      /**
       * @brief Register callback for position updates
       *
       * Callback is invoked when position changes significantly (threshold: 10 steps).
       *
       * @param cb Callback function
       */
      void set_position_update_callback(std::function<void(Position)> cb);

      /**
       * @brief Register callback for speed updates
       *
       * @param cb Callback function
       */
      void set_speed_update_callback(std::function<void(Speed)> cb);

      /**
       * @brief Register callback for protection triggered
       *
       * @param cb Callback function
       */
      void set_protection_callback(std::function<void()> cb);

      /**
       * @brief Register callback for motor enabled/disabled
       *
       * @param cb Callback function (parameter: true = enabled, false = disabled)
       */
      void set_motor_status_callback(std::function<void(bool)> cb);

      // ============================================================================
      // Transport Callbacks (Layer 4 Integration)
      // ============================================================================

      /**
       * @brief Process transport response
       *
       * Called by CommandQueue when a successful response is received from transport layer.
       * Decodes response data using ServoCommandCodec and updates internal state.
       *
       * @param cmd Command that generated this response
       * @param data Raw response data
       */
      void on_transport_response(Command cmd, const std::vector<uint8_t> &data);

      /**
       * @brief Process transport error
       *
       * Called by CommandQueue when transport layer reports an error (timeout, device error, etc.).
       * Handles retries, logs errors, and transitions to Error state if necessary.
       *
       * @param cmd Command that failed
       * @param error Error code
       */
      void on_transport_error(Command cmd, ErrorCode error);

    private:
      // ============================================================================
      // Private Members
      // ============================================================================

      ServoXxd *parent_;    ///< Parent component (configuration, helpers)
      CommandQueue *queue_; ///< Command queue for serial Modbus execution
      State state_;         ///< Current state machine state
      bool emergency_flag_; ///< Emergency stop flag (requires restart)

      // Position tracking
      Position current_position_; ///< Last known encoder position
      Position target_position_;  ///< Target position for move_to()
      int32_t encoder_carry_;     ///< Encoder carry value (for multi-turn tracking)
      uint16_t encoder_value_;    ///< Encoder value (0-16383, one revolution)

      // Status tracking
      Speed current_speed_;       ///< Last known motor speed (RPM)
      bool motor_enabled_;        ///< Motor enabled status
      bool protection_triggered_; ///< Protection triggered flag

      // Polling
      uint32_t poll_interval_ms_; ///< Polling interval for status updates
      uint32_t last_poll_time_;   ///< Last poll timestamp (millis)
      uint32_t state_enter_time_; ///< State entry timestamp for timeout tracking

      // Callbacks
      std::function<void(Position)> position_callback_;
      std::function<void(Speed)> speed_callback_;
      std::function<void()> protection_callback_;
      std::function<void(bool)> motor_status_callback_;

      // Buffered commands (for commands that need to be deferred)
      bool disable_pending_; ///< Disable command buffered (execute after stop)

      // ============================================================================
      // Private Methods - State Machine
      // ============================================================================

      /**
       * @brief Transition to new state
       *
       * Logs state change and performs state-specific initialization.
       *
       * @param new_state Target state
       */
      void transition_to(State new_state);

      /**
       * @brief Validate if command is allowed in current state
       *
       * @param command_name Command name (for logging)
       * @param allowed_states List of allowed states
       * @return True if command is allowed
       */
      bool validate_command(const char *command_name, std::initializer_list<State> allowed_states);

      /**
       * @brief Check state-specific timeouts
       *
       * Monitors movement timeout, homing timeout, etc.
       * Transitions to Error state if timeout exceeded.
       */
      void check_state_timeouts();

      // ============================================================================
      // Private Methods - Polling
      // ============================================================================

      /**
       * @brief Execute status polling cycle
       *
       * Queries encoder position, speed, motor status, protection status.
       * Called from update() when poll interval elapsed.
       */
      void execute_polling();

      /**
       * @brief Query encoder position (Command 0x30)
       *
       * Reads encoder carry + value, calculates absolute position.
       */
      void poll_encoder_position();

      /**
       * @brief Query motor speed (Command 0x32)
       *
       * Reads current speed in RPM (positive = CCW, negative = CW).
       */
      void poll_motor_speed();

      /**
       * @brief Query motor status (Command 0x3A)
       *
       * Reads motor enabled/disabled state.
       */
      void poll_motor_status();

      /**
       * @brief Query protection status (Command 0x3E)
       *
       * Reads locked-rotor protection state. Triggers Error state if protected.
       */
      void poll_protection_status();

      // ============================================================================
      // Private Methods - Event Processing
      // ============================================================================

      /**
       * @brief Process encoder position update
       *
       * Updates current_position_, checks target reached, invokes callback.
       *
       * @param carry Encoder carry (int32_t)
       * @param value Encoder value (uint16_t, 0-16383)
       */
      void process_encoder_update(int32_t carry, uint16_t value);

      /**
       * @brief Process speed update
       *
       * Updates current_speed_, checks standstill, invokes callback.
       *
       * @param speed_rpm Speed in RPM (int16_t)
       */
      void process_speed_update(int16_t speed_rpm);

      /**
       * @brief Process motor status update
       *
       * Updates motor_enabled_, invokes callback.
       *
       * @param enabled Motor enabled (true/false)
       */
      void process_motor_status_update(bool enabled);

      /**
       * @brief Process protection status update
       *
       * Updates protection_triggered_, transitions to Error if protected.
       *
       * @param protected_status Protection status (0 = OK, 1 = protected)
       */
      void process_protection_update(uint8_t protected_status);

      /**
       * @brief Check if target position reached
       *
       * Compares current_position_ with target_position_ (threshold tolerance).
       * Triggers transition Moving → Idle if reached.
       *
       * @return True if target reached
       */
      bool is_target_reached();

      /**
       * @brief Handle error condition
       *
       * Transitions to Error state, logs error, invokes callbacks.
       *
       * @param error_message Error description
       */
      void handle_error(const char *error_message);
    };

  } // namespace servoxxd
} // namespace esphome
