#include "servoxxd_stepper_engine.h"
// #include "servoxxd.h"  // TODO: Create when needed

namespace esphome
{
  namespace servoxxd
  {

    static const char *TAG_ENGINE = "servoxxd.stepper_engine";

    // ============================================================================
    // Constructor / Destructor
    // ============================================================================

    StepperEngine::StepperEngine(ServoXxd *parent, uint32_t command_timeout_ms,
                                 uint8_t max_retries, uint32_t poll_interval_ms)
        : parent_(parent),
          queue_(nullptr),
          state_(State::Disabled),
          emergency_flag_(false),
          current_position_(0.0f, PositionUnit::STEPS, parent),
          target_position_(0.0f, PositionUnit::STEPS, parent),
          encoder_carry_(0),
          encoder_value_(0),
          current_speed_(0.0f, SpeedUnit::RPM, parent),
          motor_enabled_(false),
          protection_triggered_(false),
          poll_interval_ms_(poll_interval_ms),
          last_poll_time_(0),
          disable_pending_(false)
    {

      // Create CommandQueue (parent will be cast to ServoXxd in actual usage)
      queue_ = new CommandQueue(parent, command_timeout_ms, max_retries);

      ESP_LOGD(TAG_ENGINE, "StepperEngine initialized: timeout=%ums, retries=%u, poll=%ums",
               command_timeout_ms, max_retries, poll_interval_ms);
    }

    StepperEngine::~StepperEngine()
    {
      if (queue_)
      {
        delete queue_;
        queue_ = nullptr;
      }
    }

    // ============================================================================
    // Main Update Loop
    // ============================================================================

    void StepperEngine::update()
    {
      // 1. Update CommandQueue (process timeouts, execute next command)
      if (queue_)
      {
        queue_->update();
      }

      // 2. Check state-specific timeouts
      check_state_timeouts();

      // 3. Execute polling if interval elapsed
      uint32_t now = millis();
      if (now - last_poll_time_ >= poll_interval_ms_)
      {
        execute_polling();
        last_poll_time_ = now;
      }

      // 4. Process buffered commands
      if (disable_pending_ && state_ == State::Idle)
      {
        disable_pending_ = false;
        disable(); // Execute buffered disable
      }
    }

    // ============================================================================
    // Movement Commands
    // ============================================================================

    void StepperEngine::move_to(Position target, const Speed *speed,
                                const Acceleration *accel)
    {
      // Target override behavior: allowed in Moving/Stopping states
      if (state_ == State::Moving || state_ == State::Stopping)
      {
        ESP_LOGD(TAG_ENGINE, "move_to(): Override current movement with new target");
        target_position_ = target;
        // TODO: Send new target to hardware (Command 0x??), update immediately
        return;
      }

      // Validation: only allowed in Idle state (Position Mode)
      if (!validate_command("move_to", {State::Idle}))
      {
        return;
      }

      // TODO: Check if Position Mode enabled (query parent configuration)

      target_position_ = target;

      ESP_LOGD(TAG_ENGINE, "move_to(): target=%s, speed=%s, accel=%s",
               target.to_string().c_str(),
               speed ? speed->to_string().c_str() : "default",
               accel ? accel->to_string().c_str() : "default");

      // TODO: Send move command via queue
      // queue_->enqueue_write(...);

      transition_to(State::Moving);
    }

    void StepperEngine::stop(const Acceleration *decel)
    {
      // Validation: allowed in Moving, Running, Homing, Stopping states
      if (state_ == State::Idle)
      {
        ESP_LOGD(TAG_ENGINE, "stop(): Already stopped, no-op");
        return;
      }

      if (!validate_command("stop", {State::Moving, State::Running, State::Homing, State::Stopping}))
      {
        return;
      }

      ESP_LOGD(TAG_ENGINE, "stop(): decel=%s", decel ? decel->to_string().c_str() : "default");

      // TODO: Send stop command via queue
      // queue_->enqueue_write(...);

      transition_to(State::Stopping);
    }

    void StepperEngine::emergency_stop()
    {
      ESP_LOGW(TAG_ENGINE, "emergency_stop(): Immediate halt, clearing queue");

      emergency_flag_ = true;

      // Clear command queue
      if (queue_)
      {
        queue_->clear();
      }

      // TODO: Send emergency stop command to hardware
      // Disable motor immediately

      transition_to(State::Error);
    }

    void StepperEngine::home()
    {
      // Validation: only allowed in Idle state (Position Mode)
      if (!validate_command("home", {State::Idle}))
      {
        return;
      }

      // TODO: Check if Position Mode enabled (query parent configuration)

      ESP_LOGD(TAG_ENGINE, "home(): Starting homing sequence");

      // TODO: Query homing configuration from parent (mode, direction, speed)
      // TODO: Send homing command via queue

      transition_to(State::Homing);
    }

    void StepperEngine::run_continuous(Speed speed, Acceleration accel)
    {
      // Validation: allowed in Idle or Running states (Speed Mode)
      if (!validate_command("run_continuous", {State::Idle, State::Running}))
      {
        return;
      }

      // TODO: Check if Speed Mode enabled (query parent configuration)

      ESP_LOGD(TAG_ENGINE, "run_continuous(): speed=%s, accel=%s",
               speed.to_string().c_str(), accel.to_string().c_str());

      // TODO: Send speed command via queue

      transition_to(State::Running);
    }

    // ============================================================================
    // Configuration Commands
    // ============================================================================

    void StepperEngine::enable()
    {
      if (!validate_command("enable", {State::Disabled}))
      {
        return;
      }

      ESP_LOGD(TAG_ENGINE, "enable(): Enabling motor");

      // TODO: Send enable command via queue

      transition_to(State::Idle);
    }

    void StepperEngine::disable()
    {
      // During motion: buffer the command
      if (state_ == State::Moving || state_ == State::Running ||
          state_ == State::Homing || state_ == State::Stopping)
      {
        ESP_LOGD(TAG_ENGINE, "disable(): Motor moving, buffering disable command");
        disable_pending_ = true;
        stop(); // Stop first
        return;
      }

      if (!validate_command("disable", {State::Idle, State::Error}))
      {
        return;
      }

      ESP_LOGD(TAG_ENGINE, "disable(): Disabling motor");

      // TODO: Send disable command via queue

      transition_to(State::Disabled);
    }

    void StepperEngine::release_protection()
    {
      if (!validate_command("release_protection", {State::Error, State::Idle, State::Disabled}))
      {
        return;
      }

      if (!protection_triggered_ && !emergency_flag_)
      {
        ESP_LOGD(TAG_ENGINE, "release_protection(): No protection/emergency active, no-op");
        return;
      }

      ESP_LOGD(TAG_ENGINE, "release_protection(): Clearing protection/emergency");

      protection_triggered_ = false;
      emergency_flag_ = false;

      // TODO: Send release protection command via queue

      if (state_ == State::Error)
      {
        transition_to(State::Idle);
      }
    }

    void StepperEngine::restart()
    {
      if (!validate_command("restart", {State::Disabled, State::Idle, State::Error}))
      {
        return;
      }

      ESP_LOGD(TAG_ENGINE, "restart(): Full motor restart");

      // Clear all errors
      protection_triggered_ = false;
      emergency_flag_ = false;

      // Clear queue
      if (queue_)
      {
        queue_->clear();
      }

      // TODO: Send restart command to hardware

      transition_to(State::Disabled);
    }

    void StepperEngine::set_zero()
    {
      if (!validate_command("set_zero", {State::Idle}))
      {
        return;
      }

      ESP_LOGD(TAG_ENGINE, "set_zero(): Setting current position as zero");

      // TODO: Send set zero command via queue

      // Update local position tracking
      current_position_ = Position(0.0f, PositionUnit::STEPS, parent_);
      encoder_carry_ = 0;
      encoder_value_ = 0;
    }

    // ============================================================================
    // Status Queries
    // ============================================================================

    Position StepperEngine::get_current_position() const
    {
      return current_position_;
    }

    bool StepperEngine::is_moving() const
    {
      return state_ == State::Moving || state_ == State::Running ||
             state_ == State::Homing || state_ == State::Stopping;
    }

    const char *StepperEngine::state_to_string(State state)
    {
      switch (state)
      {
      case State::Disabled:
        return "Disabled";
      case State::Idle:
        return "Idle";
      case State::Moving:
        return "Moving";
      case State::Running:
        return "Running";
      case State::Homing:
        return "Homing";
      case State::Stopping:
        return "Stopping";
      case State::Error:
        return "Error";
      default:
        return "Unknown";
      }
    }

    // ============================================================================
    // Callbacks Registration
    // ============================================================================

    void StepperEngine::set_position_update_callback(std::function<void(Position)> cb)
    {
      position_callback_ = cb;
    }

    void StepperEngine::set_speed_update_callback(std::function<void(Speed)> cb)
    {
      speed_callback_ = cb;
    }

    void StepperEngine::set_protection_callback(std::function<void()> cb)
    {
      protection_callback_ = cb;
    }

    void StepperEngine::set_motor_status_callback(std::function<void(bool)> cb)
    {
      motor_status_callback_ = cb;
    }

    // ============================================================================
    // Modbus Callbacks
    // ============================================================================

    void StepperEngine::on_modbus_response(const std::vector<uint8_t> &data)
    {
      // Forward to CommandQueue for command completion
      if (queue_)
      {
        queue_->on_response(data);
      }

      // TODO: Parse response for polled values (position, speed, status, protection)
      // Determine which command completed and process accordingly
    }

    void StepperEngine::on_modbus_error(uint8_t function_code, uint8_t exception_code)
    {
      ESP_LOGW(TAG_ENGINE, "Modbus error: fc=0x%02X, ec=0x%02X", function_code, exception_code);

      // Forward to CommandQueue for retry logic
      if (queue_)
      {
        queue_->on_error(function_code, exception_code);
      }

      // TODO: Handle critical errors (e.g., repeated failures → Error state)
    }

    // ============================================================================
    // Private Methods - State Machine
    // ============================================================================

    void StepperEngine::transition_to(State new_state)
    {
      if (state_ == new_state)
      {
        return; // No change
      }

      State old_state = state_;
      state_ = new_state;

      ESP_LOGD(TAG_ENGINE, "State transition: %s → %s",
               state_to_string(old_state), state_to_string(new_state));

      // State-specific initialization
      switch (new_state)
      {
      case State::Moving:
        // Start polling position more frequently (optional)
        break;

      case State::Idle:
        // Reset target position
        target_position_ = current_position_;
        break;

      case State::Error:
        // Stop all motion immediately
        if (queue_)
        {
          queue_->clear();
        }
        break;

      default:
        break;
      }
    }

    bool StepperEngine::validate_command(const char *command_name,
                                         std::initializer_list<State> allowed_states)
    {
      for (State allowed : allowed_states)
      {
        if (state_ == allowed)
        {
          return true; // Command allowed
        }
      }

      // Command not allowed in current state
      ESP_LOGW(TAG_ENGINE, "%s: Rejected (state=%s)", command_name, state_to_string(state_));
      return false;
    }

    void StepperEngine::check_state_timeouts()
    {
      // TODO: Implement state-specific timeouts
      // - Moving: Maximum movement duration (e.g., 30 seconds)
      // - Homing: Maximum homing duration (e.g., 60 seconds)
      // - Stopping: Maximum stop duration (e.g., 5 seconds)
    }

    // ============================================================================
    // Private Methods - Polling
    // ============================================================================

    void StepperEngine::execute_polling()
    {
      // Poll all status values in sequence
      poll_encoder_position();
      poll_motor_speed();
      poll_motor_status();
      poll_protection_status();
    }

    void StepperEngine::poll_encoder_position()
    {
      // TODO: Enqueue read command for encoder position (Command 0x30)
      // Expected response: carry (int32_t) + value (uint16_t)
      // queue_->enqueue_read(0x30, 3, [this](bool success, const std::vector<uint8_t>& data) {
      //   if (success && data.size() >= 6) {
      //     int32_t carry = ...;
      //     uint16_t value = ...;
      //     process_encoder_update(carry, value);
      //   }
      // });
    }

    void StepperEngine::poll_motor_speed()
    {
      // TODO: Enqueue read command for motor speed (Command 0x32)
      // Expected response: speed_rpm (int16_t)
    }

    void StepperEngine::poll_motor_status()
    {
      // TODO: Enqueue read command for motor status (Command 0x3A)
      // Expected response: enabled (uint8_t, 0 = disabled, 1 = enabled)
    }

    void StepperEngine::poll_protection_status()
    {
      // TODO: Enqueue read command for protection status (Command 0x3E)
      // Expected response: protection (uint8_t, 0 = OK, 1 = protected)
    }

    // ============================================================================
    // Private Methods - Event Processing
    // ============================================================================

    void StepperEngine::process_encoder_update(int32_t carry, uint16_t value)
    {
      encoder_carry_ = carry;
      encoder_value_ = value;

      // TODO: Calculate absolute position from carry + value
      // Position formula: position = (carry * 16384) + value (in encoder steps)

      Position old_position = current_position_;
      // current_position_ = ...;  // Update with new value

      // Check if target reached (in Moving state)
      if (state_ == State::Moving && is_target_reached())
      {
        ESP_LOGD(TAG_ENGINE, "Target position reached");
        transition_to(State::Idle);
      }

      // Invoke callback if position changed significantly (threshold: 10 steps)
      float delta = std::abs(current_position_.get_steps() - old_position.get_steps());
      if (delta >= 10.0f && position_callback_)
      {
        position_callback_(current_position_);
      }
    }

    void StepperEngine::process_speed_update(int16_t speed_rpm)
    {
      Speed old_speed = current_speed_;
      current_speed_ = Speed(static_cast<float>(speed_rpm), SpeedUnit::RPM, parent_);

      // Check if standstill reached (in Stopping state)
      if (state_ == State::Stopping && speed_rpm == 0)
      {
        ESP_LOGD(TAG_ENGINE, "Standstill reached");
        transition_to(State::Idle);
      }

      // Invoke callback if speed changed
      if (speed_rpm != static_cast<int16_t>(old_speed.rpm()) && speed_callback_)
      {
        speed_callback_(current_speed_);
      }
    }

    void StepperEngine::process_motor_status_update(bool enabled)
    {
      bool old_enabled = motor_enabled_;
      motor_enabled_ = enabled;

      // Invoke callback if status changed
      if (enabled != old_enabled && motor_status_callback_)
      {
        motor_status_callback_(enabled);
      }
    }

    void StepperEngine::process_protection_update(uint8_t protected_status)
    {
      bool old_protection = protection_triggered_;
      protection_triggered_ = (protected_status != 0);

      // Transition to Error state if protection triggered
      if (protection_triggered_ && !old_protection)
      {
        ESP_LOGE(TAG_ENGINE, "Protection triggered! Status=0x%02X", protected_status);
        handle_error("Locked-rotor protection triggered");

        if (protection_callback_)
        {
          protection_callback_();
        }
      }
    }

    bool StepperEngine::is_target_reached()
    {
      // TODO: Implement with tolerance threshold (e.g., ±5 steps)
      float tolerance = 5.0f; // steps
      float delta = std::abs(current_position_.get_steps() - target_position_.get_steps());
      return delta <= tolerance;
    }

    void StepperEngine::handle_error(const char *error_message)
    {
      ESP_LOGE(TAG_ENGINE, "Error: %s", error_message);
      transition_to(State::Error);
    }

  } // namespace servoxxd
} // namespace esphome
