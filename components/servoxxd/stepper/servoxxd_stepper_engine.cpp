#include "servoxxd_stepper_engine.h"
#include "servoxxd.h"
#include "servoxxd_command_codec.h"
#include "servoxxd_commands.h"
#include "servoxxd_transport.h"
#include <cmath>

namespace esphome
{
  namespace servoxxd
  {

    static const char *TAG_ENGINE = "servoxxd.stepper_engine";

    // ============================================================================
    // Constructor / Destructor
    // ============================================================================

    StepperEngine::StepperEngine(ServoXxd *parent, ITransport *transport,
                                 uint32_t command_timeout_ms, uint32_t poll_interval_ms)
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
          state_enter_time_(0),
          disable_pending_(false)
    {

      // Create CommandQueue (Layer 3) if transport provided
      if (transport != nullptr)
      {
        queue_ = new CommandQueue(transport, command_timeout_ms);
        ESP_LOGD(TAG_ENGINE, "StepperEngine initialized with transport: timeout=%ums, poll=%ums",
                 command_timeout_ms, poll_interval_ms);
      }
      else
      {
        ESP_LOGD(TAG_ENGINE, "StepperEngine initialized WITHOUT transport (stub mode)");
      }
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
    // Motor Setup
    // ============================================================================

    void StepperEngine::setup_motor()
    {
      if (!queue_)
      {
        ESP_LOGE(TAG_ENGINE, "Cannot setup motor: CommandQueue not initialized");
        return;
      }

      ESP_LOGCONFIG(TAG_ENGINE, "Enqueuing motor setup commands...");

      // 1. Set microstepping (Command 0x84 SET_SUBDIVISION)
      {
        auto subdivision_data = ServoCommandCodec::encode_set_subdivision(parent_->get_microstepping());
        auto subdivision_payload = std::vector<uint8_t>(subdivision_data.begin(), subdivision_data.end());

        queue_->enqueue(Command::SET_SUBDIVISION, subdivision_payload,
                        [this](bool success, const std::vector<uint8_t> &)
                        {
                          if (success)
                          {
                            ESP_LOGD(TAG_ENGINE, "✓ Microstepping set to %u", parent_->get_microstepping());
                          }
                          else
                          {
                            ESP_LOGW(TAG_ENGINE, "✗ Failed to set microstepping");
                          }
                        });
      }

      // 2. Set EN pin active level (Command 0x85 SET_EN_PIN_ACTIVE)
      {
        auto en_pin_data = ServoCommandCodec::encode_set_en_pin_active(static_cast<uint8_t>(parent_->get_en_pin_active()));
        auto en_pin_payload = std::vector<uint8_t>(en_pin_data.begin(), en_pin_data.end());

        queue_->enqueue(Command::SET_EN_PIN_ACTIVE, en_pin_payload,
                        [this](bool success, const std::vector<uint8_t> &)
                        {
                          if (success)
                          {
                            const char *mode_names[] = {"LOW", "HIGH", "ALWAYS"};
                            ESP_LOGD(TAG_ENGINE, "✓ EN pin active: %s", mode_names[static_cast<uint8_t>(parent_->get_en_pin_active())]);
                          }
                          else
                          {
                            ESP_LOGW(TAG_ENGINE, "✗ Failed to set EN pin active level");
                          }
                        });
      }

      // 3. Set auto screen off (Command 0x87 SET_AUTO_SCREEN_OFF)
      {
        auto screen_data = ServoCommandCodec::encode_set_auto_screen_off(parent_->get_auto_screen_off());
        auto screen_payload = std::vector<uint8_t>(screen_data.begin(), screen_data.end());

        queue_->enqueue(Command::SET_AUTO_SCREEN_OFF, screen_payload,
                        [this](bool success, const std::vector<uint8_t> &)
                        {
                          if (success)
                          {
                            ESP_LOGD(TAG_ENGINE, "✓ Auto screen off: %s", parent_->get_auto_screen_off() ? "enabled" : "disabled");
                          }
                          else
                          {
                            ESP_LOGW(TAG_ENGINE, "✗ Failed to set auto screen off");
                          }
                        });
      }

      // 4. Set key lock (Command 0x8F SET_LOCK_KEYS)
      {
        auto lock_data = ServoCommandCodec::encode_set_lock_keys(parent_->get_lock_keys_at_startup());
        auto lock_payload = std::vector<uint8_t>(lock_data.begin(), lock_data.end());

        queue_->enqueue(Command::SET_LOCK_KEYS, lock_payload,
                        [this](bool success, const std::vector<uint8_t> &)
                        {
                          if (success)
                          {
                            ESP_LOGD(TAG_ENGINE, "✓ Keys: %s", parent_->get_lock_keys_at_startup() ? "locked" : "unlocked");
                          }
                          else
                          {
                            ESP_LOGW(TAG_ENGINE, "✗ Failed to set key lock");
                          }
                        });
      }

      // 5. Set control mode (Command 0x82 SET_WORK_MODE)
      {
        ControlMode control_mode = parent_->get_control_mode();

        // Compute mode name for logging
        const char *mode_name;
        switch (control_mode)
        {
        case ControlMode::SR_OPEN:
          mode_name = "SR_OPEN";
          break;
        case ControlMode::SR_CLOSE:
          mode_name = "SR_CLOSE";
          break;
        case ControlMode::SR_VFOC:
          mode_name = "SR_vFOC";
          break;
        }

        queue_->enqueue(Command::SET_WORK_MODE,
                        ServoCommandCodec::encode_set_control_mode(control_mode),
                        [this, mode_name](bool success, const std::vector<uint8_t> &)
                        {
                          if (success)
                          {
                            ESP_LOGD(TAG_ENGINE, "✓ Control mode set to %s", mode_name);
                          }
                          else
                          {
                            ESP_LOGW(TAG_ENGINE, "✗ Failed to set control mode");
                          }
                        });
      }

      // 6. Set zero position (Command 0x92 SET_ZERO)
      // Note: Control mode is set via set_control_mode() called during component initialization
      {
        auto zero_payload = std::vector<uint8_t>();

        queue_->enqueue(Command::SET_ZERO, zero_payload,
                        [this](bool success, const std::vector<uint8_t> &)
                        {
                          if (success)
                          {
                            ESP_LOGD(TAG_ENGINE, "✓ Position reset to 0");
                            // Update internal tracking
                            current_position_ = Position(0.0f, PositionUnit::STEPS, parent_);
                          }
                          else
                          {
                            ESP_LOGW(TAG_ENGINE, "✗ Failed to reset position");
                          }
                        });
      }

      ESP_LOGCONFIG(TAG_ENGINE, "Setup: 6 commands enqueued (will execute via CommandQueue)");
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

    void StepperEngine::move_to(const Position &target, std::optional<Speed> speed,
                                std::optional<Acceleration> accel)
    {
      // Target override behavior: allowed in Moving/Stopping states
      if (state_ == State::Moving || state_ == State::Stopping)
      {
        ESP_LOGD(TAG_ENGINE, "move_to(): Override current movement with new target");
        target_position_ = target;

        // Send new target to hardware (Command 0xFE MOVE_POSITION_MODE_2)
        int32_t position_steps = static_cast<int32_t>(target.get_steps());
        uint16_t speed_units = speed.has_value() ? static_cast<uint16_t>(speed->rpm() * 16.0f) : 0;
        uint16_t accel_units = accel.has_value() ? static_cast<uint16_t>(accel->get_rpm_per_sec() / 10.0f) : 0;
        auto payload = ServoCommandCodec::encode_move_position_mode_2(position_steps, speed_units, accel_units);
        queue_->enqueue(Command::MOVE_POSITION_MODE_2, payload, nullptr);
        return;
      }

      // Validation: only allowed in Idle state (Position Mode)
      if (!validate_command("move_to", {State::Idle}))
      {
        return;
      }

      target_position_ = target;

      ESP_LOGD(TAG_ENGINE, "move_to(): target=%lld steps, speed=%.2f RPM, accel=%.2f RPM/s",
               static_cast<long long>(target.get_steps()),
               speed.has_value() ? speed->rpm() : 0.0f,
               accel.has_value() ? accel->get_rpm_per_sec() : 0.0f);

      // Send move command via queue (Command 0xFE MOVE_POSITION_MODE_2)
      int32_t position_steps = static_cast<int32_t>(target.get_steps());
      uint16_t speed_units = speed.has_value() ? static_cast<uint16_t>(speed->rpm() * 16.0f) : 0;
      uint16_t accel_units = accel.has_value() ? static_cast<uint16_t>(accel->get_rpm_per_sec() / 10.0f) : 0;
      auto payload = ServoCommandCodec::encode_move_position_mode_2(position_steps, speed_units, accel_units);
      queue_->enqueue(Command::MOVE_POSITION_MODE_2, payload, nullptr);

      transition_to(State::Moving);
    }

    void StepperEngine::stop(std::optional<Acceleration> decel)
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

      ESP_LOGD(TAG_ENGINE, "stop(): decel=%.2f RPM/s", decel.has_value() ? decel->get_rpm_per_sec() : 0.0f);

      // Send stop command via queue (Command 0xFE STOP_POSITION_MODE_2)
      uint8_t decel_units = decel.has_value() ? static_cast<uint8_t>(decel->get_rpm_per_sec() / 10.0f) : 0;
      auto payload = ServoCommandCodec::encode_stop_position_mode_2(decel_units);
      queue_->enqueue(Command::STOP_POSITION_MODE_2, payload, nullptr);

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

      // Send emergency stop command to hardware (Command 0xF7 EMERGENCY_STOP)
      std::vector<uint8_t> payload;                                     // No payload for emergency stop
      queue_->enqueue(Command::EMERGENCY_STOP, payload, nullptr, true); // Priority command

      // Disable motor immediately
      auto disable_payload = ServoCommandCodec::encode_enable_motor(false);
      queue_->enqueue(Command::ENABLE_MOTOR, disable_payload, nullptr, true);

      transition_to(State::Error);
    }

    void StepperEngine::home()
    {
      // Validation: only allowed in Idle state (Position Mode)
      if (!validate_command("home", {State::Idle}))
      {
        return;
      }

      ESP_LOGD(TAG_ENGINE, "home(): Starting homing sequence");

      // Send homing command via queue (Command 0x9A START_HOMING)
      std::vector<uint8_t> payload; // No payload for basic homing
      queue_->enqueue(Command::START_HOMING, payload, nullptr);

      transition_to(State::Homing);
    }

    void StepperEngine::run_continuous(std::optional<Speed> speed,
                                       std::optional<Acceleration> accel)
    {
      // Validation: allowed in Idle or Running states (Speed Mode)
      if (!validate_command("run_continuous", {State::Idle, State::Running}))
      {
        return;
      }

      // Use default values if not provided (requires parent defaults)
      float speed_rpm = speed.has_value() ? speed->rpm() : parent_->get_default_speed().rpm();
      float accel_rpm_s = accel.has_value() ? accel->get_rpm_per_sec() : parent_->get_default_acceleration().get_rpm_per_sec();

      ESP_LOGD(TAG_ENGINE, "run_continuous(): speed=%.2f RPM, accel=%.2f RPM/s",
               speed_rpm, accel_rpm_s);

      // Send speed command via queue (Command 0xF6 MOVE_SPEED_MODE)
      uint16_t speed_units = static_cast<uint16_t>(std::abs(speed_rpm) * 16.0f);
      uint8_t accel_units = static_cast<uint8_t>(accel_rpm_s / 10.0f);
      uint8_t direction = (speed_rpm >= 0.0f) ? 0x00 : 0x01; // 0=CW, 1=CCW
      auto payload = ServoCommandCodec::encode_move_speed_mode(speed_units, accel_units, direction);
      queue_->enqueue(Command::MOVE_SPEED_MODE, payload, nullptr);

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

      // Send enable command via queue (Command 0xF3 ENABLE_MOTOR)
      auto payload = ServoCommandCodec::encode_enable_motor(true);
      queue_->enqueue(Command::ENABLE_MOTOR, payload, nullptr);

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

      // Send disable command via queue (Command 0xF3 ENABLE_MOTOR with false)
      auto payload = ServoCommandCodec::encode_enable_motor(false);
      queue_->enqueue(Command::ENABLE_MOTOR, payload, nullptr);

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

      // Send release protection command via queue (Command 0x0E RELEASE_PROTECTION)
      std::vector<uint8_t> payload; // No payload
      queue_->enqueue(Command::RELEASE_PROTECTION, payload, nullptr);

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

      // Send restart command to hardware (Command 0x0F RESTART)
      std::vector<uint8_t> payload; // No payload
      queue_->enqueue(Command::RESTART, payload, nullptr);

      transition_to(State::Disabled);
    }

    void StepperEngine::set_zero()
    {
      if (!validate_command("set_zero", {State::Idle}))
      {
        return;
      }

      ESP_LOGD(TAG_ENGINE, "set_zero(): Sending SET_ZERO command to hardware");

      // Send set zero command via queue (Command 0x92 SET_ZERO)
      std::vector<uint8_t> payload; // No payload
      
      // Update position tracking only after hardware confirms
      queue_->enqueue(Command::SET_ZERO, payload, [this]() {
        // Hardware confirmed - reset encoder position
        current_position_ = Position(0.0f, PositionUnit::STEPS, parent_);
        encoder_carry_ = 0;
        encoder_value_ = 0;
        
        // Also reset parent's offset and position tracking
        parent_->position_offset_ = Position(0.0f, PositionUnit::STEPS, parent_);
        parent_->current_position = 0;
        
        ESP_LOGI(TAG_ENGINE, "set_zero: Hardware confirmed, encoder and offset reset to zero");
      });
    }

    // ============================================================================
    // Status Queries
    // ============================================================================

    Position StepperEngine::get_raw_encoder_position() const
    {
      // Return raw encoder position without any offset
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
    // Transport Callbacks (Layer 4 Integration)
    // ============================================================================

    void StepperEngine::on_transport_response(Command cmd, const std::vector<uint8_t> &data)
    {
      ESP_LOGD(TAG_ENGINE, "on_transport_response: cmd=0x%02X, %zu bytes", static_cast<uint8_t>(cmd), data.size());

      // Decode response based on command type
      switch (cmd)
      {
      case Command::READ_ENCODER_CARRY:
      {
        auto ev = ServoCommandCodec::decode_encoder_carry(data);
        process_encoder_update(ev.carry, ev.value);
        break;
      }

      case Command::READ_CURRENT_SPEED:
      {
        int16_t speed = ServoCommandCodec::decode_current_speed(data);
        process_speed_update(speed);
        break;
      }

      case Command::READ_MOTOR_STATUS:
      {
        auto status = ServoCommandCodec::decode_motor_status(data);
        bool enabled = (status.state != ServoCommandCodec::MotorStatus::STOP);
        process_motor_status_update(enabled);
        break;
      }

      case Command::READ_PROTECTION_STATUS:
      {
        auto ps = ServoCommandCodec::decode_protection_status(data);
        process_protection_update(ps.protected_state ? 1 : 0);
        break;
      }

      default:
        ESP_LOGD(TAG_ENGINE, "on_transport_response: Unhandled command 0x%02X", static_cast<uint8_t>(cmd));
        break;
      }
    }

    void StepperEngine::on_transport_error(Command cmd, ErrorCode error)
    {
      ESP_LOGW(TAG_ENGINE, "on_transport_error: cmd=0x%02X, error=%d", static_cast<uint8_t>(cmd), static_cast<int>(error));

      // Error handling based on severity
      if (error == ErrorCode::TIMEOUT)
      {
        ESP_LOGW(TAG_ENGINE, "Command timeout for 0x%02X", static_cast<uint8_t>(cmd));
        // Timeouts are already handled by CommandQueue retry logic
        // Only transition to Error state if critical movement command times out
        if (cmd == Command::MOVE_POSITION_MODE_2 || cmd == Command::EMERGENCY_STOP)
        {
          handle_error("Critical command timeout");
        }
      }
      else if (error == ErrorCode::DEVICE_ERROR)
      {
        ESP_LOGE(TAG_ENGINE, "Device error for command 0x%02X, transitioning to Error state", static_cast<uint8_t>(cmd));
        transition_to(State::Error);
      }
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
      state_enter_time_ = millis(); // Track state entry time for timeout monitoring

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
      // State-specific timeout monitoring
      uint32_t now = millis();
      uint32_t state_duration = now - state_enter_time_;

      switch (state_)
      {
      case State::Moving:
        // Maximum movement duration: 30 seconds
        if (state_duration > 30000)
        {
          ESP_LOGE(TAG_ENGINE, "Movement timeout after %u ms", state_duration);
          handle_error("Movement timeout");
        }
        break;

      case State::Homing:
        // Maximum homing duration: 60 seconds
        if (state_duration > 60000)
        {
          ESP_LOGE(TAG_ENGINE, "Homing timeout after %u ms", state_duration);
          handle_error("Homing timeout");
        }
        break;

      case State::Stopping:
        // Maximum stop duration: 5 seconds
        if (state_duration > 5000)
        {
          ESP_LOGE(TAG_ENGINE, "Stopping timeout after %u ms", state_duration);
          // Force transition to Idle even if not at standstill
          transition_to(State::Idle);
        }
        break;

      default:
        // No timeout for other states
        break;
      }
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
      // Enqueue read command for encoder position (Command 0x30 READ_ENCODER_CARRY)
      // Expected response: carry (int32_t) + value (uint16_t) = 6 bytes
      queue_->enqueue_read(Command::READ_ENCODER_CARRY, [this](bool success, const std::vector<uint8_t> &data)
                           {
        if (success && data.size() >= 6) {
          auto ev = ServoCommandCodec::decode_encoder_carry(data);
          process_encoder_update(ev.carry, ev.value);
        } });
    }

    void StepperEngine::poll_motor_speed()
    {
      // Enqueue read command for motor speed (Command 0x32 READ_CURRENT_SPEED)
      // Expected response: speed_rpm (int16_t) = 2 bytes
      queue_->enqueue_read(Command::READ_CURRENT_SPEED, [this](bool success, const std::vector<uint8_t> &data)
                           {
        if (success && data.size() >= 2) {
          int16_t speed = ServoCommandCodec::decode_current_speed(data);
          process_speed_update(speed);
        } });
    }

    void StepperEngine::poll_motor_status()
    {
      // Enqueue read command for motor status (Command 0x3A READ_MOTOR_STATUS)
      // Expected response: status (uint8_t, 0=STOP, 1=MOVING, 2=HOMING) = 2 bytes (1 register)
      queue_->enqueue_read(Command::READ_MOTOR_STATUS, [this](bool success, const std::vector<uint8_t> &data)
                           {
        if (success && !data.empty()) {
          auto status = ServoCommandCodec::decode_motor_status(data);
          bool enabled = (status.state != ServoCommandCodec::MotorStatus::STOP);
          process_motor_status_update(enabled);
        } });
    }

    void StepperEngine::poll_protection_status()
    {
      // Enqueue read command for protection status (Command 0x3E READ_PROTECTION_STATUS)
      // Expected response: protection (uint8_t, 0 = OK, 1 = protected) = 2 bytes (1 register)
      queue_->enqueue_read(Command::READ_PROTECTION_STATUS, [this](bool success, const std::vector<uint8_t> &data)
                           {
        if (success && !data.empty()) {
          auto ps = ServoCommandCodec::decode_protection_status(data);
          process_protection_update(ps.protected_state ? 1 : 0);
        } });
    }

    // ============================================================================
    // Private Methods - Event Processing
    // ============================================================================

    void StepperEngine::process_encoder_update(int32_t carry, uint16_t value)
    {
      encoder_carry_ = carry;
      encoder_value_ = value;

      // Calculate absolute position from carry + value
      // Position formula: position = (carry * 16384) + value (in encoder steps)
      int32_t total_encoder_steps = (carry * 16384) + static_cast<int32_t>(value);

      Position old_position = current_position_;
      current_position_ = Position(static_cast<float>(total_encoder_steps), PositionUnit::STEPS, parent_);

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
      // Check if current position is within tolerance of target
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
