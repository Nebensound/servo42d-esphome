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
                                 uint32_t command_timeout_ms)
        : parent_(parent),
          queue_(nullptr),
          state_(State::Disabled),
          emergency_flag_(false),
          current_speed_(0.0f, SpeedUnit::RPM, parent),
          motor_enabled_(false),
          protection_triggered_(false),
          state_enter_time_(0),
          disable_pending_(false)
    {

      // Create CommandQueue (Layer 3) if transport provided
      if (transport != nullptr)
      {
        queue_ = new CommandQueue(transport, command_timeout_ms);
        ESP_LOGD(TAG_ENGINE, "StepperEngine initialized with transport: timeout=%ums",
                 command_timeout_ms);
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
    // Hardware Polling Methods
    // ============================================================================

    void StepperEngine::poll_motor_speed()
    {
      // Enqueue read command for motor speed (Command 0x32 READ_CURRENT_SPEED)
      // Expected response: speed_rpm (int16_t) = 2 bytes
      queue_->enqueue_read(Command::READ_CURRENT_SPEED, [this](bool success, const std::vector<uint8_t> &data)
                           {
        if (success && data.size() >= 2) {
          Speed speed = ServoCommandCodec::decode_current_speed(data);
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
    // Motor Setup
    // ============================================================================

    void StepperEngine::setup_motor()
    {
      if (!queue_)
      {
        ESP_LOGE(TAG_ENGINE, "Cannot setup motor: CommandQueue not initialized");
        return;
      }

      ESP_LOGCONFIG(TAG_ENGINE, "Enqueuing motor restart...");

      // 0. Restart motor to ensure clean state (Command 0x41 RESTART)
      // Motor needs ~3s to reboot before accepting configuration commands
      {
        std::vector<uint8_t> restart_payload; // No payload for restart

        restart();
      }

      // 1. Set microstepping (Command 0x84 SET_SUBDIVISION)
      {
        queue_->enqueue(Command::SET_SUBDIVISION,
                        ServoCommandCodec::encode_set_subdivision(parent_->get_microstepping()),
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
        queue_->enqueue(Command::SET_EN_PIN_ACTIVE,
                        ServoCommandCodec::encode_set_en_pin_active(parent_->get_en_pin_active()),
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
        queue_->enqueue(Command::SET_AUTO_SCREEN_OFF,
                        ServoCommandCodec::encode_set_auto_screen_off(parent_->get_auto_screen_off()),
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
        queue_->enqueue(Command::SET_LOCK_KEYS,
                        ServoCommandCodec::encode_set_lock_keys(parent_->get_lock_keys_at_startup()),
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

      // Note: SET_ZERO command is NOT called here during setup.
      // Position zeroing should be done explicitly via set_zero() or during homing.
      // Automatically resetting position during motor initialization could cause unexpected behavior.

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

      // 3. Process buffered commands
      if (disable_pending_ && state_ == State::Idle)
      {
        disable_pending_ = false;
        disable(); // Execute buffered disable
      }
    }

    // ============================================================================
    // Hardware Polling
    // ============================================================================

    void StepperEngine::poll_hardware()
    {
      // Poll all status values in sequence
      poll_encoder_position();
      poll_motor_speed();
      poll_motor_status();
      poll_protection_status();
    }

    // ============================================================================
    // Movement Commands
    // ============================================================================

    void StepperEngine::move_to(const Position &target, std::optional<Speed> speed,
                                std::optional<Acceleration> accel)
    {
      // Use default values if not provided
      Speed speed_units = speed.has_value() ? speed.value() : parent_->get_default_speed();
      Acceleration accel_units = accel.has_value() ? accel.value() : parent_->get_default_acceleration();

      // Validation: only allowed in Idle state (Position Mode)
      if (!validate_command("move_to", {State::Idle, State::Moving, State::Stopping}))
      {
        return;
      }
      ESP_LOGD(TAG_ENGINE, "move_to(): target=%lld steps, speed=%.2f RPM, accel=%.2f RPM/s",
               static_cast<long long>(target.get_steps()),
               speed_units.rpm(),
               accel_units.get_rpm_per_sec());

      // Note: target_pos_ already updated by caller (ServoXxd::move_to or set_target_pos)
      // Simply send new move command - hardware will update mid-movement
      auto payload = ServoCommandCodec::encode_move_position_mode_2(target, speed_units, accel_units);
      queue_->enqueue(Command::MOVE_POSITION_MODE_2, payload, nullptr);

      if (state_ == State::Moving || state_ == State::Stopping)
      {
        return;
      }
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
      Acceleration decel_units = decel.has_value() ? decel.value() : parent_->get_default_acceleration();
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

      // Get homing configuration from parent
      auto &homing = parent_->homing_;

      ESP_LOGD(TAG_ENGINE, "home(): Starting homing sequence (mode=%d, direction=%d)",
               static_cast<int>(homing.mode), static_cast<int>(homing.direction));

      switch (homing.mode)
      {
      case HomingMode::VIRTUAL:
      {
        // Virtual homing: Return to stored zero position via motor restart
        // Speed level (0-4), direction, and "set zero" flag
        ZeroingSpeed speed_level = static_cast<ZeroingSpeed>(homing.level);
        Direction dir = (homing.direction == HomingDirection::CW) ? Direction::CW : Direction::CCW;

        // Step 1: Configure zero mode parameters (Command 0x9A SET_ZERO_MODE)
        auto payload = ServoCommandCodec::encode_set_zero_mode(
            homing.direction, // mode (CW=DirMode, CCW=DirMode, NEAREST=NearMode)
            true,             // enable = true (set zero)
            speed_level,      // speed level 0-4 (VERY_SLOW..VERY_FAST)
            dir);             // direction (only used for DirMode)

        queue_->enqueue(Command::START_HOMING, payload, nullptr);

        ESP_LOGD(TAG_ENGINE, "  Virtual homing: level=%d, dir=%d - will restart motor",
                 homing.level, static_cast<int>(dir));

        // Step 2: Restart motor to execute virtual homing (Command 0x0F RESTART)
        // This makes the motor return to the stored zero position
        std::vector<uint8_t> restart_payload;
        queue_->enqueue(Command::RESTART, restart_payload,
                        [this](bool success, const std::vector<uint8_t> &)
                        {
                          if (success)
                          {
                            ESP_LOGD(TAG_ENGINE, "✓ Virtual homing: Motor restarting to zero position");
                          }
                          else
                          {
                            ESP_LOGW(TAG_ENGINE, "✗ Virtual homing: Failed to restart motor");
                          }
                        });
        break;
      }

      case HomingMode::ENDSTOP:
      case HomingMode::SENSORLESS:
      {
        // Real homing (ENDSTOP or SENSORLESS): Not yet fully implemented
        // These modes require additional hardware commands and configuration

        ESP_LOGE(TAG_ENGINE, "home(): ENDSTOP and SENSORLESS modes not yet implemented!");
        ESP_LOGE(TAG_ENGINE, "  Required: Hardware commands for endstop detection or stall sensing");
        ESP_LOGE(TAG_ENGINE, "  Use VIRTUAL mode for now, or implement hardware-specific homing");
        return;
      }

      default:
        ESP_LOGE(TAG_ENGINE, "home(): Invalid homing mode: %d", static_cast<int>(homing.mode));
        return;
      }

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
      Speed speed_obj = speed.has_value() ? speed.value() : parent_->get_default_speed();
      Acceleration accel_obj = accel.has_value() ? accel.value() : parent_->get_default_acceleration();

      ESP_LOGD(TAG_ENGINE, "run_continuous(): speed=%.2f RPM, accel=%.2f RPM/s",
               speed_obj.rpm(), accel_obj.get_rpm_per_sec());

      // Send speed command via queue (Command 0xF6 MOVE_SPEED_MODE)
      auto payload = ServoCommandCodec::encode_move_speed_mode(speed_obj, accel_obj);
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

      delay(3000); // Wait 3s for motor to reboot

      transition_to(State::Idle);
    }

    void StepperEngine::calibrate()
    {
      ESP_LOGD(TAG_ENGINE, "calibrate(): Starting encoder calibration");

      // Send calibrate encoder command via queue (Command 0x80 CALIBRATE_ENCODER)
      auto payload = ServoCommandCodec::encode_calibrate_encoder();
      queue_->enqueue(Command::CALIBRATE_ENCODER, payload, nullptr);
    }

    void StepperEngine::key_lock()
    {
      ESP_LOGD(TAG_ENGINE, "key_lock(): Locking physical keys");

      // Send key lock command via queue (Command 0x8F SET_LOCK_KEYS)
      auto payload = ServoCommandCodec::encode_set_lock_keys(true);
      queue_->enqueue(Command::SET_LOCK_KEYS, payload, nullptr);
    }

    void StepperEngine::key_unlock()
    {
      ESP_LOGD(TAG_ENGINE, "key_unlock(): Unlocking physical keys");

      // Send key unlock command via queue (Command 0x8F SET_LOCK_KEYS)
      auto payload = ServoCommandCodec::encode_set_lock_keys(false);
      queue_->enqueue(Command::SET_LOCK_KEYS, payload, nullptr);
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
      queue_->enqueue(Command::SET_ZERO, payload, [this](bool success, const std::vector<uint8_t> &)
                      {
        if (!success) {
          ESP_LOGW(TAG_ENGINE, "set_zero: Hardware command failed");
          return;
        }
        
        // Hardware confirmed - reset parent's offset and position tracking
        parent_->position_offset_ = Position(0.0f, PositionUnit::STEPS, parent_);
        parent_->current_position = 0;
        
        ESP_LOGI(TAG_ENGINE, "set_zero: Hardware confirmed, encoder and offset reset to zero"); });
    }

    // ============================================================================
    // Status Queries
    // ============================================================================

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
        auto position = ServoCommandCodec::decode_encoder_carry(data);
        process_encoder_update(position);
        break;
      }

      case Command::READ_CURRENT_SPEED:
      {
        auto speed = ServoCommandCodec::decode_current_speed(data);
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
        parent_->target_pos_ = parent_->current_pos_;
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

    void StepperEngine::poll_encoder_position(std::function<void(const Position &)> callback)
    {
      // Enqueue read command for encoder position (Command 0x30 READ_ENCODER_CARRY)
      // Expected response: carry (int32_t) + value (uint16_t) = 6 bytes
      queue_->enqueue_read(Command::READ_ENCODER_CARRY, [this, callback](bool success, const std::vector<uint8_t> &data)
                           {
        if (success && data.size() >= 6) {
          auto position = ServoCommandCodec::decode_encoder_carry(data, parent_);
          process_encoder_update(position);
        

        // Check if target reached (in Moving state)
        if (state_ == State::Moving && is_target_reached())
        {
          ESP_LOGD(TAG_ENGINE, "Target position reached");
          transition_to(State::Idle);
        } 
        parent_->set_current_pos(position);
        
        // Invoke user callback if provided
        if (callback) {
          callback(position);
        }
        } });
    }

    // ============================================================================
    // Private Methods - Event Processing
    // ============================================================================

    void StepperEngine::process_encoder_update(const Position &position)
    {
      Position old_position = parent_->current_pos_;
      parent_->set_current_pos(position);

      // Check if target reached (in Moving state)
      if (state_ == State::Moving && is_target_reached())
      {
        ESP_LOGD(TAG_ENGINE, "Target position reached");
        transition_to(State::Idle);
      }

      // Invoke callback if position changed significantly (threshold: 10 steps)
      float delta = std::abs(parent_->current_pos_.get_steps() - old_position.get_steps());
      if (delta >= 10.0f && position_callback_)
      {
        position_callback_(parent_->current_pos_);
      }
    }

    void StepperEngine::process_speed_update(const Speed &speed)
    {
      Speed old_speed = current_speed_;
      current_speed_ = speed;

      // Check if standstill reached (in Stopping state)
      if (state_ == State::Stopping && speed.rpm() == 0)
      {
        ESP_LOGD(TAG_ENGINE, "Standstill reached");
        transition_to(State::Idle);
      }

      // Invoke callback if speed changed
      if (speed.rpm() != old_speed.rpm() && speed_callback_)
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
      float delta = std::abs(parent_->current_pos_.get_steps() - parent_->target_pos_.get_steps());
      return delta <= tolerance;
    }

    void StepperEngine::handle_error(const char *error_message)
    {
      ESP_LOGE(TAG_ENGINE, "Error: %s", error_message);
      transition_to(State::Error);
    }

  } // namespace servoxxd
} // namespace esphome
