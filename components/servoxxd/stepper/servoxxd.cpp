#include "servoxxd.h"
#include "servoxxd_command_factory.h"
#include "servoxxd_stepper_engine.h"
#include "servoxxd_command_decoder.h"
#include "servoxxd_commands.h"

namespace esphome {
namespace servoxxd {

static const char *const TAG = "servoxxd";

// ============================================================================
// State Query Methods
// ============================================================================

State ServoXxd::get_state() {
  if (engine_ != nullptr) {
    return engine_->get_state();
  }
  return State::SettingUp;  // Default state before engine initialized
}

std::string ServoXxd::get_state_as_string() {
  if (engine_ != nullptr) {
    return engine_->get_state_string();
  }
  return "Not Initialized";
}

// ============================================================================
// Action-API Methods
// ============================================================================

void ServoXxd::set_control_mode(ControlMode mode) {
  this->config_.mode = mode;

  // Only send to hardware if setup is complete
  if (!this->is_setup_ || this->engine_ == nullptr)
    return;

  // Critical setting - requires motor restart and full reconfiguration
  // setup_motor() will read config_.mode and send it to hardware
  ESP_LOGW(TAG, "Control mode changed - restarting motor...");
  this->engine_->setup_motor();
}

// ============================================================================
// Stepper Compatibility Methods
// ============================================================================

void ServoXxd::set_target(int32_t steps) {
  // Convert int32_t steps to Position and delegate to set_target_pos
  Position target = Position::from_steps(steps, this);
  this->set_target_pos(target);
}

void ServoXxd::set_max_speed(float speed) {
  // Convert float steps/s to Speed object and delegate to set_speed
  // ESPHome uses steps/s for stepper speed
  Speed speed_obj(speed, SpeedUnit::STEPS_PER_SEC, this);
  this->set_speed(speed_obj);
  // Also update base class member for compatibility
  this->max_speed_ = speed;
}

void ServoXxd::set_deceleration(float decel) {
  // ServoXxd uses same value for acceleration and deceleration
  // Convert to Acceleration object (ESPHome uses steps/s^2)
  Acceleration accel_obj(decel, AccelerationUnit::STEPS_PER_SEC_SQ, this);
  this->set_acceleration(accel_obj);
  // Also update base class member for compatibility
  this->deceleration_ = decel;
}

void ServoXxd::set_acceleration(float accel) {
  // Convert to Acceleration object (ESPHome uses steps/s^2)
  Acceleration accel_obj(accel, AccelerationUnit::STEPS_PER_SEC_SQ, this);
  this->set_acceleration(accel_obj);
  // Also update base class member for compatibility
  this->acceleration_ = accel;
}

// ============================================================================
// Action-API Methods
// ============================================================================

void ServoXxd::set_speed(const Speed &speed) {
  this->default_speed_ = speed;
  ESP_LOGD(TAG, "set_speed: %.2f RPM", speed.rpm());
}

void ServoXxd::set_microsteps(uint16_t microsteps) {
  // Validate microstepping value (1-256 per spec)
  if (microsteps < 1 || microsteps > 256) {
    ESP_LOGE(TAG, "Invalid microsteps: %u (must be 1-256)", microsteps);
    return;
  }

  // Always update member variable
  this->config_.subdivision = microsteps;

  // Only send to hardware if setup is complete
  if (!this->is_setup_ || this->engine_ == nullptr)
    return;

  // Critical setting - requires motor restart and full reconfiguration
  ESP_LOGW(TAG, "Microstepping changed - restarting motor...");
  this->engine_->setup_motor();
}

void ServoXxd::set_acceleration(const Acceleration &accel) {
  this->default_acceleration_ = accel;
  ESP_LOGD(TAG, "set_acceleration: %.2f RPM/s", accel.get_rpm_per_sec());
}

void ServoXxd::set_zero() {
  // Validate that virtual homing is configured
  if (this->homing_.mode != HomingMode::VIRTUAL) {
    ESP_LOGE(TAG, "set_zero: Only valid with homing.mode: VIRTUAL (current mode: %s)",
             this->homing_.mode == HomingMode::ENDSTOP ? "ENDSTOP" : "SENSORLESS");
    return;
  }

  ESP_LOGD(TAG, "set_zero: Sending command to hardware...");

  // Delegate to StepperEngine - position will be updated via callback after confirmation
  this->engine_->set_zero();
}

void ServoXxd::report_position(const Position &pos) {
  // Calculate offset: offset = desired_position - raw_encoder_position
  // So that: current_position = raw_encoder + offset = desired_position

  // Poll encoder asynchronously and calculate offset in callback
  this->engine_->poll_encoder_position([this, pos](const Position &raw_encoder) {
    this->position_offset_ = pos - raw_encoder;
    set_current_pos(pos);

    ESP_LOGD(TAG, "report_position: Raw=%.2f, Desired=%.2f, Offset=%.2f", raw_encoder.get_steps(), pos.get_steps(),
             this->position_offset_.get_steps());
  });
}

// ============================================================================
// Position Synchronization Helpers
// ============================================================================

void ServoXxd::set_current_pos(const Position &pos) {
  this->current_pos_ = pos;
  this->current_position = static_cast<int32_t>(pos.get_steps());
}

void ServoXxd::set_target_pos(const Position &pos) {
  this->target_pos_ = pos;
  this->target_position = static_cast<int32_t>(pos.get_steps());
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

ServoXxd::ServoXxd() {
  // Initialize Position object parent pointer in config_
  config_.nolimit_reverse_angle_ticks.parent_ = this;

  // Note: homing_ union will be initialized by Python setters from YAML configuration
  // Note: transport_ and engine_ are created in setup() after all setters have run
}

ServoXxd::~ServoXxd() {
  if (this->engine_ != nullptr) {
    delete this->engine_;
    this->engine_ = nullptr;
  }
  if (this->transport_ != nullptr) {
    delete this->transport_;
    this->transport_ = nullptr;
  }
}

// ============================================================================
// Pure Delegation Methods (Action-API)
// ============================================================================

void ServoXxd::release_protection() {
  if (this->engine_ != nullptr)
    this->engine_->release_protection();
}

void ServoXxd::restart() {
  if (this->engine_ != nullptr)
    this->engine_->restart();
}

void ServoXxd::calibrate() {
  if (this->engine_ != nullptr)
    this->engine_->calibrate();
}

void ServoXxd::key_lock() {
  if (this->engine_ != nullptr)
    this->engine_->key_lock();
}

void ServoXxd::key_unlock() {
  if (this->engine_ != nullptr)
    this->engine_->key_unlock();
}

void ServoXxd::home() {
  if (this->operating_mode_ != OperatingMode::POSITION) {
    ESP_LOGE(TAG, "home: Only valid in POSITION mode (current mode: SPEED)");
    return;
  }
  if (this->engine_ != nullptr)
    this->engine_->home();
}

std::string ServoXxd::get_state_string() const {
  if (this->engine_ == nullptr) {
    return "Unknown";
  }
  return this->engine_->get_state_string();
}

void ServoXxd::stop(std::optional<Acceleration> decel) {
  if (this->engine_ == nullptr)
    return;

  Acceleration actual_decel = decel.has_value() ? decel.value() : this->default_acceleration_;
  this->engine_->stop(actual_decel);
}

void ServoXxd::run_continuous(std::optional<Speed> speed, std::optional<Acceleration> accel) {
  // Speed Mode validation
  if (this->operating_mode_ != OperatingMode::SPEED) {
    ESP_LOGE(TAG, "run_continuous: Only valid in SPEED mode (current mode: POSITION)");
    return;
  }

  if (this->engine_ == nullptr)
    return;

  Speed actual_speed = speed.has_value() ? speed.value() : this->default_speed_;
  Acceleration actual_accel = accel.has_value() ? accel.value() : this->default_acceleration_;
  this->engine_->run_continuous(actual_speed, actual_accel);
}

void ServoXxd::emergency_stop() {
  if (this->engine_ != nullptr)
    this->engine_->emergency_stop();
}

void ServoXxd::enable() {
  if (this->engine_ != nullptr)
    this->engine_->enable();
}

void ServoXxd::disable() {
  if (this->engine_ != nullptr)
    this->engine_->disable();
}

// ============================================================================
// Component Lifecycle
// ============================================================================

void ServoXxd::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ServoXxd Modbus...");

  // Validate configuration
  if (this->steps_per_revolution_ <= 0.0f) {
    ESP_LOGE(TAG, "Invalid steps_per_revolution: %.1f (must be > 0)", this->steps_per_revolution_);
    this->mark_failed();
    return;
  }

  // Create Layer 4: ModbusTransport
  this->transport_ = new ModbusTransport(this);
  if (this->transport_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate ModbusTransport");
    this->mark_failed();
    return;
  }

  // Create Layer 2: StepperEngine with CommandQueue (Layer 3)
  this->engine_ = new StepperEngine(this, this->transport_);
  if (this->engine_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate StepperEngine");
    this->mark_failed();
    return;
  }

  // Start async motor initialization
  ESP_LOGCONFIG(TAG, "Starting motor initialization (async)...");
  this->setup_state_ = SetupState::IN_PROGRESS;
  this->setup_start_time_ = millis();
  this->engine_->setup_motor();

  // Note: setup() returns immediately, motor init runs in background
  // ESPHome will call can_proceed() repeatedly to check if we're ready
  // Hardware polling starts only after motor initialization completes
  // "Setup complete" message will be logged in setup_motor() completion callback

  // Note: Homing at startup is handled by motor's 0_Mode feature (Commandtype 0x9A)
  // When homing.at_startup=true and homing.mode=VIRTUAL, the motor automatically
  // returns to zero position after restart. No ESPHome-side action required.
  // For ENDSTOP/SENSORLESS modes, homing must be triggered manually via home() action.
}

void ServoXxd::loop() {
  // Lightweight loop: State machine updates only
  // Position sync: set_interval (100ms) - updates ESPHome base class
  // Hardware polling: Engine::update() - manages own timing (poll_interval_ms_)
  if (this->engine_ != nullptr) {
    // State machine update (high frequency for smooth motion control)
    // Also processes CommandQueue and hardware polling (poll_interval_ms_)
    this->engine_->update();
  }

  // Only process user commands after setup is complete
  if (this->setup_state_ != SetupState::COMPLETED) {
    // Setup still in progress - check for timeout
    if (this->setup_state_ == SetupState::IN_PROGRESS) {
      const uint32_t SETUP_TIMEOUT_MS = 10000;  // 10s timeout (motor restart takes 4s)
      uint32_t now = millis();

      if (now - this->setup_start_time_ > SETUP_TIMEOUT_MS) {
        ESP_LOGE(TAG, "Motor setup timeout after %ums", SETUP_TIMEOUT_MS);
        this->setup_state_ = SetupState::FAILED;
        this->mark_failed();
      }
    }
    return;  // Don't process user commands yet
  }

  // Setup complete - process user commands
  if (this->engine_ != nullptr) {
    // Check if target_position was changed externally (via ESPHome action)
    if (this->target_position != static_cast<int32_t>(this->target_pos_.get_steps())) {
      Position position(this->target_position, PositionUnit::STEPS, this);
      set_target_pos(position);
      move_to(position);
    }
  }
}

void ServoXxd::dump_config() {
  ESP_LOGCONFIG(TAG, "ServoXxd Modbus Stepper:");
  LOG_STEPPER(this);

  // Operating mode (determines available features)
  [[maybe_unused]] const char *op_modes[] = {"POSITION", "SPEED"};
  ESP_LOGCONFIG(TAG, "  Operating Mode: %s", op_modes[static_cast<uint8_t>(this->operating_mode_)]);

  // Control mode (hardware loop type)
  [[maybe_unused]] const char *ctrl_modes[] = {"", "", "", "SR_OPEN", "SR_CLOSE", "SR_VFOC"};
  ESP_LOGCONFIG(TAG, "  Control Mode: %s", ctrl_modes[static_cast<uint8_t>(this->config_.mode)]);

  // Motor configuration
  ESP_LOGCONFIG(TAG, "  Steps per Revolution: %.1f", this->steps_per_revolution_);
  ESP_LOGCONFIG(TAG, "  Microstepping: %u", this->config_.subdivision);

  // Current settings (only effective in SR_OPEN and SR_CLOSE modes)
  if (this->config_.mode != ControlMode::SR_VFOC) {
    ESP_LOGCONFIG(TAG, "  Working Current: %u mA", this->config_.working_current_ma);
    ESP_LOGCONFIG(TAG, "  Holding Current: %u%% of working", this->config_.holding_current_percent);
  }

  // Motor behavior
  ESP_LOGCONFIG(TAG, "  Shaft Direction: %s", this->config_.shaft_reversed ? "Reversed" : "Normal");
  [[maybe_unused]] const char *en_modes[] = {"LOW", "HIGH", "ALWAYS"};
  ESP_LOGCONFIG(TAG, "  EN Pin Active: %s", en_modes[static_cast<uint8_t>(this->config_.en_pin_active)]);
  ESP_LOGCONFIG(TAG, "  Auto Screen Off: %s", this->config_.auto_screen_off ? "enabled" : "disabled");
  ESP_LOGCONFIG(TAG, "  Lock Keys at Startup: %s", this->config_.key_lock ? "yes" : "no");

  // Homing configuration (only in POSITION mode)
  if (this->operating_mode_ == OperatingMode::POSITION) {
    if (this->homing_.mode != HomingMode::NO_HOMING) {
      [[maybe_unused]] const char *homing_modes[] = {"NO_HOMING", "ENDSTOP", "SENSORLESS", "VIRTUAL"};
      ESP_LOGCONFIG(TAG, "  Homing Mode: %s", homing_modes[static_cast<uint8_t>(this->homing_.mode)]);
      ESP_LOGCONFIG(TAG, "  Homing at Startup: %s", this->homing_.at_startup ? "YES" : "NO");

      [[maybe_unused]] const char *homing_dirs[] = {"CW", "CCW", "NEAREST"};
      ESP_LOGCONFIG(TAG, "  Homing Direction: %s", homing_dirs[static_cast<uint8_t>(this->homing_.direction)]);

      // Speed formatting depends on mode
      if (this->homing_.mode == HomingMode::VIRTUAL) {
        [[maybe_unused]] const char *speed_levels[] = {"VERY_SLOW", "SLOW", "MEDIUM", "FAST", "VERY_FAST"};
        ESP_LOGCONFIG(TAG, "  Homing Speed: %s (level %u)", speed_levels[static_cast<uint8_t>(this->homing_.level)],
                      static_cast<uint8_t>(this->homing_.level));
      } else {
        ESP_LOGCONFIG(TAG, "  Homing Speed: %.1f RPM", this->homing_.speed.rpm());
      }

      // Mode-specific settings
      if (this->homing_.mode == HomingMode::ENDSTOP) {
        [[maybe_unused]] const char *endstop_triggers[] = {"LOW", "HIGH"};
        ESP_LOGCONFIG(TAG, "  Endstop Trigger: %s",
                      endstop_triggers[static_cast<uint8_t>(this->homing_.endstop_trigger)]);
      } else if (this->homing_.mode == HomingMode::SENSORLESS) {
        ESP_LOGCONFIG(TAG, "  Homing Current: %u mA", this->homing_.current_ma);
      }
    } else {
      ESP_LOGCONFIG(TAG, "  Homing: Not configured");
    }
  }

  // Default motion parameters
  ESP_LOGCONFIG(TAG, "  Default Speed: %.1f RPM (%.0f steps/s)", this->default_speed_.rpm(),
                this->default_speed_.steps_per_sec());
  ESP_LOGCONFIG(TAG, "  Default Acceleration: %.1f RPM/s (%.0f steps/s^2)",
                this->default_acceleration_.get_rpm_per_sec(), this->default_acceleration_.get_steps_per_sec2());

  // Motion parameters (from Stepper base class)
  ESP_LOGCONFIG(TAG, "  Base Class Acceleration: %.0f steps/s^2", this->acceleration_);
  ESP_LOGCONFIG(TAG, "  Base Class Deceleration: %.0f steps/s^2", this->deceleration_);
  ESP_LOGCONFIG(TAG, "  Base Class Max Speed: %.0f steps/s", this->max_speed_);

  // Power management (only in POSITION mode)
  if (this->operating_mode_ == OperatingMode::POSITION) {
    ESP_LOGCONFIG(TAG, "  Sleep When Done: %s", this->sleep_when_done_ ? "YES" : "NO");
  }

  // Current state
  ESP_LOGCONFIG(TAG, "  Current Position: %d steps (%.2f rev)", this->current_position,
                this->current_pos_.revolutions());
  ESP_LOGCONFIG(TAG, "  Target Position: %d steps (%.2f rev)", this->target_position, this->target_pos_.revolutions());
  ESP_LOGCONFIG(TAG, "  Position Offset: %.0f steps (%.2f rev)", this->position_offset_.get_steps(),
                this->position_offset_.revolutions());
  ESP_LOGCONFIG(TAG, "  Current Speed: %.1f steps/s (%.1f RPM)", this->current_speed_,
                this->current_speed_ * 60.0f / this->steps_per_revolution_);

  // Engine state
  if (this->engine_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Engine State: %s", this->engine_->state_to_string(this->engine_->get_state()));
    ESP_LOGCONFIG(TAG, "  Is Moving: %s", this->engine_->is_moving() ? "YES" : "NO");
  } else {
    ESP_LOGCONFIG(TAG, "  StepperEngine: Not initialized");
  }
}

// ============================================================================
// Modbus Callbacks
// ============================================================================

void ServoXxd::on_modbus_data(const std::vector<uint8_t> &data) {
  // Forward response data to transport layer
  if (this->transport_ != nullptr) {
    this->transport_->handle_response(data);
  } else {
    ESP_LOGW(TAG, "on_modbus_data() called but transport is null - received %zu bytes", data.size());
  }
}

void ServoXxd::on_modbus_error(uint8_t function_code, uint8_t exception_code) {
  // Forward error to ModbusTransport to clear "busy" state immediately
  // This prevents 4-second timeout wait after motor rejects a command
  // Note: Detailed logging happens in handle_error_response() with command context
  if (this->engine_) {
    auto *modbus_transport = static_cast<ModbusTransport *>(this->engine_->get_transport());
    if (modbus_transport) {
      modbus_transport->handle_error_response(function_code, exception_code);
    }
  }
}

// ============================================================================
// Public API - Action Methods (Minimal Implementation)
// ============================================================================

void ServoXxd::move_to(const Position &position, std::optional<Speed> speed, std::optional<Acceleration> accel) {
  // Position Mode validation
  if (this->operating_mode_ != OperatingMode::POSITION) {
    ESP_LOGE(TAG, "move_to: Only valid in POSITION mode (current mode: SPEED)");
    return;
  }

  // Use default values if not provided
  Speed actual_speed = speed.has_value() ? speed.value() : this->default_speed_;
  Acceleration actual_accel = accel.has_value() ? accel.value() : this->default_acceleration_;

  this->engine_->move_to(position, actual_speed, actual_accel);
}

// ============================================================================
// Config Update Command Generator
// ============================================================================

std::vector<Command> generate_config_update_commands(const ConfigData &current, const ConfigData &desired,
                                                     const ServoXxd *parent) {
  std::vector<Command> commands;
  commands.reserve(5);  // Pre-allocate for typical number of changes (usually 0-2)

  // Order commands logically for optimal motor configuration:
  // 1. Basic motor settings (mode, currents, microstepping)
  // 2. Pin and display settings
  // 3. Homing configuration
  // 4. Special features (zero mode, port remap, etc.)

  // ========== Basic Motor Settings ==========

  if (current.mode != desired.mode) {
    commands.push_back(CommandFactory::set_control_mode(desired.mode));
  }

  if (current.working_current_ma != desired.working_current_ma) {
    commands.push_back(CommandFactory::set_working_current(desired.working_current_ma));
  }

  if (current.holding_current_percent != desired.holding_current_percent) {
    commands.push_back(CommandFactory::set_holding_current_percent(desired.holding_current_percent));
  }

  if (current.subdivision != desired.subdivision) {
    commands.push_back(CommandFactory::set_subdivision(desired.subdivision));
  }

  // ========== Pin and Display Settings ==========

  if (current.en_pin_active != desired.en_pin_active) {
    commands.push_back(CommandFactory::set_en_pin_active(desired.en_pin_active));
  }

  if (current.auto_screen_off != desired.auto_screen_off) {
    commands.push_back(CommandFactory::set_auto_screen_off(desired.auto_screen_off));
  }

  if (current.key_lock != desired.key_lock) {
    commands.push_back(CommandFactory::set_lock_keys(desired.key_lock));
  }

  // ========== Homing Configuration ==========

  // Check if any homing parameters changed
  bool homing_params_changed =
      (current.homing_trigger != desired.homing_trigger || current.homing_direction != desired.homing_direction ||
       current.homing_speed_rpm != desired.homing_speed_rpm || current.endlimit_enable != desired.endlimit_enable);

  if (homing_params_changed) {
    // Create Speed object for homing speed
    Speed homing_speed = Speed::from_rpm(desired.homing_speed_rpm, parent);
    commands.push_back(CommandFactory::set_homing_parameters(desired.homing_trigger, desired.homing_direction,
                                                             homing_speed, desired.endlimit_enable));
  }

  // Check if sensorless homing parameters changed
  bool nolimit_params_changed =
      (current.nolimit_reverse_angle_ticks.get_ticks() != desired.nolimit_reverse_angle_ticks.get_ticks() ||
       current.nolimit_mode != desired.nolimit_mode || current.nolimit_current_ma != desired.nolimit_current_ma);

  if (nolimit_params_changed) {
    commands.push_back(CommandFactory::set_nolimit_homing_params(desired.nolimit_reverse_angle_ticks,
                                                                 desired.nolimit_mode, desired.nolimit_current_ma));
  }

  // ========== Special Features ==========

  // Check if zero mode parameters changed
  bool zero_mode_changed =
      (current.zero_mode != desired.zero_mode || current.zero_task != desired.zero_task ||
       current.zero_speed != desired.zero_speed || current.zero_direction != desired.zero_direction);

  if (zero_mode_changed) {
    commands.push_back(CommandFactory::set_zero_mode(desired.zero_mode, desired.zero_task, desired.zero_speed,
                                                     desired.zero_direction));
  }

  if (current.limit_port_remap != desired.limit_port_remap) {
    commands.push_back(CommandFactory::set_limit_port_remap(desired.limit_port_remap));
  }

  return commands;
}

// ============================================================================
// ConfigData::get_update_command_types - Generate list of command types to update config
// ============================================================================
std::vector<Commandtype> ConfigData::get_update_command_types(const ConfigData &desired) const {
  std::vector<Commandtype> command_types;

  // Order of commands (optimized for dependency chain):
  // 1. Basic settings (subdivision, en_pin, screen, keys)
  // 2. Protection and trigger config (safety features)
  // 3. Control mode (requires other settings to be stable first)
  // 4. Current settings (holding current depends on control mode)
  // 5. Homing configuration
  // 6. Special features (zero mode, limit remap)

  // === Basic Settings ===
  if (this->subdivision != desired.subdivision) {
    command_types.push_back(Commandtype::SET_SUBDIVISION);
  }

  if (this->en_pin_active != desired.en_pin_active) {
    command_types.push_back(Commandtype::SET_EN_PIN_ACTIVE);
  }

  if (this->auto_screen_off != desired.auto_screen_off) {
    command_types.push_back(Commandtype::SET_AUTO_SCREEN_OFF);
  }

  if (this->key_lock != desired.key_lock) {
    command_types.push_back(Commandtype::SET_LOCK_KEYS);
  }

  // === Safety Features ===
  // Always set EN trigger config (safety feature, always configured)
  command_types.push_back(Commandtype::SET_EN_TRIGGER_CONFIG);

  // === Control Mode ===
  if (this->mode != desired.mode) {
    command_types.push_back(Commandtype::SET_WORK_MODE);
  }

  // === Current Settings ===
  // Holding current only applicable for SR_OPEN and SR_CLOSE modes
  if ((desired.mode == ControlMode::SR_OPEN || desired.mode == ControlMode::SR_CLOSE) &&
      this->holding_current_percent != desired.holding_current_percent) {
    command_types.push_back(Commandtype::SET_HOLDING_CURRENT_PERCENT);
  }

  // === Homing Configuration ===
  bool homing_changed =
      (this->homing_trigger != desired.homing_trigger || this->homing_direction != desired.homing_direction ||
       this->homing_speed_rpm != desired.homing_speed_rpm || this->endlimit_enable != desired.endlimit_enable);

  if (homing_changed) {
    command_types.push_back(Commandtype::SET_HOMING_PARAMETERS);
  }

  bool nolimit_homing_changed =
      (this->nolimit_mode != desired.nolimit_mode || this->nolimit_current_ma != desired.nolimit_current_ma ||
       this->nolimit_reverse_angle_ticks.get_ticks() != desired.nolimit_reverse_angle_ticks.get_ticks());

  if (nolimit_homing_changed) {
    command_types.push_back(Commandtype::SET_NOLIMIT_HOMING_PARAMS);
  }

  // === Special Features ===
  bool zero_mode_changed = (this->zero_mode != desired.zero_mode || this->zero_task != desired.zero_task ||
                            this->zero_speed != desired.zero_speed || this->zero_direction != desired.zero_direction);

  if (zero_mode_changed) {
    command_types.push_back(Commandtype::SET_ZERO_MODE);
  }

  if (this->limit_port_remap != desired.limit_port_remap) {
    command_types.push_back(Commandtype::SET_LIMIT_PORT_REMAP);
  }

  return command_types;
}

}  // namespace servoxxd
}  // namespace esphome
