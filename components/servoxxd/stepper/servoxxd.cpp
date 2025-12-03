#include "servoxxd.h"
#include "servoxxd_stepper_engine.h"
#include "servoxxd_command_codec.h"
#include "servoxxd_commands.h"

namespace esphome
{
  namespace servoxxd
  {

    static const char *const TAG = "servoxxd";

    // ==== Action-API Methoden ====
    void ServoXxd::set_work_mode(OperatingMode mode)
    {
      if (this->engine_ == nullptr)
        return;

      // Map OperatingMode to WorkMode enum from Command layer
      uint8_t work_mode_value;
      switch (mode)
      {
      case OperatingMode::POSITION:
        work_mode_value = 3; // SR_CLOSE_LOOP
        break;
      case OperatingMode::SPEED:
        work_mode_value = 5; // SR_VFOC
        break;
      default:
        ESP_LOGW(TAG, "Unknown operating mode: %d", static_cast<int>(mode));
        return;
      }

      auto data = ServoCommandCodec::encode_set_subdivision(work_mode_value);
      // TODO: Send via StepperEngine when Layer 2 is complete
      // engine_->send_command(Command::SET_WORK_MODE, data);
      ESP_LOGD(TAG, "set_work_mode: mode=%d", static_cast<int>(work_mode_value));
    }

    // void ServoXxd::set_microsteps(uint16_t microsteps) { /* bereits implementiert */ }
    // void ServoXxd::set_working_current(uint16_t current_ma) { /* bereits implementiert */ }
    // void ServoXxd::set_holding_current_percent(uint8_t percent) { /* bereits implementiert */ }

    void ServoXxd::set_speed(const Speed &speed)
    {
      this->default_speed_ = speed;
      ESP_LOGD(TAG, "set_speed: %.2f RPM", speed.rpm());
    }

    void ServoXxd::set_acceleration(const Acceleration &accel)
    {
      this->default_acceleration_ = accel;
      ESP_LOGD(TAG, "set_acceleration: %.2f RPM/s", accel.get_rpm_per_sec());
    }

    void ServoXxd::set_zero()
    {
      if (this->engine_ == nullptr)
        return;

      // Delegate to StepperEngine
      this->engine_->set_zero();

      // Update internal position tracking
      this->current_position = 0;

      ESP_LOGD(TAG, "set_zero: current position set to zero");
    }

    void ServoXxd::report_position(const Position &pos)
    {
      // Set the internal position without moving the motor
      this->current_position = pos.get_steps();

      ESP_LOGD(TAG, "report_position: position set to %lld steps", pos.get_steps());
    }

    void ServoXxd::release_protection()
    {
      if (this->engine_ == nullptr)
        return;

      // Delegate to StepperEngine
      this->engine_->release_protection();

      ESP_LOGD(TAG, "release_protection: clearing error state");
    }

    void ServoXxd::restart()
    {
      if (this->engine_ == nullptr)
        return;

      // Delegate to StepperEngine
      this->engine_->restart();

      ESP_LOGW(TAG, "restart: restarting motor controller");
    }

    void ServoXxd::calibrate()
    {
      if (this->engine_ == nullptr)
        return;

      // Send CALIBRATE_ENCODER command
      // TODO: Send via StepperEngine when Layer 2 is complete
      // engine_->send_command(Command::CALIBRATE_ENCODER, {});

      ESP_LOGD(TAG, "calibrate: starting encoder calibration");
    }

    void ServoXxd::key_lock()
    {
      if (this->engine_ == nullptr)
        return;

      // Send KEY_LOCK command (value 0x01 = lock)
      auto data = ServoCommandCodec::encode_enable_motor(true); // Reuse encoder, value=1 means lock
      // TODO: Send via StepperEngine when Layer 2 is complete
      // engine_->send_command(Command::KEY_LOCK, data);

      ESP_LOGD(TAG, "key_lock: locking physical buttons");
    }

    void ServoXxd::key_unlock()
    {
      if (this->engine_ == nullptr)
        return;

      // Send KEY_LOCK command (value 0x00 = unlock)
      auto data = ServoCommandCodec::encode_enable_motor(false); // Reuse encoder, value=0 means unlock
      // TODO: Send via StepperEngine when Layer 2 is complete
      // engine_->send_command(Command::KEY_LOCK, data);

      ESP_LOGD(TAG, "key_unlock: unlocking physical buttons");
    }

    // ============================================================================
    // Constructor / Destructor
    // ============================================================================

    ServoXxd::ServoXxd()
    {
      // Initialize Speed in homing_ union with valid parent pointer
      // Default mode is ENDSTOP, so we initialize the Speed member
      new (&homing_.speed) Speed(100.0f, SpeedUnit::RPM, this);
    }

    ServoXxd::~ServoXxd()
    {
      if (this->engine_ != nullptr)
      {
        delete this->engine_;
        this->engine_ = nullptr;
      }
      if (this->transport_ != nullptr)
      {
        delete this->transport_;
        this->transport_ = nullptr;
      }
    }

    // ============================================================================
    // Component Lifecycle
    // ============================================================================

    void ServoXxd::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up ServoXxd Modbus...");

      // Validate configuration
      if (this->steps_per_revolution_ <= 0.0f)
      {
        ESP_LOGE(TAG, "Invalid steps_per_revolution: %.1f (must be > 0)", this->steps_per_revolution_);
        // this->mark_failed();  // TODO: Uncomment when Component base is properly accessible
        return;
      }

      // Layer 4: Create ModbusTransport
      // Note: Get slave address from ModbusDevice base class
      this->transport_ = new ModbusTransport(this, this->address_);
      if (this->transport_ == nullptr)
      {
        ESP_LOGE(TAG, "Failed to allocate ModbusTransport");
        return;
      }

      // Initialize StepperEngine (Layer 2) with transport
      this->engine_ = new StepperEngine(this, this->transport_);
      if (this->engine_ == nullptr)
      {
        ESP_LOGE(TAG, "Failed to allocate StepperEngine");
        // this->mark_failed();  // TODO: Uncomment when Component base is properly accessible
        return;
      }

      ESP_LOGCONFIG(TAG, "  Steps per Revolution: %.1f", this->steps_per_revolution_);
      ESP_LOGCONFIG(TAG, "  Microstepping: %u", this->microstepping_);
      ESP_LOGCONFIG(TAG, "  Working Current: %u mA", this->working_current_);
      ESP_LOGCONFIG(TAG, "  Holding Current: %u%%", this->holding_current_percent_);

      // TODO: Query initial motor state from hardware
      // - Read current position
      // - Read enabled state
      // - Read protection status
      // - Sync with StepperEngine

      // TODO: Send initial configuration to hardware
      // - Set microstepping
      // - Set currents
      // - Set work mode
      // - Set direction

      ESP_LOGCONFIG(TAG, "ServoXxd Modbus setup complete");
    }

    void ServoXxd::loop()
    {
      // Call StepperEngine state machine update
      if (this->engine_ != nullptr)
      {
        this->engine_->update();

        // Sync position with ESPHome base class if changed
        Position current_pos = this->engine_->get_current_position();
        int32_t new_position = static_cast<int32_t>(current_pos.get_steps());

        // Only update if position changed to avoid unnecessary writes
        if (this->current_position != new_position)
        {
          this->current_position = new_position;
        }

        // Check if target_position was changed externally (via ESPHome action)
        // and sync to StepperEngine if needed
        // Note: Polling is handled inside StepperEngine::update()
      }
    }

    void ServoXxd::dump_config()
    {
      ESP_LOGCONFIG(TAG, "ServoXxd Modbus Stepper:");
      // LOG_STEPPER(this);  // TODO: Use proper ESPHome macro when available

      // Motor configuration
      ESP_LOGCONFIG(TAG, "  Steps per Revolution: %.1f", this->steps_per_revolution_);
      ESP_LOGCONFIG(TAG, "  Microstepping: %u", this->microstepping_);

      // Current settings
      ESP_LOGCONFIG(TAG, "  Working Current: %u mA", this->working_current_);
      ESP_LOGCONFIG(TAG, "  Holding Current: %u%% of working", this->holding_current_percent_);

      // Motor behavior
      ESP_LOGCONFIG(TAG, "  Shaft Direction: %s", this->shaft_reversed_ ? "Reversed" : "Normal");
      ESP_LOGCONFIG(TAG, "  EN Pin Active: %s", this->en_pin_active_high_ ? "HIGH" : "LOW");

      // Homing configuration
      ESP_LOGCONFIG(TAG, "  Home Direction: %s", "Unknown");
      ESP_LOGCONFIG(TAG, "  Home Speed: %s", "Unknown");

      // Motion parameters
      ESP_LOGCONFIG(TAG, "  Default Acceleration: %.1f RPM/s", this->default_acceleration_.get_rpm_per_sec());

      // Power management
      ESP_LOGCONFIG(TAG, "  Sleep When Done: %s", this->sleep_when_done_ ? "YES" : "NO");

      // Engine state
      if (this->engine_ != nullptr)
      {
        ESP_LOGCONFIG(TAG, "  Current State: %s", this->engine_->state_to_string(this->engine_->get_state()));
        ESP_LOGCONFIG(TAG, "  Is Moving: %s", this->engine_->is_moving() ? "YES" : "NO");
      }
      else
      {
        ESP_LOGCONFIG(TAG, "  StepperEngine: Not initialized");
      }

      // TODO: Add more detailed state info
      // - Current position in various units
      // - Current speed
      // - Target position (if moving)
      // - Protection status
      // - Work mode
    }

    // ============================================================================
    // Modbus Callbacks
    // ============================================================================

    void ServoXxd::on_modbus_data(const std::vector<uint8_t> &data)
    {
      // TODO: Implementation required
      // - Parse response data
      // - Delegate to StepperEngine for processing
      // - Update internal state based on response

      ESP_LOGW(TAG, "on_modbus_data() not yet implemented - received %zu bytes", data.size());
    }

    void ServoXxd::on_modbus_error(uint8_t function_code, uint8_t exception_code)
    {
      // TODO: Implementation required
      // - Log error details
      // - Delegate to StepperEngine for error handling
      // - Potentially transition to Error state

      ESP_LOGE(TAG, "Modbus error - Function: 0x%02X, Exception: 0x%02X", function_code, exception_code);
    }

    // ============================================================================
    // Public API - Action Methods (Minimal Implementation)
    // ============================================================================

    void ServoXxd::move_to(const Position &position, std::optional<Speed> speed,
                           std::optional<Acceleration> accel)
    {
      if (this->engine_ == nullptr)
      {
        ESP_LOGE(TAG, "Cannot move_to: StepperEngine not initialized");
        return;
      }

      // Minimal implementation: just delegate to engine
      // TODO later: Add Position Mode validation, error state check
      this->engine_->move_to(position, speed, accel);
    }

    void ServoXxd::home()
    {
      if (this->engine_ == nullptr)
      {
        ESP_LOGE(TAG, "Cannot home: StepperEngine not initialized");
        return;
      }

      // Minimal implementation: just delegate to engine
      // TODO later: Add Position Mode validation, error state check
      this->engine_->home();
    }

    void ServoXxd::stop(std::optional<Acceleration> decel)
    {
      if (this->engine_ == nullptr)
      {
        ESP_LOGE(TAG, "Cannot stop: StepperEngine not initialized");
        return;
      }

      // Minimal implementation: just delegate to engine
      this->engine_->stop(decel);
    }

    void ServoXxd::run_continuous(std::optional<Speed> speed,
                                  std::optional<Acceleration> accel)
    {
      if (this->engine_ == nullptr)
      {
        ESP_LOGE(TAG, "Cannot run_continuous: StepperEngine not initialized");
        return;
      }

      // Minimal implementation: just delegate to engine
      // TODO later: Add Speed Mode validation, error state check
      this->engine_->run_continuous(speed, accel);
    }

    void ServoXxd::emergency_stop()
    {
      if (this->engine_ == nullptr)
      {
        ESP_LOGE(TAG, "Cannot emergency_stop: StepperEngine not initialized");
        return;
      }

      // Minimal implementation: just delegate to engine
      this->engine_->emergency_stop();
    }

    void ServoXxd::enable()
    {
      if (this->engine_ == nullptr)
      {
        ESP_LOGE(TAG, "Cannot enable: StepperEngine not initialized");
        return;
      }

      // Minimal implementation: just delegate to engine
      this->engine_->enable();
    }

    void ServoXxd::disable()
    {
      if (this->engine_ == nullptr)
      {
        ESP_LOGE(TAG, "Cannot disable: StepperEngine not initialized");
        return;
      }

      // Minimal implementation: just delegate to engine
      this->engine_->disable();
    }

  } // namespace servoxxd
} // namespace esphome
