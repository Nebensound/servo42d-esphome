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

    void ServoXxd::set_control_mode(ControlMode mode)
    {
      this->control_mode_ = mode;

      // Only send to hardware if setup is complete
      if (!this->is_setup_ || this->engine_ == nullptr)
        return;

      // Critical setting - requires motor restart and full reconfiguration
      // setup_motor() will read control_mode_ and send it to hardware
      ESP_LOGW(TAG, "Control mode changed - restarting motor...");
      this->engine_->setup_motor();
    }

    void ServoXxd::set_speed(const Speed &speed)
    {
      this->default_speed_ = speed;
      ESP_LOGD(TAG, "set_speed: %.2f RPM", speed.rpm());
    }

    void ServoXxd::set_microsteps(uint16_t microsteps)
    {
      // Validate microstepping value (1-256 per spec)
      if (microsteps < 1 || microsteps > 256)
      {
        ESP_LOGE(TAG, "Invalid microsteps: %u (must be 1-256)", microsteps);
        return;
      }

      // Always update member variable
      this->microstepping_ = microsteps;

      // Only send to hardware if setup is complete
      if (!this->is_setup_ || this->engine_ == nullptr)
        return;

      // Critical setting - requires motor restart and full reconfiguration
      ESP_LOGW(TAG, "Microstepping changed - restarting motor...");
      this->engine_->setup_motor();
    }

    void ServoXxd::set_acceleration(const Acceleration &accel)
    {
      this->default_acceleration_ = accel;
      ESP_LOGD(TAG, "set_acceleration: %.2f RPM/s", accel.get_rpm_per_sec());
    }

    void ServoXxd::set_zero()
    {
      // Validate that virtual homing is configured
      if (this->homing_.mode != HomingMode::VIRTUAL)
      {
        ESP_LOGE(TAG, "set_zero: Only valid with homing.mode: VIRTUAL (current mode: %s)",
                 this->homing_.mode == HomingMode::ENDSTOP ? "ENDSTOP" : "SENSORLESS");
        return;
      }

      ESP_LOGD(TAG, "set_zero: Sending command to hardware...");

      // Delegate to StepperEngine - position will be updated via callback after confirmation
      this->engine_->set_zero();
    }

    void ServoXxd::report_position(const Position &pos)
    {
      // Calculate offset: offset = desired_position - raw_encoder_position
      // So that: current_position = raw_encoder + offset = desired_position
      Position raw_encoder = this->engine_->get_raw_encoder_position();
      this->position_offset_ = pos - raw_encoder;
      set_current_pos(pos);

      ESP_LOGD(TAG, "report_position: Raw=%.2f, Desired=%.2f, Offset=%.2f",
               raw_encoder.get_steps(), pos.get_steps(), this->position_offset_.get_steps());
    }

    // ============================================================================
    // Position Synchronization Helpers
    // ============================================================================

    void ServoXxd::set_current_pos(const Position &pos)
    {
      this->current_pos_ = pos;
      this->current_position = static_cast<int32_t>(pos.get_steps());
    }

    void ServoXxd::set_target_pos(const Position &pos)
    {
      this->target_pos_ = pos;
      this->target_position = static_cast<int32_t>(pos.get_steps());
    }

    // ============================================================================
    // Constructor / Destructor
    // ============================================================================

    ServoXxd::ServoXxd()
    {
      // Initialize homing_.speed union member with valid parent pointer
      // Default mode in HomingConfig is ENDSTOP, so we need to initialize Speed member
      // (Python setters will override this if homing is configured in YAML)
      new (&homing_.speed) Speed(100.0f, SpeedUnit::RPM, this);

      // Note: transport_ and engine_ are created in setup() after all setters have run
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

      // Create Layer 4: ModbusTransport
      this->transport_ = new ModbusTransport(this, this->address_);
      if (this->transport_ == nullptr)
      {
        ESP_LOGE(TAG, "Failed to allocate ModbusTransport");
        return;
      }

      // Create Layer 2: StepperEngine with CommandQueue (Layer 3)
      this->engine_ = new StepperEngine(this, this->transport_);
      if (this->engine_ == nullptr)
      {
        ESP_LOGE(TAG, "Failed to allocate StepperEngine");
        return;
      }

      // Send initial configuration via CommandQueue
      ESP_LOGCONFIG(TAG, "Enqueuing initial configuration commands...");
      this->engine_->setup_motor();

      ESP_LOGCONFIG(TAG, "  Steps per Revolution: %.1f", this->steps_per_revolution_);
      ESP_LOGCONFIG(TAG, "  Microstepping: %u", this->microstepping_);
      ESP_LOGCONFIG(TAG, "  Working Current: %u mA", this->working_current_);
      ESP_LOGCONFIG(TAG, "  Holding Current: %u%%", this->holding_current_percent_);

      // TODO: Query initial motor state from hardware
      // - Read enabled state
      // - Read protection status
      // - Sync with StepperEngine

      // Mark setup as complete - setters can now update hardware
      this->is_setup_ = true;

      ESP_LOGCONFIG(TAG, "ServoXxd Modbus setup complete");
    }

    void ServoXxd::loop()
    {
      // Call StepperEngine state machine update
      if (this->engine_ != nullptr)
      {
        this->engine_->update();

        // Sync current position from hardware (raw encoder + offset)
        Position current_pos = this->engine_->get_raw_encoder_position() + this->position_offset_;
        if (current_pos != this->current_pos_)
        {
          set_current_pos(current_pos);
        }

        // Check if target_position was changed externally (via ESPHome action)
        if (this->target_position != static_cast<int32_t>(this->target_pos_.get_steps()))
        {
          set_target_pos(Position(this->target_position, PositionUnit::STEPS, this));
          // TODO: Sync to StepperEngine if needed
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
      const char *en_modes[] = {"LOW", "HIGH", "ALWAYS"};
      ESP_LOGCONFIG(TAG, "  EN Pin Active: %s", en_modes[static_cast<uint8_t>(this->en_pin_active_)]);
      ESP_LOGCONFIG(TAG, "  Auto Screen Off: %s", this->auto_screen_off_ ? "enabled" : "disabled");
      ESP_LOGCONFIG(TAG, "  Lock Keys at Startup: %s", this->lock_keys_at_startup_ ? "yes" : "no");

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
      // Forward response data to transport layer
      if (this->transport_ != nullptr)
      {
        // Check if transport is waiting for read response
        if (this->transport_->is_busy() && !this->transport_->is_waiting_write())
        {
          this->transport_->handle_read_response(data);
        }
        else if (this->transport_->is_waiting_write())
        {
          // Write command completed (motor acknowledged the write)
          this->transport_->handle_write_response();
        }
        else
        {
          ESP_LOGW(TAG, "Received unsolicited Modbus data: %zu bytes", data.size());
        }
      }
      else
      {
        ESP_LOGW(TAG, "on_modbus_data() called but transport is null - received %zu bytes", data.size());
      }
    }

    void ServoXxd::on_modbus_error(uint8_t function_code, uint8_t exception_code)
    {
      ESP_LOGE(TAG, "Modbus error - Function: 0x%02X, Exception: 0x%02X", function_code, exception_code);

      // TODO: Forward error to transport/engine for proper error handling
      // For now, just log the error
    }

    // ============================================================================
    // Public API - Action Methods (Minimal Implementation)
    // ============================================================================

    void ServoXxd::move_to(const Position &position, std::optional<Speed> speed,
                           std::optional<Acceleration> accel)
    {
      // Use default values if not provided
      Speed actual_speed = speed.has_value() ? speed.value() : this->default_speed_;
      Acceleration actual_accel = accel.has_value() ? accel.value() : this->default_acceleration_;

      // Minimal implementation: just delegate to engine
      // TODO later: Add Position Mode validation, error state check
      this->engine_->move_to(position, actual_speed, actual_accel);
    }



  } // namespace servoxxd
} // namespace esphome
