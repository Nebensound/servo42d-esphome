#include "servoxxd_modbus.h"
#include "servoxxd_stepper_engine.h"

namespace esphome
{
  namespace servoxxd_modbus
  {

    static const char *const TAG = "servoxxd_modbus";

    // ============================================================================
    // Constructor / Destructor
    // ============================================================================

    ServoXxdModbus::~ServoXxdModbus()
    {
      if (this->engine_ != nullptr)
      {
        delete this->engine_;
        this->engine_ = nullptr;
      }
    }

    // ============================================================================
    // Component Lifecycle
    // ============================================================================

    void ServoXxdModbus::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up ServoXxd Modbus...");

      // Validate configuration
      if (this->steps_per_revolution_ <= 0.0f)
      {
        ESP_LOGE(TAG, "Invalid steps_per_revolution: %.1f (must be > 0)", this->steps_per_revolution_);
        // this->mark_failed();  // TODO: Uncomment when Component base is properly accessible
        return;
      }

      // Initialize StepperEngine
      this->engine_ = new StepperEngine(this);
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

    void ServoXxdModbus::loop()
    {
      // Call StepperEngine state machine update
      if (this->engine_ != nullptr)
      {
        this->engine_->update();

        // TODO: Sync position with ESPHome base class if changed
        // Position current_pos = this->engine_->get_current_position();
        // this->current_position = current_pos.steps();

        // TODO: Handle polling for motor state updates
        // - Read position periodically
        // - Read speed
        // - Read enabled state
        // - Read protection status
      }
    }

    void ServoXxdModbus::dump_config()
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
      ESP_LOGCONFIG(TAG, "  Home Direction: %s", this->home_direction_cw_ ? "Clockwise" : "Counter-Clockwise");
      ESP_LOGCONFIG(TAG, "  Home Speed: %.1f RPM", this->home_speed_.rpm());

      // Motion parameters
      ESP_LOGCONFIG(TAG, "  Default Acceleration: %.1f RPM/s", this->default_acceleration_.rpm_per_sec());

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

    void ServoXxdModbus::on_modbus_data(const std::vector<uint8_t> &data)
    {
      // TODO: Implementation required
      // - Parse response data
      // - Delegate to StepperEngine for processing
      // - Update internal state based on response

      ESP_LOGW(TAG, "on_modbus_data() not yet implemented - received %zu bytes", data.size());
    }

    void ServoXxdModbus::on_modbus_error(uint8_t function_code, uint8_t exception_code)
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

    void ServoXxdModbus::move_to(const Position &position)
    {
      if (this->engine_ == nullptr) {
        ESP_LOGE(TAG, "Cannot move_to: StepperEngine not initialized");
        return;
      }
      
      // Minimal implementation: just delegate to engine
      // TODO later: Add Position Mode validation, error state check
      this->engine_->move_to(position, nullptr, nullptr);
    }

    void ServoXxdModbus::home()
    {
      if (this->engine_ == nullptr) {
        ESP_LOGE(TAG, "Cannot home: StepperEngine not initialized");
        return;
      }
      
      // Minimal implementation: just delegate to engine
      // TODO later: Add Position Mode validation, error state check
      this->engine_->home();
    }

    void ServoXxdModbus::stop()
    {
      if (this->engine_ == nullptr) {
        ESP_LOGE(TAG, "Cannot stop: StepperEngine not initialized");
        return;
      }
      
      // Minimal implementation: just delegate to engine
      this->engine_->stop(nullptr);
    }

    void ServoXxdModbus::run_continuous(const Speed &speed)
    {
      if (this->engine_ == nullptr) {
        ESP_LOGE(TAG, "Cannot run_continuous: StepperEngine not initialized");
        return;
      }
      
      // Minimal implementation: just delegate to engine
      // TODO later: Add Speed Mode validation, error state check
      // Use default acceleration from configuration
      this->engine_->run_continuous(speed, this->default_acceleration_);
    }

    void ServoXxdModbus::emergency_stop()
    {
      if (this->engine_ == nullptr) {
        ESP_LOGE(TAG, "Cannot emergency_stop: StepperEngine not initialized");
        return;
      }
      
      // Minimal implementation: just delegate to engine
      this->engine_->emergency_stop();
    }

    void ServoXxdModbus::enable()
    {
      if (this->engine_ == nullptr) {
        ESP_LOGE(TAG, "Cannot enable: StepperEngine not initialized");
        return;
      }
      
      // Minimal implementation: just delegate to engine
      this->engine_->enable();
    }

    void ServoXxdModbus::disable()
    {
      if (this->engine_ == nullptr) {
        ESP_LOGE(TAG, "Cannot disable: StepperEngine not initialized");
        return;
      }
      
      // Minimal implementation: just delegate to engine
      this->engine_->disable();
    }

  } // namespace servoxxd_modbus
} // namespace esphome
