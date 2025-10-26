#include "servo42d.h"
#include "servo42d_modbus_commands.h"
#include "servo42d_motor_control.h"
#include "servo42d_position.h"
#include "servo42d_helpers.h"
#include "esphome/core/log.h"
#include <cmath>
#include <climits>

namespace esphome
{
  namespace servo42d_rs485
  {

    static const char *const TAG = "servo42d_rs485";

    void Servo42dRs485::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up Servo42D RS485...");

      // Initialize command queue
      this->command_queue_ = std::make_unique<CommandQueue>();

      // Initialize helper classes
      this->motor_control_ = std::make_unique<Servo42dMotorControl>(this);
      this->position_ = std::make_unique<Servo42dPosition>(this);

      // Wait a bit for motor to be ready
      this->set_interval("init_delay", 500, [this]()
                         {
    // Initial configuration sequence
    
    ESP_LOGI(TAG, "=== Starting Motor Setup Sequence (7 steps) ===");
    
    // 1. Restart motor to synchronize with ESPHome boot
    ESP_LOGI(TAG, "Step 1/7: Restarting motor controller...");
    auto restart_motor = std::make_unique<WriteCommand>(
        ModbusRegisters::Write::SYSTEM_RESET, 0x0001);
    restart_motor->set_completion_callback([this](BaseCommand*, bool success) {
      if (success) {
        ESP_LOGI(TAG, "OK Motor controller restarted - encoder reset to 0");
        this->encoder_base_value_ = 0;
        this->encoder_base_set_ = true;
        this->current_position = 0;
      } else {
        ESP_LOGW(TAG, "FAIL Motor restart failed!");
      }
    });
    this->command_queue_->enqueue(std::move(restart_motor));
    
        // 2. Set work mode (includes holding current for OPEN/CLOSE modes)
    ESP_LOGI(TAG, "Step 2/7: Setting work mode...");
    this->motor_control_->set_work_mode(this->control_mode_);
    
    // 3. Set Microstepping
    ESP_LOGI(TAG, "Step 3/7: Setting up microstepping...");
    auto set_microstepping = std::make_unique<WriteCommand>(
        ModbusRegisters::Write::SUBDIVISION, this->microsteps_);
    set_microstepping->set_completion_callback([this](BaseCommand *, bool success)
                                               {
      if (success) {
        ESP_LOGI(TAG, "OK Microstepping set to %d steps", this->microsteps_);
      } });
    this->command_queue_->enqueue(std::move(set_microstepping));

    // 4. Set working current
    ESP_LOGI(TAG, "Step 4/7: Setting working current to %d mA...", this->working_current_);
    auto set_current = std::make_unique<WriteCommand>(
        ModbusRegisters::Write::WORKING_CURRENT, this->working_current_);
    set_current->set_completion_callback([this](BaseCommand*, bool success) {
      if (success) {
        ESP_LOGI(TAG, "OK Working current set to %d mA", this->working_current_);
      }
    });
    this->command_queue_->enqueue(std::move(set_current));
    
    // Note: Holding current is now set automatically by set_work_mode() for OPEN/CLOSE modes
    
    // 5. Set EN pin active mode
    ESP_LOGI(TAG, "Step 5/7: Setting EN pin active mode...");
    auto set_en_active = std::make_unique<WriteCommand>(
        ModbusRegisters::Write::EN_ACTIVE, this->en_pin_active_);
    set_en_active->set_completion_callback([this](BaseCommand*, bool success) {
      if (success) {
        const char* mode_str = (this->en_pin_active_ == 0) ? "LOW" : (this->en_pin_active_ == 1) ? "HIGH" : "ALWAYS";
        ESP_LOGI(TAG, "OK EN pin active set to %s", mode_str);
      }
    });
    this->command_queue_->enqueue(std::move(set_en_active));
    
    // 6. Set auto screen off
    ESP_LOGI(TAG, "Step 6/7: Setting auto screen off: %s...", this->auto_screen_off_ ? "ON" : "OFF");
    auto set_screen = std::make_unique<WriteCommand>(
        ModbusRegisters::Write::AUTO_SCREEN_OFF, this->auto_screen_off_ ? 0x0001 : 0x0000);
    set_screen->set_completion_callback([this](BaseCommand*, bool success) {
      if (success) {
        ESP_LOGI(TAG, "OK Auto screen off: %s", this->auto_screen_off_ ? "enabled" : "disabled");
      }
    });
    this->command_queue_->enqueue(std::move(set_screen));
    
    // 6b. Configure 0_Mode (virtual homing) if requested
    if (this->use_virtual_home_) {
      // Map homing_direction_: 2=NEAREST -> NearMode(2), else DirMode(1)
      uint16_t zero_mode = (this->homing_direction_ == 2) ? 2 : 1;
      // 0_Speed is 0..4; we don't have a dedicated level in schema yet; use medium=2
      uint16_t zero_speed = 2;
      // 0_Dir: 0=CW, 1=CCW (ignored for NearMode)
      uint16_t zero_dir = (this->homing_direction_ == 1) ? 1 : 0;

      ModbusRegisters::Payload::ZeroModeParams zm = {
        .mode = zero_mode,
        .enable = 0,      // Do not change the stored zero automatically here
        .speed = zero_speed,
        .dir = zero_dir
      };

      ESP_LOGI(TAG, "Configuring 0_Mode: mode=%u, speed=%u, dir=%u", zero_mode, zero_speed, zero_dir);
      auto set_zero_mode = std::make_unique<MultiWriteCommand>(
        ModbusRegisters::MultiWrite::ZERO_MODE_PARAMS,
        Servo42dHelpers::to_vector(zm)
      );
      set_zero_mode->set_completion_callback([](BaseCommand*, bool ok){
        if (ok) ESP_LOGI(TAG, "0_Mode parameters configured");
      });
      this->command_queue_->enqueue(std::move(set_zero_mode));
    }
    
    // 7. Enable motor (drive enable)
    ESP_LOGI(TAG, "Step 7/7: Enabling motor...");
    auto enable_motor_cmd = std::make_unique<WriteCommand>(
        ModbusRegisters::Write::EN_CONTROL, 0x0001);
    enable_motor_cmd->set_completion_callback([](BaseCommand*, bool success) {
      if (success) {
        ESP_LOGI(TAG, "OK Motor enabled");
        ESP_LOGI(TAG, "=== Setup Complete ===");
      } else {
        ESP_LOGW(TAG, "FAIL Failed to enable motor");
      }
    });
    this->command_queue_->enqueue(std::move(enable_motor_cmd));
    
  // Query initial status
    this->query_motor_status();
    this->query_encoder_value();
    
  // After motor restart, both encoder and target are 0
  // No need to send initial move command - wait for user's first set_target
  this->target_synced_ = true;

  // Cancel this one-time setup interval
  this->cancel_interval("init_delay"); });

      // Set up periodic polling for motor status (replaces PollingComponent::update)
      // update() calls query_encoder_value(), query_motor_speed(), query_motor_status(), and query_protection_status()
      this->set_interval("status_poll", 500, [this]()
                         {
                           this->update(); // Call our update method periodically
                         });
    }

    void Servo42dRs485::dump_config()
    {
      ESP_LOGCONFIG(TAG, "Servo42D RS485 Stepper:");
      ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
      ESP_LOGCONFIG(TAG, "  Steps per revolution: %.1f", this->steps_per_revolution_);
      ESP_LOGCONFIG(TAG, "  Microsteps: %d", this->microsteps_);
      ESP_LOGCONFIG(TAG, "  Sleep when done: %s", YESNO(this->sleep_when_done_));
      ESP_LOGCONFIG(TAG, "  Max speed: %.1f steps/s", this->max_speed_);
      ESP_LOGCONFIG(TAG, "  Acceleration: %.1f steps/s²", this->acceleration_);
      ESP_LOGCONFIG(TAG, "  Deceleration: %.1f steps/s²", this->deceleration_);
    }

    void Servo42dRs485::loop()
    {
      static uint32_t loop_counter = 0;
      static uint32_t last_log = 0;
      uint32_t now = millis();

      loop_counter++;
      if (now - last_log > 10000)
      {
        ESP_LOGD(TAG, "loop() called %u times in last 10s, queue=%p, size=%d",
                 loop_counter, this->command_queue_.get(),
                 this->command_queue_ ? this->command_queue_->size() : -1);
        loop_counter = 0;
        last_log = now;
      }

      // Process command queue
      if (this->command_queue_)
      {
        // Process and execute commands from the queue (no per-loop logging to reduce output)
        this->command_queue_->process_next();
        this->command_queue_->execute_next(this);
      }
    }

    void Servo42dRs485::update()
    {
      // Track previous motor status for transition detection
      uint8_t prev_status_snapshot = this->motor_status_;

      // Continuously poll encoder, speed, and status (like in original code)
      // These are queued but won't duplicate if already in queue
      this->position_->query_encoder_value();
      this->position_->query_motor_speed();
      this->position_->query_motor_status();
      this->position_->query_protection_status();

      // Sleep when done: Disable motor if target reached and motor is idle
      if (this->sleep_when_done_)
      {
        if (!this->is_motor_moving())
        {
          // Check if we're at the target position (within a small tolerance)
          int32_t position_error = abs(this->target_position - this->current_position);
          if (position_error <= 2 && !this->motor_auto_disabled_)
          {
            if (this->post_arrival_hold_ms_ == 0)
            {
              ESP_LOGD(TAG, "Target reached, disabling motor (sleep_when_done)");
              this->disable_motor();
              this->motor_auto_disabled_ = true;
            }
            else if (!this->post_hold_scheduled_)
            {
              ESP_LOGD(TAG, "Target reached, holding for %u ms before disable", this->post_arrival_hold_ms_);
              this->post_hold_scheduled_ = true;
              this->set_timeout("post_hold_disable", this->post_arrival_hold_ms_, [this]()
                                {
                // Double-check still within tolerance and not moving
                if (!this->is_motor_moving() && abs(this->target_position - this->current_position) <= 2)
                {
                  ESP_LOGD(TAG, "Post-hold disable now");
                  this->disable_motor();
                  this->motor_auto_disabled_ = true;
                }
                this->post_hold_scheduled_ = false; });
            }
          }
        }
      }

      // Restore working current after homing finishes (virtual homing current override)
      using namespace ModbusRegisters::MotorStatus;
      uint8_t curr_status = this->motor_status_;
      if (this->homing_override_active_ && prev_status_snapshot == MOTOR_IS_HOMING && curr_status != MOTOR_IS_HOMING)
      {
        uint16_t restore_ma = this->previous_working_current_ > 0 ? this->previous_working_current_ : this->working_current_;
        ESP_LOGI(TAG, "Homing finished. Restoring working current to %u mA", restore_ma);
        auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::WORKING_CURRENT, restore_ma);
        cmd->set_completion_callback([this, restore_ma](BaseCommand *, bool ok)
                                     {
          if (ok) {
            ESP_LOGI(TAG, "Working current restored to %u mA", restore_ma);
          }
          this->homing_override_active_ = false; });
        this->command_queue_->enqueue(std::move(cmd));
      }

      // Keep a copy for next cycle
      this->prev_motor_status_ = curr_status;
    }

    void Servo42dRs485::set_target(int32_t steps)
    {
      ESP_LOGI(TAG, "set_target: target=%d (current: %d)", steps, this->current_position);

      // Re-enable motor if it was auto-disabled and we're moving to a new target
      if (this->motor_auto_disabled_)
      {
        ESP_LOGD(TAG, "Re-enabling motor for new target");
        this->enable_motor();
        this->motor_auto_disabled_ = false;
      }

      // Send move and update target
      this->send_absolute_move_(steps, /*update_target=*/true);
      this->target_synced_ = true;
    }

    void Servo42dRs485::handle_protection_change_(uint8_t prev, uint8_t curr)
    {
      if (prev == curr)
        return;

      this->protection_status_ = curr;

      if (curr != 0)
      {
        ESP_LOGW(TAG, "Protection event detected (status=0x%02X)", curr);
        // Mark target as not synced until user recovers/reissues command
        this->target_synced_ = false;
        // Optional external callback
        if (this->on_protection_)
        {
          this->on_protection_();
        }
      }
      else
      {
        ESP_LOGI(TAG, "Protection cleared");
      }
    }

    void Servo42dRs485::compute_speed_and_accel_(uint16_t &speed_rpm, uint16_t &accel_internal) const
    {
      // Compute speed and acceleration with clamps and fallbacks
      if (this->steps_per_revolution_ > 0.0f)
      {
        speed_rpm = Servo42dHelpers::steps_per_second_to_rpm(this->max_speed_, this->steps_per_revolution_);
        accel_internal = Servo42dHelpers::acceleration_to_internal(this->acceleration_, this->steps_per_revolution_);
      }
      else
      {
        // Fallback conservative defaults
        speed_rpm = 60;      // 60 RPM
        accel_internal = 10; // conservative acceleration
        ESP_LOGW(TAG, "steps_per_revolution not set; using fallback speed/accel");
      }

      // Clamp to safe values
      if (speed_rpm < 10)
        speed_rpm = 10;
      if (speed_rpm > 3000)
        speed_rpm = 3000;
      if (accel_internal < 1)
        accel_internal = 1;
      if (accel_internal > 255)
        accel_internal = 255;
    }

    void Servo42dRs485::send_absolute_move_(int32_t steps, bool update_target)
    {
      uint16_t speed_rpm = 0;
      uint16_t acceleration = 0;
      this->compute_speed_and_accel_(speed_rpm, acceleration);

      // Convert ESPHome steps to motor position (steps + offset)
      int32_t motor_target = steps + this->position_offset_;

      ESP_LOGD(TAG, "send_absolute_move_: motor_target=%d (steps=%d, offset=%d), speed=%u RPM, accel=%u",
               motor_target, steps, this->position_offset_, speed_rpm, acceleration);

      // ALWAYS send move command for reliability
      this->position_->move_to_position_mode2(acceleration, speed_rpm, motor_target);

      if (update_target)
      {
        // Update target position for ESPHome base class
        this->target_position = steps;
      }
    }

    void Servo42dRs485::report_position(int32_t position)
    {
      // Adjust offset so current encoder position maps to desired position
      // Formula: current_position = encoder_position - offset
      // Therefore: offset = encoder_position - desired_position
      this->position_offset_ = this->encoder_position_ - position;
      this->current_position = position;
      this->target_position = position;

      ESP_LOGI(TAG, "Position reset to %d (encoder=%d, offset=%d)",
               position, this->encoder_position_, this->position_offset_);
    }

    void Servo42dRs485::on_modbus_data(const std::vector<uint8_t> &data)
    {
      ESP_LOGV(TAG, "Received MODBUS data: %zu bytes", data.size());

      // Forward response to command queue
      if (this->command_queue_)
      {
        this->command_queue_->handle_response(data);
      }
    }

    void Servo42dRs485::on_modbus_error(uint8_t function_code, uint8_t exception_code)
    {
      ESP_LOGW(TAG, "MODBUS error - Function: 0x%02X, Exception: 0x%02X",
               function_code, exception_code);

      // Forward error to command queue
      if (this->command_queue_)
      {
        this->command_queue_->handle_error(function_code, exception_code);
      }
    }

    // ============================================================================
    // Motor Control Actions - Delegated to Servo42dMotorControl
    // ============================================================================

    void Servo42dRs485::enable_motor()
    {
      this->motor_control_->enable_motor();
    }

    void Servo42dRs485::disable_motor()
    {
      this->motor_control_->disable_motor();
    }

    void Servo42dRs485::emergency_stop()
    {
      this->motor_control_->emergency_stop();
    }

    void Servo42dRs485::run_continuous(float speed_steps_per_sec, uint8_t direction)
    {
      this->motor_control_->run_continuous(speed_steps_per_sec, direction);
    }

    void Servo42dRs485::stop_motor()
    {
      this->motor_control_->stop_motor();
    }

    void Servo42dRs485::home()
    {
      this->motor_control_->home();
    }

    void Servo42dRs485::reset_position()
    {
      this->motor_control_->reset_position();
    }

    void Servo42dRs485::calibrate_motor()
    {
      this->motor_control_->calibrate_motor();
    }

    void Servo42dRs485::release_protection()
    {
      this->motor_control_->release_protection();
    }

    void Servo42dRs485::restart_motor()
    {
      this->motor_control_->restart_motor();
    }

    void Servo42dRs485::set_work_mode(uint16_t mode)
    {
      this->motor_control_->set_work_mode(mode);
    }

    void Servo42dRs485::set_working_current_runtime(uint16_t current_ma)
    {
      this->motor_control_->set_working_current_runtime(current_ma);
    }

    void Servo42dRs485::set_holding_current_percent_runtime(uint8_t percent)
    {
      this->motor_control_->set_holding_current_percent_runtime(percent);
    }

    void Servo42dRs485::set_microstepping(uint16_t subdivision)
    {
      this->motor_control_->set_microstepping(subdivision);
    }

    void Servo42dRs485::key_lock()
    {
      this->motor_control_->key_lock();
    }

    void Servo42dRs485::key_unlock()
    {
      this->motor_control_->key_unlock();
    }

    // ============================================================================
    // Position Control Methods - Delegated to Servo42dPosition
    // ============================================================================

    void Servo42dRs485::move_to_position_mode1(uint16_t direction, uint16_t acceleration,
                                               uint16_t speed, uint16_t pulses)
    {
      this->position_->move_to_position_mode1(direction, acceleration, speed, pulses);
    }

    void Servo42dRs485::move_to_position_mode2(uint16_t acceleration, uint16_t speed,
                                               int32_t abs_steps)
    {
      this->position_->move_to_position_mode2(acceleration, speed, abs_steps);
    }

    void Servo42dRs485::move_to_position_mode3(uint16_t acceleration, uint16_t speed,
                                               int32_t rel_steps)
    {
      this->position_->move_to_position_mode3(acceleration, speed, rel_steps);
    }

    void Servo42dRs485::move_to_position_mode4(uint16_t acceleration, uint16_t speed,
                                               int32_t abs_steps)
    {
      this->position_->move_to_position_mode4(acceleration, speed, abs_steps);
    }

    // ============================================================================
    // Status Query Methods - Delegated to Servo42dPosition
    // ============================================================================

    void Servo42dRs485::query_motor_status()
    {
      this->position_->query_motor_status();
    }

    void Servo42dRs485::query_encoder_value()
    {
      this->position_->query_encoder_value();
    }

    void Servo42dRs485::query_motor_speed()
    {
      this->position_->query_motor_speed();
    }

    void Servo42dRs485::query_pulse_count()
    {
      this->position_->query_pulse_count();
    }

    void Servo42dRs485::query_angle_error()
    {
      this->position_->query_angle_error();
    }

    // ============================================================================
    // Status Access - Delegated to Servo42dPosition
    // ============================================================================

    bool Servo42dRs485::is_motor_moving() const
    {
      return this->position_->is_motor_moving();
    }

    // ============================================================================
    // Position Getters with Unit Conversion
    // ============================================================================

    float Servo42dRs485::get_encoder_degrees() const
    {
      // Convert encoder position (steps) to degrees using configured steps_per_revolution
      return Servo42dHelpers::steps_to_degrees(encoder_position_, steps_per_revolution_);
    }

    float Servo42dRs485::get_encoder_radians() const
    {
      // Convert encoder position (steps) to radians using configured steps_per_revolution
      return Servo42dHelpers::steps_to_radians(encoder_position_, steps_per_revolution_);
    }

    void Servo42dRs485::process_protection_status(uint8_t status)
    {
      uint8_t prev = this->protection_status_;
      this->handle_protection_change_(prev, status);
    }

  } // namespace servo42d_rs485
} // namespace esphome
