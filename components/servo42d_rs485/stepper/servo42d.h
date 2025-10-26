#pragma once

#include "esphome/core/component.h"
#include "esphome/components/stepper/stepper.h"
#include "esphome/components/modbus/modbus.h"
#include "servo42d_command_queue.h"
#include "servo42d_modbus_commands.h"
#include "servo42d_modbus_registers.h"
#include "servo42d_register_types.h"
#include "servo42d_helpers.h"
#include <memory>

namespace esphome
{
  namespace servo42d_rs485
  {

    // Forward declarations
    class Servo42dMotorControl;
    class Servo42dPosition;
    class Servo42dHelpers;

    /**
     * @brief Servo42D RS485 Stepper Motor Component
     *
     * ESPHome component for controlling MKS Servo42D/57D closed-loop stepper motors
     * via MODBUS-RTU protocol over RS485.
     *
     * Features:
     * - Position control with multiple modes
     * - Speed and acceleration control
     * - Homing/Zeroing functionality
     * - Real-time status monitoring
     * - Protection and emergency stop
     */
    class Servo42dRs485 : public stepper::Stepper, public modbus::ModbusDevice, public Component
    {
    public:
      Servo42dRs485() = default;

      // ESPHome Component Lifecycle
      void setup() override;
      void dump_config() override;
      void loop() override;
      void update(); // Periodic polling (called via set_interval)

      // MODBUS Device callbacks
      void on_modbus_data(const std::vector<uint8_t> &data) override;
      void on_modbus_error(uint8_t function_code, uint8_t exception_code) override;

      // Configuration Methods
      void set_steps_per_revolution(float steps) { this->steps_per_revolution_ = steps; }
      void set_microsteps(uint16_t steps) { this->microsteps_ = steps; }
      void set_sleep_when_done(bool sleep) { this->sleep_when_done_ = sleep; }
      void set_control_mode(uint16_t mode) { this->control_mode_ = mode; }
      void set_working_current(uint16_t current) { this->working_current_ = current; }
      void set_holding_current_percent(uint8_t percent) { this->holding_current_percent_ = percent; }
      void set_homing_current(uint16_t current) { this->homing_current_ = current; }
      void set_en_pin_active(uint8_t mode) { this->en_pin_active_ = mode; }
      void set_home_at_startup(bool enable) { this->home_at_startup_ = enable; }
      void set_use_virtual_home(bool enable) { this->use_virtual_home_ = enable; }
      void set_virtual_home_angle(uint16_t angle) { this->virtual_home_angle_ = angle; }
      void set_homing_speed(float speed) { this->homing_speed_ = speed; } // In steps/s
      void set_homing_direction(uint8_t direction) { this->homing_direction_ = direction; }
      void set_auto_screen_off(bool enable) { this->auto_screen_off_ = enable; }
      void set_lock_keys_at_startup(bool enable) { this->lock_keys_at_startup_ = enable; }
      void set_post_arrival_hold_ms(uint32_t ms) { this->post_arrival_hold_ms_ = ms; }

      // Motor Control Actions
      void enable_motor();
      void disable_motor();
      void emergency_stop();
      void run_continuous(float speed_steps_per_sec, uint8_t direction);
      void stop_motor();
      void home();
      void reset_position();
      void calibrate_motor();
      void release_protection();
      void restart_motor();
      void set_work_mode(uint16_t mode);
      void set_working_current_runtime(uint16_t current_ma);
      void set_holding_current_percent_runtime(uint8_t percent);
      void set_microstepping(uint16_t subdivision);
      void key_lock();
      void key_unlock();

      // Position Control
      void move_to_position_mode1(uint16_t direction, uint16_t acceleration, uint16_t speed, uint16_t pulses);
      void move_to_position_mode2(uint16_t acceleration, uint16_t speed, int32_t abs_steps);
      void move_to_position_mode3(uint16_t acceleration, uint16_t speed, int32_t rel_axis);
      void move_to_position_mode4(uint16_t acceleration, uint16_t speed, int32_t abs_axis);

      // Status Queries
      void query_motor_status();
      void query_encoder_value();
      void query_motor_speed();
      void query_pulse_count();
      void query_angle_error();

      // Status Access
      uint8_t get_motor_status() const { return motor_status_; }
      int16_t get_motor_speed() const { return motor_speed_; }
      bool is_motor_moving() const;

      // Position Getters - Different Units
      int64_t get_encoder_ticks() const { return encoder_value_; }    // Raw encoder ticks (16384 per revolution)
      int32_t get_encoder_steps() const { return encoder_position_; } // Position in steps (TRUTH - hardware position)
      float get_encoder_degrees() const;                              // Position in degrees (0-359.99...)
      float get_encoder_radians() const;                              // Position in radians (0-2π)
      int32_t get_pulse_count() const { return pulse_count_; }        // Pulse count from motor
      uint8_t get_protection_status() const { return protection_status_; }

      // Conversion Helpers - Delegate to Servo42dHelpers for actual math
      // These are convenience wrappers that use the configured steps_per_revolution
      float steps_to_degrees(int32_t steps) const { return Servo42dHelpers::steps_to_degrees(steps, steps_per_revolution_); }
      float steps_to_radians(int32_t steps) const { return Servo42dHelpers::steps_to_radians(steps, steps_per_revolution_); }
      int32_t degrees_to_steps(float degrees) const { return Servo42dHelpers::degrees_to_steps(degrees, steps_per_revolution_); }
      int32_t radians_to_steps(float radians) const { return Servo42dHelpers::radians_to_steps(radians, steps_per_revolution_); }

      // Ticks conversions (static, no config needed - 16384 ticks/rev is hardware constant)
      static float ticks_to_degrees(int64_t ticks) { return Servo42dHelpers::ticks_to_degrees(ticks); }
      static float ticks_to_radians(int64_t ticks) { return Servo42dHelpers::ticks_to_radians(ticks); }
      static int64_t degrees_to_ticks(float degrees) { return Servo42dHelpers::degrees_to_ticks(degrees); }
      static int64_t radians_to_ticks(float radians) { return Servo42dHelpers::radians_to_ticks(radians); }

      // Getters for helper classes
      CommandQueue *get_command_queue() { return command_queue_.get(); }
      float get_steps_per_revolution() const { return steps_per_revolution_; }
      uint16_t get_microsteps() const { return microsteps_; }
      uint16_t get_working_current() const { return working_current_; }
      uint16_t get_homing_current() const { return homing_current_; }
      uint8_t get_holding_current_percent() const { return holding_current_percent_; }
      uint8_t get_en_pin_active() const { return en_pin_active_; }
      bool get_auto_screen_off() const { return auto_screen_off_; }
      bool get_use_virtual_home() const { return use_virtual_home_; }
      uint16_t get_virtual_home_angle() const { return virtual_home_angle_; }
      float get_homing_speed() const { return homing_speed_; }
      uint8_t get_homing_direction() const { return homing_direction_; }
      int32_t get_position_offset() const { return position_offset_; }
      int32_t get_encoder_position() const { return encoder_position_; }

      // Setters for helper classes
      void set_motor_status(uint8_t status) { motor_status_ = status; }
      void set_encoder_value(int64_t value) { encoder_value_ = value; }
      void set_encoder_position(int32_t pos) { encoder_position_ = pos; }
      void set_current_position(int32_t pos) { current_position = pos; }
      void set_motor_speed(int16_t speed) { motor_speed_ = speed; }
      void set_pulse_count(int32_t count) { pulse_count_ = count; }
      void set_angle_error(int32_t error) { angle_error_ = error; }
      void set_encoder_base_value(int64_t value) { encoder_base_value_ = value; }
      void set_protection_status(uint8_t status) { protection_status_ = status; }

      // ESPHome Stepper Interface - set_target to send commands to motor
      void set_target(int32_t steps);
      void report_position(int32_t position);

      // Homing current override control (used for virtual/noLimit homing)
      void set_homing_override_active(bool active) { homing_override_active_ = active; }
      bool get_homing_override_active() const { return homing_override_active_; }
      void set_previous_working_current(uint16_t ma) { previous_working_current_ = ma; }
      uint16_t get_previous_working_current() const { return previous_working_current_; }
      void set_prev_motor_status(uint8_t s) { prev_motor_status_ = s; }
      uint8_t get_prev_motor_status() const { return prev_motor_status_; }

      // Optional external notification on protection events
      void set_on_protection_callback(std::function<void()> cb) { this->on_protection_ = std::move(cb); }
      // Notify about protection status change (called by position helper)
      void handle_protection_change_(uint8_t prev, uint8_t curr);
      // Public wrapper to process a freshly read PROTECTION_STATUS value
      void process_protection_status(uint8_t status);

    protected:
      // Configuration
      float steps_per_revolution_{0.0f}; // 0 = not set (only steps-based units allowed)
      uint16_t microsteps_{16};
      bool sleep_when_done_{false};

      // Motor configuration (from YAML)
      uint16_t control_mode_{5};            // Default: SR_vFOC (mode 5)
      uint16_t working_current_{1500};      // Default: 1500mA (safe for testing)
      uint16_t homing_current_{0};          // Optional homing current (mA) for virtual homing
      uint8_t holding_current_percent_{50}; // Default: 50% of working current
      uint8_t en_pin_active_{2};            // Default: ALWAYS (mode 2)

      // Homing configuration (from YAML)
      bool home_at_startup_{false};
      bool use_virtual_home_{false};
      uint16_t virtual_home_angle_{0}; // 0-359 degrees
      float homing_speed_{0.0f};       // steps/s (0 = not set)
      uint8_t homing_direction_{0};    // 0=CW, 1=CCW, 2=NEAREST

      // Display configuration (from YAML)
      bool auto_screen_off_{true}; // Default: auto turn off after 15s
      bool lock_keys_at_startup_{false};
      uint32_t post_arrival_hold_ms_{0}; // Optional hold time before auto-disable

      // Command Queue System
      std::unique_ptr<CommandQueue> command_queue_;

      // Helper classes for organized functionality
      std::unique_ptr<Servo42dMotorControl> motor_control_;
      std::unique_ptr<Servo42dPosition> position_;

      // Motor State
      uint8_t motor_status_{0};
      int64_t encoder_value_{0};
      int64_t encoder_base_value_{0};
      bool encoder_base_set_{false};
      int32_t encoder_position_{0}; // Encoder position in steps (TRUTH - hardware position)
      int32_t position_offset_{0};  // Offset: current_position = encoder_position - offset
      int16_t motor_speed_{0};
      int32_t pulse_count_{0};
      int32_t angle_error_{0};
      uint8_t io_status_{0};
      uint8_t protection_status_{0};
      bool motor_auto_disabled_{false}; // Track if motor was auto-disabled by sleep_when_done
      bool target_synced_{false};       // True if motor has been commanded to current target after (re)start/protect
      bool post_hold_scheduled_{false}; // Prevent duplicate post-hold scheduling
      int32_t last_sent_target_{0};     // Last target sent to motor (for detecting external changes)

      // Homing current override state
      bool homing_override_active_{false};
      uint16_t previous_working_current_{0};
      uint8_t prev_motor_status_{0};

      // Protection callback
      std::function<void()> on_protection_{};

      // Internal helpers
      void send_modbus_command_(std::unique_ptr<BaseCommand> command);
      void update_motor_state_();
      void compute_speed_and_accel_(uint16_t &speed_rpm, uint16_t &accel_internal) const;
      void send_absolute_move_(int32_t steps, bool update_target);
    };

  } // namespace servo42d_rs485
} // namespace esphome
