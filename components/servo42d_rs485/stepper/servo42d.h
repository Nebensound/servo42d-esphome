#pragma once

#include "esphome/core/component.h"
#include "esphome/components/stepper/stepper.h"
#include "esphome/components/modbus/modbus.h"
#include "servo42d_command_queue.h"
#include "servo42d_modbus_commands.h"
#include "servo42d_modbus_registers.h"
#include "servo42d_register_types.h"
#include <memory>

namespace esphome
{
  namespace servo42d_rs485
  {

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
    class Servo42dRs485 : public stepper::Stepper, public modbus::ModbusDevice, public PollingComponent
    {
    public:
      Servo42dRs485() = default;

      // ESPHome Component Lifecycle
      void setup() override;
      void dump_config() override;
      void loop() override;
      void update() override;

      // MODBUS Device callbacks
      void on_modbus_data(const std::vector<uint8_t> &data) override;
      void on_modbus_error(uint8_t function_code, uint8_t exception_code) override;

      // Configuration Methods
      void set_steps_per_revolution(float steps) { this->steps_per_revolution_ = steps; }
      void set_microsteps(uint16_t steps) { this->microsteps_ = steps; }
      void set_sleep_when_done(bool sleep) { this->sleep_when_done_ = sleep; }

      // Motor Control Methods
      void enable_motor(bool enable);
      void emergency_stop();
      void release_protection();
      void calibrate_motor();
      void go_to_zero(bool enable, uint16_t speed = 500, uint16_t direction = 0);

      // Position Control
      void move_to_position_mode1(uint16_t direction, uint16_t acceleration, uint16_t speed, uint16_t pulses);
      void move_to_position_mode2(uint16_t acceleration, uint16_t speed, uint32_t abs_pulses);
      void move_to_position_mode3(uint16_t acceleration, uint16_t speed, int32_t rel_axis);
      void move_to_position_mode4(uint16_t acceleration, uint16_t speed, int32_t abs_axis);

      // Status Queries
      void query_motor_status();
      void query_encoder_value();
      void query_motor_speed();
      void query_pulse_count();
      void query_angle_error();

      // Configuration
      void set_work_mode(uint16_t mode);
      void set_working_current(uint16_t current_ma);
      void set_subdivision(uint16_t subdivision);
      void set_en_pin_mode(uint16_t mode);
      void set_direction(uint16_t direction);

      // Status Access
      uint8_t get_motor_status() const { return motor_status_; }
      int64_t get_encoder_value() const { return encoder_value_; }
      int16_t get_motor_speed() const { return motor_speed_; }
      int32_t get_pulse_count() const { return pulse_count_; }
      bool is_motor_moving() const;

    protected:
      // Configuration
      float steps_per_revolution_{3200.0f};
      uint16_t microsteps_{16};
      bool sleep_when_done_{false};

      // Command Queue System
      std::unique_ptr<CommandQueue> command_queue_;

      // Motor State
      uint8_t motor_status_{0};
      int64_t encoder_value_{0};
      int16_t motor_speed_{0};
      int32_t pulse_count_{0};
      int32_t angle_error_{0};
      uint8_t io_status_{0};
      uint8_t protection_status_{0};

      // Internal helpers
      void send_modbus_command_(std::unique_ptr<BaseCommand> command);
      void update_motor_state_();

      // Conversion helpers for ESPHome stepper interface
      uint16_t steps_per_second_to_rpm_(float steps_per_second) const;
      float rpm_to_steps_per_second_(uint16_t rpm) const;
      uint16_t acceleration_to_internal_(float steps_per_second_sq) const;
    };

  } // namespace servo42d_rs485
} // namespace esphome
