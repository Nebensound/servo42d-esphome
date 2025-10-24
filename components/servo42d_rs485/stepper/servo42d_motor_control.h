#pragma once

#include "esphome/core/component.h"
#include "servo42d_command_queue.h"
#include "servo42d_modbus_registers.h"
#include <memory>

namespace esphome
{
  namespace servo42d_rs485
  {

    // Forward declaration
    class Servo42dRs485;

    /**
     * Motor control functions for Servo42D
     * Handles enable/disable, continuous rotation, homing, calibration, etc.
     */
    class Servo42dMotorControl
    {
    public:
      Servo42dMotorControl(Servo42dRs485 *parent) : parent_(parent) {}

      // Motor enable/disable
      void enable_motor();
      void disable_motor();
      void emergency_stop();

      // Continuous rotation
      void run_continuous(float speed_steps_per_sec, uint8_t direction);
      void stop_motor();

      // Homing and calibration
      void home();
      void reset_position();
      void calibrate_motor();

      // Motor configuration
      void release_protection();
      void restart_motor();
      void set_work_mode(uint16_t mode);
      void set_working_current_runtime(uint16_t current_ma);
      void set_holding_current_percent_runtime(uint8_t percent);
      void set_microstepping(uint16_t subdivision);

      // Key lock/unlock
      void key_lock();
      void key_unlock();

    private:
      Servo42dRs485 *parent_;
    };

  } // namespace servo42d_rs485
} // namespace esphome
