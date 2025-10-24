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
     * Position control and encoder reading for Servo42D
     * Handles positioning modes, encoder queries, and status monitoring
     */
    class Servo42dPosition
    {
    public:
      Servo42dPosition(Servo42dRs485 *parent) : parent_(parent) {}

      // Position control modes
      void move_to_position_mode1(uint16_t direction, uint16_t acceleration,
                                  uint16_t speed, uint16_t pulses);
      void move_to_position_mode2(uint16_t acceleration, uint16_t speed,
                                  int32_t abs_steps);
      void move_to_position_mode3(uint16_t acceleration, uint16_t speed,
                                  int32_t rel_steps);
      void move_to_position_mode4(uint16_t acceleration, uint16_t speed,
                                  int32_t abs_steps);

      // Status queries
      void query_motor_status();
      void query_encoder_value();
      void query_motor_speed();
      void query_pulse_count();
      void query_angle_error();
  void query_protection_status();

      // Status access
      bool is_motor_moving() const;

    private:
      Servo42dRs485 *parent_;
    };

  } // namespace servo42d_rs485
} // namespace esphome
