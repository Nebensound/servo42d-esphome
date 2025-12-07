#pragma once

#include <vector>
#include <cstdint>
#include "servoxxd.h"

namespace esphome
{
  namespace servoxxd
  {

    class ServoCommandCodec
    {
    public:
      // Movement encoders
      // Modbus Register Layout for 0xFE (Position Mode 2):
      // Bytes: [acc_hi] [acc_lo] [speed_hi] [speed_lo] [pos_b3] [pos_b2] [pos_b1] [pos_b0]
      static std::vector<uint8_t> encode_move_position_mode_2(int32_t position_steps, uint16_t speed_units, uint16_t accel_units)
      {
        std::vector<uint8_t> data;
        data.reserve(8);
        // Acceleration (uint16_t) - FIRST
        data.push_back(static_cast<uint8_t>((accel_units >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>(accel_units & 0xFF));
        // Speed (uint16_t) - SECOND
        data.push_back(static_cast<uint8_t>((speed_units >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>(speed_units & 0xFF));
        // Position (int32_t, big-endian) - THIRD
        data.push_back(static_cast<uint8_t>((position_steps >> 24) & 0xFF));
        data.push_back(static_cast<uint8_t>((position_steps >> 16) & 0xFF));
        data.push_back(static_cast<uint8_t>((position_steps >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>(position_steps & 0xFF));
        return data;
      }

      static std::vector<uint8_t> encode_stop_position_mode_2(uint8_t decel_units)
      {
        return {decel_units};
      }

      static std::vector<uint8_t> encode_move_speed_mode(uint16_t speed_units, uint8_t accel_units, uint8_t direction)
      {
        return {direction, static_cast<uint8_t>((speed_units >> 8) & 0xFF), static_cast<uint8_t>(speed_units & 0xFF), accel_units};
      }

      // Configuration encoders (minimal placeholders)
      static std::vector<uint8_t> encode_set_working_current(uint16_t mA)
      {
        return {static_cast<uint8_t>((mA >> 8) & 0xFF), static_cast<uint8_t>(mA & 0xFF)};
      }

      static std::vector<uint8_t> encode_set_subdivision(uint8_t microsteps)
      {
        return {microsteps};
      }

      static std::vector<uint8_t> encode_enable_motor(bool enable)
      {
        return {static_cast<uint8_t>(enable ? 0x01 : 0x00)};
      }

      static std::vector<uint8_t> encode_set_holding_current_percent(uint8_t percent)
      {
        // Hardware expects: 0=10%, 1=20%, ..., 8=90%
        // Convert 0-100% to 0-8 range: value = (percent / 10) - 1
        // Clamp to valid range
        uint8_t hw_value = 0;
        if (percent >= 90)
          hw_value = 8;
        else if (percent >= 20)
          hw_value = (percent / 10) - 1;
        // else hw_value = 0 (10%)
        return {hw_value};
      }

      static std::vector<uint8_t> encode_set_en_pin_active(uint8_t mode)
      {
        // 0=LOW, 1=HIGH, 2=ALWAYS (Hold mode)
        return {mode};
      }

      static std::vector<uint8_t> encode_set_auto_screen_off(bool enable)
      {
        return {static_cast<uint8_t>(enable ? 0x01 : 0x00)};
      }

      static std::vector<uint8_t> encode_set_lock_keys(bool lock)
      {
        return {static_cast<uint8_t>(lock ? 0x01 : 0x00)};
      }

      static std::vector<uint8_t> encode_set_control_mode(ControlMode mode)
      {
        // ControlMode enum values: SR_OPEN=3, SR_CLOSE=4, SR_VFOC=5
        return {static_cast<uint8_t>(mode)};
      }

      // Response decoders (minimal validation)
      static int16_t decode_current_speed(const std::vector<uint8_t> &data)
      {
        if (data.size() < 2)
          return 0;
        return static_cast<int16_t>((static_cast<int16_t>(data[0]) << 8) | data[1]);
      }

      static int32_t decode_pulse_count(const std::vector<uint8_t> &data)
      {
        if (data.size() < 4)
          return 0;
        return (static_cast<int32_t>(data[0]) << 24) | (static_cast<int32_t>(data[1]) << 16) |
               (static_cast<int32_t>(data[2]) << 8) | static_cast<int32_t>(data[3]);
      }

      struct EncoderValue
      {
        int32_t carry;
        uint16_t value;
      };

      static EncoderValue decode_encoder_carry(const std::vector<uint8_t> &data)
      {
        EncoderValue ev{0, 0};
        if (data.size() >= 6)
        {
          ev.carry = (static_cast<int32_t>(data[0]) << 24) | (static_cast<int32_t>(data[1]) << 16) |
                     (static_cast<int32_t>(data[2]) << 8) | static_cast<int32_t>(data[3]);
          ev.value = static_cast<uint16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]);
        }
        return ev;
      }

      struct MotorStatus
      {
        enum State
        {
          STOP,
          MOVING,
          HOMING
        };
        State state{STOP};
      };

      static MotorStatus decode_motor_status(const std::vector<uint8_t> &data)
      {
        MotorStatus st{};
        if (!data.empty())
        {
          uint8_t v = data[0];
          st.state = (v == 0 ? MotorStatus::STOP : (v == 1 ? MotorStatus::MOVING : MotorStatus::HOMING));
        }
        return st;
      }

      struct ProtectionStatus
      {
        bool protected_state{false};
      };

      static ProtectionStatus decode_protection_status(const std::vector<uint8_t> &data)
      {
        ProtectionStatus ps{};
        ps.protected_state = (!data.empty() && data[0] != 0);
        return ps;
      }
    };

  } // namespace servoxxd
} // namespace esphome
