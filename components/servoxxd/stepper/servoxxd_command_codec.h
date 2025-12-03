#pragma once

#include <vector>
#include <cstdint>

namespace esphome
{
  namespace servoxxd
  {

    class ServoCommandCodec
    {
    public:
      // Movement encoders (minimal placeholders)
      static std::vector<uint8_t> encode_move_position_mode_2(int32_t position_steps, uint16_t speed_units, uint8_t accel_units)
      {
        std::vector<uint8_t> data;
        data.reserve(7);
        data.push_back(static_cast<uint8_t>((position_steps >> 24) & 0xFF));
        data.push_back(static_cast<uint8_t>((position_steps >> 16) & 0xFF));
        data.push_back(static_cast<uint8_t>((position_steps >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>(position_steps & 0xFF));
        data.push_back(static_cast<uint8_t>((speed_units >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>(speed_units & 0xFF));
        data.push_back(accel_units);
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
