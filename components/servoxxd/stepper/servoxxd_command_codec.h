#pragma once

#include "servoxxd.h"
#include <vector>
#include <cstdint>
#include <cmath>

namespace esphome
{
  namespace servoxxd
  {
    /**
     * @brief Type-safe Modbus command encoder/decoder for ServoXxd (Hybrid Approach)
     *
     * Uses Position, Speed, and Acceleration objects for type-safe API.
     * Internal encoding helpers eliminate code duplication (DRY principle).
     * Static methods keep memory footprint minimal (ESP32-friendly).
     */
    class ServoCommandCodec
    {
    protected:
      // ============================================================================
      // REUSABLE ENCODING HELPERS - DRY Principle
      // ============================================================================

      static void encode_uint8(std::vector<uint8_t> &data, uint8_t value)
      {
        data.push_back(value);
      }

      static void encode_uint16_be(std::vector<uint8_t> &data, uint16_t value)
      {
        data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>(value & 0xFF));
      }

      static void encode_int32_be(std::vector<uint8_t> &data, int32_t value)
      {
        data.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
        data.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
        data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>(value & 0xFF));
      }

      static void encode_uint32_be(std::vector<uint8_t> &data, uint32_t value)
      {
        data.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
        data.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
        data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
        data.push_back(static_cast<uint8_t>(value & 0xFF));
      }

    public:
      // ============================================================================
      // POSITION MODE ENCODERS
      // ============================================================================

      /**
       * @brief Position Mode 2: Absolute motion by pulses (0xFE)
       * Payload: [acc_hi][acc_lo][speed_hi][speed_lo][pos_b3][pos_b2][pos_b1][pos_b0]
       */
      static std::vector<uint8_t> encode_move_position_mode_2(
          const Position &position,
          const Speed &speed,
          const Acceleration &accel)
      {
        std::vector<uint8_t> data;
        data.reserve(8);

        encode_uint16_be(data, static_cast<uint16_t>(accel.acc_internal()));
        encode_uint16_be(data, static_cast<uint16_t>(std::abs(speed.rpm_internal())));
        encode_int32_be(data, static_cast<int32_t>(position.get_ticks()));

        return data;
      }

      /**
       * @brief Stop Position Mode 2 movement
       */
      static std::vector<uint8_t> encode_stop_position_mode_2(const Acceleration &decel)
      {
        return {decel.acc_internal()};
      }

      /**
       * @brief Position Mode 1: Relative motion by pulses (0xFD)
       * Payload: [dir][speed_hi][speed_lo][acc][pulses_b3][pulses_b2][pulses_b1][pulses_b0]
       */
      static std::vector<uint8_t> encode_move_position_mode_1(
          Direction direction,
          const Speed &speed,
          const Acceleration &accel,
          const Position &relative_position)
      {
        std::vector<uint8_t> data;
        data.reserve(8);

        encode_uint8(data, static_cast<uint8_t>(direction));
        encode_uint16_be(data, static_cast<uint16_t>(std::abs(speed.rpm_internal())));
        encode_uint8(data, accel.acc_internal());
        encode_uint32_be(data, static_cast<uint32_t>(relative_position.get_ticks()));

        return data;
      }

      /**
       * @brief Position Mode 3: Relative motion by axis (0xF4)
       * Payload: [speed_hi][speed_lo][acc][axis_b3][axis_b2][axis_b1][axis_b0]
       */
      static std::vector<uint8_t> encode_move_position_mode_3(
          const Speed &speed,
          const Acceleration &accel,
          const Position &relative_position)
      {
        std::vector<uint8_t> data;
        data.reserve(7);

        encode_uint16_be(data, static_cast<uint16_t>(std::abs(speed.rpm_internal())));
        encode_uint8(data, accel.acc_internal());
        encode_int32_be(data, static_cast<int32_t>(relative_position.get_ticks()));

        return data;
      }

      /**
       * @brief Position Mode 4: Absolute motion by axis (0xF5)
       * Same format as Mode 3
       */
      static std::vector<uint8_t> encode_move_position_mode_4(
          const Speed &speed,
          const Acceleration &accel,
          const Position &absolute_position)
      {
        return encode_move_position_mode_3(speed, accel, absolute_position);
      }

      // ============================================================================
      // SPEED MODE ENCODERS
      // ============================================================================

      /**
       * @brief Speed mode movement (0xF6)
       * Payload: [dir][speed_hi][speed_lo][acc]
       */
      static std::vector<uint8_t> encode_move_speed_mode(
          const Speed &speed,
          const Acceleration &accel)
      {
        std::vector<uint8_t> data;
        data.reserve(4);

        int16_t rpm = speed.rpm_internal();
        encode_uint8(data, (rpm < 0) ? 0x00 : 0x01);
        encode_uint16_be(data, static_cast<uint16_t>(std::abs(rpm)));
        encode_uint8(data, accel.acc_internal());

        return data;
      }

      /**
       * @brief Stop speed mode
       */
      static std::vector<uint8_t> encode_stop_speed_mode(const Acceleration &decel)
      {
        return {0x00, 0x00, 0x00, decel.acc_internal()};
      }

      // ============================================================================
      // CONFIGURATION ENCODERS
      // ============================================================================

      static std::vector<uint8_t> encode_set_working_current(uint16_t mA)
      {
        std::vector<uint8_t> data;
        encode_uint16_be(data, mA);
        return data;
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
        uint8_t hw_value = 0;
        if (percent >= 90)
          hw_value = 8;
        else if (percent >= 20)
          hw_value = (percent / 10) - 1;
        return {hw_value};
      }

      static std::vector<uint8_t> encode_set_en_pin_active(EnPinActive mode)
      {
        return {static_cast<uint8_t>(mode)};
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
        return {static_cast<uint8_t>(mode)};
      }

      static std::vector<uint8_t> encode_set_baud_rate(uint8_t baud_code)
      {
        return {baud_code};
      }

      static std::vector<uint8_t> encode_set_slave_address(uint8_t address)
      {
        return {address};
      }

      static std::vector<uint8_t> encode_set_response_mode(bool respond_enabled, bool active_enabled)
      {
        std::vector<uint8_t> data;
        encode_uint8(data, respond_enabled ? 0x01 : 0x00);
        encode_uint8(data, active_enabled ? 0x01 : 0x00);
        return data;
      }

      static std::vector<uint8_t> encode_set_modbus_rtu(bool enable)
      {
        return {static_cast<uint8_t>(enable ? 0x01 : 0x00)};
      }

      static std::vector<uint8_t> encode_set_group_address(uint8_t group_addr)
      {
        return {group_addr};
      }

      static std::vector<uint8_t> encode_set_motor_rotation_direction(Direction direction)
      {
        return {static_cast<uint8_t>(direction)};
      }

      static std::vector<uint8_t> encode_set_stall_protection(bool enable)
      {
        return {static_cast<uint8_t>(enable ? 0x01 : 0x00)};
      }

      static std::vector<uint8_t> encode_set_subdivision_interpolation(bool enable)
      {
        return {static_cast<uint8_t>(enable ? 0x01 : 0x00)};
      }

      // ============================================================================
      // HOMING ENCODERS
      // ============================================================================

      static std::vector<uint8_t> encode_set_home_parameters(
          EndstopTrigger trigger,
          HomingDirection direction,
          const Speed &speed,
          bool endlimit_enable)
      {
        std::vector<uint8_t> data;
        data.reserve(5);

        encode_uint8(data, static_cast<uint8_t>(trigger));
        encode_uint8(data, static_cast<uint8_t>(direction));
        encode_uint16_be(data, static_cast<uint16_t>(std::abs(speed.rpm_internal())));
        encode_uint8(data, endlimit_enable ? 0x01 : 0x00);

        return data;
      }

      static std::vector<uint8_t> encode_go_home()
      {
        return {};
      }

      static std::vector<uint8_t> encode_set_current_axis_zero()
      {
        return {};
      }

      static std::vector<uint8_t> encode_set_nolimit_home_parameters(
          const Position &reverse_angle,
          HomingMode home_mode,
          uint16_t home_current_ma)
      {
        std::vector<uint8_t> data;
        data.reserve(7);

        encode_int32_be(data, static_cast<int32_t>(reverse_angle.get_ticks()));
        encode_uint8(data, static_cast<uint8_t>(home_mode));
        encode_uint16_be(data, home_current_ma);

        return data;
      }

      static std::vector<uint8_t> encode_set_limit_port_remap(bool enable)
      {
        return {static_cast<uint8_t>(enable ? 0x01 : 0x00)};
      }

      // ============================================================================
      // ZERO MODE
      // ============================================================================

      /**
       * @brief Set 0_Mode parameters (Command 0x9A)
       *
       * Payload: [0_Mode][Set 0][0_Speed][0_Dir]
       *
       * @param mode 0=Disable, 1=DirMode, 2=NearMode (maps to HomingDirection enum)
       * @param enable true=set zero, false=clean zero
       * @param speed Speed level 0-4 (ZeroingSpeed enum)
       * @param direction CW/CCW for DirMode (only used when mode=DirMode)
       */
      static std::vector<uint8_t> encode_set_zero_mode(
          HomingDirection mode,
          bool enable,
          ZeroingSpeed speed,
          Direction direction)
      {
        std::vector<uint8_t> data;
        data.reserve(4);

        encode_uint8(data, static_cast<uint8_t>(mode));
        encode_uint8(data, enable ? 0x01 : 0x00);
        encode_uint8(data, static_cast<uint8_t>(speed));
        encode_uint8(data, static_cast<uint8_t>(direction));

        return data;
      }

      // ============================================================================
      // SYSTEM COMMANDS
      // ============================================================================
      // TODO:
      static std::vector<uint8_t> encode_restore_defaults() { return {}; }
      static std::vector<uint8_t> encode_restart_motor() { return {}; }
      static std::vector<uint8_t> encode_restart() { return encode_restart_motor(); } // Alias
      static std::vector<uint8_t> encode_calibrate_encoder() { return {0x00}; }
      static std::vector<uint8_t> encode_emergency_stop() { return {}; }
      static std::vector<uint8_t> encode_query_motor_status() { return {}; }
      static std::vector<uint8_t> encode_release_protection() { return {}; }
      static std::vector<uint8_t> encode_key_lock() { return {}; }
      static std::vector<uint8_t> encode_key_unlock() { return {}; }

      static std::vector<uint8_t> encode_save_speed_mode_params(bool save)
      {
        return {static_cast<uint8_t>(save ? 0xC8 : 0xCA)};
      }

      // ============================================================================
      // RESPONSE DECODERS
      // ============================================================================

      static Speed decode_current_speed(const std::vector<uint8_t> &data)
      {
        if (data.size() < 2)
          return Speed(nullptr);
        int16_t rpm = static_cast<int16_t>((static_cast<int16_t>(data[0]) << 8) | data[1]);
        return Speed::from_rpm(rpm, nullptr);
      }

      static Position decode_pulse_count(const std::vector<uint8_t> &data)
      {
        if (data.size() < 4)
          return Position(nullptr);
        int32_t ticks = (static_cast<int32_t>(data[0]) << 24) |
                        (static_cast<int32_t>(data[1]) << 16) |
                        (static_cast<int32_t>(data[2]) << 8) |
                        static_cast<int32_t>(data[3]);
        return Position::from_ticks(ticks);
      }

      static Position decode_encoder_addition(const std::vector<uint8_t> &data)
      {
        if (data.size() < 6)
          return Position(nullptr);
        int64_t ticks = 0;
        for (size_t i = 0; i < 6; i++)
        {
          ticks = (ticks << 8) | data[i];
        }
        if (ticks & 0x800000000000LL)
        {
          ticks |= 0xFFFF000000000000LL;
        }
        return Position::from_ticks(ticks);
      }

      static Position decode_angle_error(const std::vector<uint8_t> &data)
      {
        return decode_pulse_count(data);
      }

      static bool decode_enable_status(const std::vector<uint8_t> &data)
      {
        if (data.size() < 2)
          return false;
        return data[1] != 0;
      }

      struct IOPortStatus
      {
        bool in1{false};
        bool in2{false};
        bool out1{false};
        bool out2{false};
      };

      static IOPortStatus decode_io_port_status(const std::vector<uint8_t> &data)
      {
        IOPortStatus io{};
        if (data.size() >= 2)
        {
          uint8_t status = data[1];
          io.in1 = (status & 0x01) != 0;
          io.in2 = (status & 0x02) != 0;
          io.out1 = (status & 0x04) != 0;
          io.out2 = (status & 0x08) != 0;
        }
        return io;
      }

      struct ZeroingStatus
      {
        enum State
        {
          GOING_TO_ZERO = 0,
          SUCCESS = 1,
          FAILED = 2
        };
        State state{GOING_TO_ZERO};
      };

      static ZeroingStatus decode_zeroing_status(const std::vector<uint8_t> &data)
      {
        ZeroingStatus zs{};
        if (data.size() >= 2)
        {
          uint8_t status = data[1];
          if (status == 0)
            zs.state = ZeroingStatus::GOING_TO_ZERO;
          else if (status == 1)
            zs.state = ZeroingStatus::SUCCESS;
          else
            zs.state = ZeroingStatus::FAILED;
        }
        return zs;
      }

      struct DetailedMotorStatus
      {
        enum State
        {
          FAIL = 0,
          STOP = 1,
          SPEED_UP = 2,
          SPEED_DOWN = 3,
          FULL_SPEED = 4,
          HOMING = 5,
          CALIBRATING = 6
        };
        State state{FAIL};
      };

      static DetailedMotorStatus decode_detailed_motor_status(const std::vector<uint8_t> &data)
      {
        DetailedMotorStatus dms{};
        if (data.size() >= 2)
        {
          uint8_t status = data[1];
          switch (status)
          {
          case 0:
            dms.state = DetailedMotorStatus::FAIL;
            break;
          case 1:
            dms.state = DetailedMotorStatus::STOP;
            break;
          case 2:
            dms.state = DetailedMotorStatus::SPEED_UP;
            break;
          case 3:
            dms.state = DetailedMotorStatus::SPEED_DOWN;
            break;
          case 4:
            dms.state = DetailedMotorStatus::FULL_SPEED;
            break;
          case 5:
            dms.state = DetailedMotorStatus::HOMING;
            break;
          case 6:
            dms.state = DetailedMotorStatus::CALIBRATING;
            break;
          default:
            dms.state = DetailedMotorStatus::FAIL;
          }
        }
        return dms;
      }

      struct CommandResponse
      {
        enum Status
        {
          FAIL = 0,
          SUCCESS = 1,
          RUNNING = 2,
          ENDLIMIT_STOPPED = 3
        };
        Status status{FAIL};
      };

      static CommandResponse decode_command_response(const std::vector<uint8_t> &data)
      {
        CommandResponse cr{};
        if (!data.empty())
        {
          uint8_t status = data[0];
          switch (status)
          {
          case 0:
            cr.status = CommandResponse::FAIL;
            break;
          case 1:
            cr.status = CommandResponse::SUCCESS;
            break;
          case 2:
            cr.status = CommandResponse::RUNNING;
            break;
          case 3:
            cr.status = CommandResponse::ENDLIMIT_STOPPED;
            break;
          default:
            cr.status = CommandResponse::FAIL;
          }
        }
        return cr;
      }

      /**
       * @brief Decode encoder value with carry/overflow tracking (Command 0x30)
       * 
       * Hardware returns carry (int32_t) + value (uint16_t, 0-0x3FFF).
       * Absolute position = carry × 0x4000 + value
       * 
       * Example: carry=5, value=0x1234 → position = 0x14234 encoder ticks
       * 
       * Note: Use decode_encoder_addition() for direct int48_t position (Command 0x31).
       */
      static Position decode_encoder_carry(const std::vector<uint8_t> &data, const ServoXxd *parent = nullptr)
      {
        if (data.size() < 6)
          return Position(parent);
        
        int32_t carry = (static_cast<int32_t>(data[0]) << 24) |
                        (static_cast<int32_t>(data[1]) << 16) |
                        (static_cast<int32_t>(data[2]) << 8) |
                        static_cast<int32_t>(data[3]);
        uint16_t value = static_cast<uint16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]);
        
        // Combined position = carry × 0x4000 + value
        int64_t absolute_ticks = (static_cast<int64_t>(carry) * 0x4000LL) + static_cast<int64_t>(value);
        return Position::from_ticks(absolute_ticks, parent);
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
