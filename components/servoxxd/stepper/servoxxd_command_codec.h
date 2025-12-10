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
       * @brief Position Mode 2: Absolute motion by pulses
       *
       * @details Command::MOVE_POSITION_MODE_2 (0xFE)
       * Function: 0x06 (Write Single Register)
       * Payload: 8 bytes - [acc_hi][acc_lo][speed_hi][speed_lo][pos_b3][pos_b2][pos_b1][pos_b0]
       *
       * @param position Target absolute position
       * @param speed Movement speed
       * @param accel Acceleration
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
       * @brief Stop Position Mode 2 with deceleration
       *
       * @details Command::STOP_POSITION_MODE_2 (0xFF)
       * Function: 0x06 (Write Single Register)
       * Payload: 1 byte - [deceleration]
       *
       * @param decel Deceleration
       */
      static std::vector<uint8_t> encode_stop_position_mode_2(const Acceleration &decel)
      {
        return {decel.acc_internal()};
      }

      /**
       * @brief Position Mode 1: Relative motion by pulses
       *
       * @details Command::MOVE_POSITION_MODE_1 (0xFD)
       * Function: 0x06 (Write Single Register)
       * Payload: 8 bytes - [dir][speed_hi][speed_lo][acc][pulses_b3][pulses_b2][pulses_b1][pulses_b0]
       *
       * @param direction Movement direction (CW/CCW)
       * @param speed Movement speed
       * @param accel Acceleration
       * @param relative_position Relative position offset
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
       * @brief Position Mode 3: Relative motion by axis
       *
       * @details Command::MOVE_POSITION_MODE_3 (0xF4)
       * Function: 0x06 (Write Single Register)
       * Payload: 7 bytes - [speed_hi][speed_lo][acc][axis_b3][axis_b2][axis_b1][axis_b0]
       *
       * @param speed Movement speed
       * @param accel Acceleration
       * @param relative_position Relative position offset
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
       * @brief Position Mode 4: Absolute motion by axis
       *
       * @details Command::MOVE_POSITION_MODE_4 (0xF5)
       * Function: 0x06 (Write Single Register)
       * Same payload format as Mode 3
       *
       * @param speed Movement speed
       * @param accel Acceleration
       * @param absolute_position Absolute target position
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
       * @brief Speed mode movement - constant velocity rotation
       *
       * @details Command::MOVE_SPEED_MODE (0xF6)
       * Function: 0x06 (Write Single Register)
       * Payload: 4 bytes - [dir][speed_hi][speed_lo][acc]
       *
       * @param speed Target speed (sign determines direction)
       * @param accel Acceleration
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
       * @brief Stop speed mode with deceleration
       *
       * @details Command::MOVE_SPEED_MODE (0xF6) with speed=0
       * Function: 0x06 (Write Single Register)
       * Payload: 4 bytes - [0x00][0x00][0x00][decel]
       *
       * @param decel Deceleration
       */
      static std::vector<uint8_t> encode_stop_speed_mode(const Acceleration &decel)
      {
        return {0x00, 0x00, 0x00, decel.acc_internal()};
      }

      // ============================================================================
      // CONFIGURATION ENCODERS
      // ============================================================================

      /**
       * @brief Set working current
       *
       * @details Command::SET_WORKING_CURRENT_RUNTIME (0x83)
       * Function: 0x06 (Write Single Register)
       * Payload: 2 bytes - [current_hi][current_lo]
       *
       * @param mA Working current in milliamps (0-5200 depending on motor)
       */
      static std::vector<uint8_t> encode_set_working_current(uint16_t mA)
      {
        std::vector<uint8_t> data;
        encode_uint16_be(data, mA);
        return data;
      }

      /**
       * @brief Set microstepping subdivision
       *
       * @details Command::SET_SUBDIVISION (0x84)
       * Function: 0x06 (Write Single Register)
       * Payload: 1 byte - [microsteps] (1, 2, 4, 8, 16, 32, 64, etc.)
       */
      static std::vector<uint8_t> encode_set_subdivision(uint8_t microsteps)
      {
        return {microsteps};
      }

      /**
       * @brief Enable or disable motor
       *
       * @details Command::ENABLE_MOTOR (0xF3)
       * Function: 0x06 (Write Single Register)
       * Payload: 1 byte - [0x01=enable, 0x00=disable]
       */
      static std::vector<uint8_t> encode_enable_motor(bool enable)
      {
        return {static_cast<uint8_t>(enable ? 0x01 : 0x00)};
      }

      /**
       * @brief Set holding current percentage
       *
       * @details Command::SET_HOLDING_CURRENT_PERCENT (0x9B)
       * Function: 0x06 (Write Single Register)
       * Payload: 1 byte - [0-8 for 10%-90%]
       *
       * @param percent Holding current percentage (10-90)
       */
      static std::vector<uint8_t> encode_set_holding_current_percent(uint8_t percent)
      {
        uint8_t hw_value = 0;
        if (percent >= 90)
          hw_value = 8;
        else if (percent >= 20)
          hw_value = (percent / 10) - 1;
        return {hw_value};
      }

      /**
       * @brief Set EN pin active level
       *
       * @details Command::SET_EN_PIN_ACTIVE (0x85)
       * Function: 0x06 (Write Single Register)
       * Payload: 1 byte - [0=LOW, 1=HIGH, 2=ALWAYS]
       */
      static std::vector<uint8_t> encode_set_en_pin_active(EnPinActive mode)
      {
        return {static_cast<uint8_t>(mode)};
      }

      /**
       * @brief Set auto screen off
       *
       * @details Command::SET_AUTO_SCREEN_OFF (0x87)
       * Function: 0x06 (Write Single Register)
       * Payload: 1 byte - [0x00=Disable, 0x01=Enable]
       */
      static std::vector<uint8_t> encode_set_auto_screen_off(bool enable)
      {
        return {static_cast<uint8_t>(enable ? 0x01 : 0x00)};
      }

      /**
       * @brief Set key lock
       *
       * @details Command::SET_LOCK_KEYS (0x8F)
       * Function: 0x06 (Write Single Register)
       * Payload: 1 byte - [0x00=unlock, 0x01=lock]
       */
      static std::vector<uint8_t> encode_set_lock_keys(bool lock)
      {
        return {static_cast<uint8_t>(lock ? 0x01 : 0x00)};
      }

      /**
       * @brief Set control mode
       *
       * @details Command::SET_WORK_MODE (0x82)
       * Function: 0x06 (Write Single Register)
       * Payload: 1 byte - [mode] (SR_OPEN, SR_CLOSE, SR_VFOC)
       */
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
        data.reserve(6); // 3 registers = 6 bytes

        encode_uint8(data, static_cast<uint8_t>(trigger));
        encode_uint8(data, static_cast<uint8_t>(direction));
        encode_uint16_be(data, static_cast<uint16_t>(std::abs(speed.rpm_internal())));
        encode_uint8(data, endlimit_enable ? 0x01 : 0x00);
        encode_uint8(data, 0x00); // Padding to 6 bytes (3 registers)

        return data;
      }

      /**
       * @brief Start homing sequence - Go home
       *
       * @details Command::START_HOMING (0x91)
       * Function: 0x06 (Write Single Register)
       * Payload: 0 bytes (empty)
       *
       * Note: Homing parameters must be set first via encode_set_home_parameters()
       */
      static std::vector<uint8_t> encode_go_home()
      {
        return {};
      }

      static std::vector<uint8_t> encode_set_current_axis_zero()
      {
        return {};
      }

      /**
       * @brief Set the parameter of home
       *
       * @details Command::SET_HOMING_PARAMETERS (0x90)
       * Function: 0x10 (Write Multiple Registers)
       * Payload: 5 bytes - [hmTrig][hmDir][HmSpeed_hi][HmSpeed_lo][EndLimit]
       * Hardware Manual: "Set the parameter of home"
       *
       * @param endstop_trigger hmTrig: HIGH or LOW trigger level
       * @param direction hmDir: CW or CCW
       * @param speed HmSpeed: Homing speed (0-3000 RPM)
       * @param endlimit_enable EndLimit: 0=disable, 1=enable endstop-limit
       */
      static std::vector<uint8_t> encode_set_home_parameters(
          EndstopTrigger endstop_trigger,
          Direction direction,
          const Speed &speed,
          bool endlimit_enable)
      {
        std::vector<uint8_t> data;
        data.reserve(5);

        encode_uint8(data, static_cast<uint8_t>(endstop_trigger));                     // hmTrig: 0=LOW, 1=HIGH
        encode_uint8(data, static_cast<uint8_t>(direction));                           // hmDir: 0=CW, 1=CCW
        encode_uint16_be(data, static_cast<uint16_t>(std::abs(speed.rpm_internal()))); // HmSpeed: 0-3000 RPM
        encode_uint8(data, endlimit_enable ? 1 : 0);                                   // EndLimit: 0=disable, 1=enable

        return data;
      }

      /**
       * @brief Set the parameter of "noLimit" go home
       *
       * @details Command::SET_HOMING_CURRENT (0x94)
       * Modbus Function 0x10 (Write Multiple Registers)
       * Starting Address: 0x0094, Quantity: 4 registers (8 bytes)
       * Payload order: [retValue (uint32_t)][mode (uint16_t)][ma (uint16_t)]
       * Hardware Manual: "Set the parameter of 'noLimit' go home"
       * Note: retValue 4000 = 360°, 2000 = 180°
       *
       * @param reverse_angle Reverse angle after homing (0x4000 = 360°, 0x2000 = 180°)
       * @param sensorless_enabled false=use limit switch, true=sensorless (no limit switch)
       * @param home_current_ma Current threshold for "noLimit" go home (mA)
       */
      static std::vector<uint8_t> encode_set_nolimit_home_parameters(
          const Position &reverse_angle = Position::from_ticks(2000),
          bool sensorless_enabled = false,
          uint16_t home_current_ma = 100)
      {
        std::vector<uint8_t> data;
        data.reserve(8);

        // Payload order: retValue (4 bytes), mode (2 bytes), ma (2 bytes)
        encode_uint32_be(data, reverse_angle.get_ticks()); // retValue (4 bytes)
        uint16_t mode = sensorless_enabled ? 1 : 0;        // mode (2 bytes): 0=limit switch, 1=sensorless
        encode_uint16_be(data, mode);                      //
        encode_uint16_be(data, home_current_ma);           // ma (2 bytes): current threshold

        return data;
      }

      /**
       * @brief Remap limit switch ports (swap IN1 and IN2)
       *
       * @details Command::SET_LIMIT_PORT_REMAP (0x95)
       * Function: 0x06 (Write Single Register)
       * Payload: 1 byte - [0x00=normal, 0x01=swapped]
       * Hardware Manual: "Limit remap"
       *
       * @param enable true=swap IN1/IN2 ports, false=normal assignment
       */
      static std::vector<uint8_t> encode_set_limit_port_remap(bool enable)
      {
        return {static_cast<uint8_t>(enable ? 0x01 : 0x00)};
      }

      // ============================================================================
      // ZERO MODE
      // ============================================================================

      enum class ZeroModeMode : uint8_t
      {
        MODE_DISABLED = 0x00, // Disable - do not go back to zero
        DIR_MODE = 0x01,      // DirMode - go back to zero with direction
        NEAR_MODE = 0x02      // NearMode - go back to zero with minimum angle
      };
      enum class ZeroModeTask : uint8_t
      {
        CLEAN = 0x00, // Clean zero
        SET = 0x01    // Set zero
      };

      /**
       * @brief Set the parameter of 0_Mode
       *
       * @details Command::SET_ZERO_MODE (0x9A)
       * Function: 0x10 (Write Multiple Registers)
       * Payload: 4 bytes - [0_Mode][Set 0][0_Speed][0_Dir]
       * Hardware Manual: "Set the parameter of 0_Mode"
       * In 0_Mode, motor automatically returns to zero position on power-on (max 359°)
       *
       * @param mode 0_Mode: DISABLED=0, DIR_MODE=1 (direction-based), NEAR_MODE=2 (minimum angle)
       * @param clean_set Set 0: SET=set zero position, CLEAN=clean/clear zero
       * @param speed 0_Speed: VERY_SLOW to VERY_FAST (0-4, higher = faster)
       * @param direction 0_Dir: CW or CCW (only used when mode=DIR_MODE)
       */
      static std::vector<uint8_t> encode_set_zero_mode(
          ZeroModeMode mode = ZeroModeMode::MODE_DISABLED,
          ZeroModeTask clean_set = ZeroModeTask::CLEAN,
          ZeroingSpeed speed = ZeroingSpeed::MEDIUM,
          Direction direction = Direction::CW)
      {
        std::vector<uint8_t> data;
        data.reserve(4);

        encode_uint8(data, static_cast<uint8_t>(mode));      // Mode: 1=DIR, 2=NEAR
        encode_uint8(data, static_cast<uint8_t>(clean_set)); // Set/clean zero
        encode_uint8(data, static_cast<uint8_t>(speed));     // Speed level 0-4
        encode_uint8(data, static_cast<uint8_t>(direction)); // CW/CCW

        return data;
      }

      // ============================================================================
      // SYSTEM COMMANDS
      // ============================================================================

      /**
       * @brief Restart motor controller
       *
       * @details Command::RESTART (0x41)
       * Function: 0x06 (Write Single Register)
       * Payload: 1 byte - [0x01]
       *
       * Note: Motor needs 3-4 seconds to fully restart before accepting new commands
       */
      static std::vector<uint8_t> encode_restart() { return {0x01}; }

      /**
       * @brief Calibrate encoder
       *
       * @details Command::CALIBRATE_ENCODER (0x80)
       * Function: 0x06 (Write Single Register)
       * Payload: 1 byte - [0x00]
       */
      static std::vector<uint8_t> encode_calibrate_encoder() { return {0x00}; }

      /**
       * @brief Emergency stop
       *
       * @details Command::EMERGENCY_STOP (0xF7)
       * Function: 0x06 (Write Single Register)
       * Payload: 0 bytes (empty)
       *
       * Warning: Immediate halt without deceleration - not recommended above 1000 RPM
       */
      static std::vector<uint8_t> encode_emergency_stop() { return {}; }

      /**
       * @brief Release protection state
       *
       * @details Command::RELEASE_PROTECTION (0x0E)
       * Function: 0x06 (Write Single Register)
       * Payload: 0 bytes (empty)
       */
      static std::vector<uint8_t> encode_release_protection() { return {}; }

      // Deprecated/unused functions
      static std::vector<uint8_t> encode_restore_defaults() { return {0x01}; }
      static std::vector<uint8_t> encode_restart_motor() { return {0x01}; }
      static std::vector<uint8_t> encode_query_motor_status() { return {}; }
      static std::vector<uint8_t> encode_key_lock() { return {}; }
      static std::vector<uint8_t> encode_key_unlock() { return {}; }

      static std::vector<uint8_t> encode_save_speed_mode_params(bool save)
      {
        return {static_cast<uint8_t>(save ? 0xC8 : 0xCA)};
      }

      // ============================================================================
      // RESPONSE DECODERS
      // ============================================================================

      /**
       * @brief Decode current motor speed
       *
       * @details Command::READ_CURRENT_SPEED (0x32)
       * Function: 0x04 (Read Input Registers)
       * Response: 2 bytes - [speed_hi][speed_lo] (int16_t RPM)
       */
      static Speed decode_current_speed(const std::vector<uint8_t> &data)
      {
        if (data.size() < 2)
          return Speed(nullptr);
        int16_t rpm = static_cast<int16_t>((static_cast<int16_t>(data[0]) << 8) | data[1]);
        return Speed::from_rpm(rpm, nullptr);
      }

      /**
       * @brief Decode pulse count
       *
       * @details Command::READ_PULSE_COUNT (0x33)
       * Function: 0x04 (Read Input Registers)
       * Response: 4 bytes - [count_b3][count_b2][count_b1][count_b0] (int32_t)
       */
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

      /**
       * @brief Decode encoder addition value
       *
       * @details Command::READ_ENCODER_ADDITION (0x31)
       * Function: 0x04 (Read Input Registers)
       * Response: 6 bytes - int48_t position (signed)
       */
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

      /**
       * @brief Decode angle error
       *
       * @details Command::READ_ANGLE_ERROR (0x39)
       * Function: 0x04 (Read Input Registers)
       * Response: Same format as pulse count (4 bytes)
       */
      static Position decode_angle_error(const std::vector<uint8_t> &data)
      {
        return decode_pulse_count(data);
      }

      /**
       * @brief Decode motor enable status (unused)
       *
       * @details Command::READ_ENABLE_STATUS (0x3D)
       * Function: 0x04 (Read Input Registers)
       * Response: 2 bytes - [0x00][status] (0=disabled, 1=enabled)
       */
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

      /**
       * @brief Decode IO port status
       *
       * @details Command::READ_IO_STATUS (0x34)
       * Function: 0x04 (Read Input Registers)
       * Response: 2 bytes - [0x00][status] (bit0=IN1, bit1=IN2, bit2=OUT1, bit3=OUT2)
       */
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

      /**
       * @brief Decode zeroing/homing status
       *
       * @details Command::READ_ZEROING_STATUS (0x3C)
       * Function: 0x04 (Read Input Registers)
       * Response: 2 bytes - [0x00][status] (0=GOING_TO_ZERO, 1=SUCCESS, 2=FAILED)
       */
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

      /**
       * @brief Decode detailed motor status
       *
       * @details Command::READ_DETAILED_STATUS (0x35)
       * Function: 0x04 (Read Input Registers)
       * Response: 2 bytes - [0x00][status] (0=FAIL, 1=STOP, 2=SPEED_UP, 3=SPEED_DOWN, 4=FULL_SPEED, 5=HOMING, 6=CALIBRATING)
       */
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

      /**
       * @brief Decode command response status
       *
       * @details Generic response decoder for command execution status
       * Function: 0x06 (Write Single Register) response
       * Response: 1 byte - [status] (0=FAIL, 1=SUCCESS, 2=RUNNING, 3=ENDLIMIT_STOPPED)
       */
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
       * @brief Decode encoder value with carry/overflow tracking
       *
       * @details
       * Command::READ_ENCODER_CARRY (0x30)
       * Function: 0x04 (Read Input Registers)
       * Response: 6 bytes - [carry_b3][carry_b2][carry_b1][carry_b0][value_hi][value_lo]
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

      /**
       * @brief Decode motor status
       *
       * @details Command::READ_MOTOR_STATUS (0x3A)
       * Function: 0x04 (Read Input Registers)
       * Response: 1 byte - [status] (0=STOP, 1=MOVING, 2=HOMING)
       */
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

      enum ZeroReturnStatus
      {
        IN_PROGRESS = 0, // Go back to zero in progress
        SUCCESS = 1,     // Returned to zero successfully
        FAIL = 2         // Return to zero failed (timeout, obstacle, etc.)
      };

      /**
       * @brief Encode read zero return status command (no payload)
       *
       * @details Command::READ_ZERO_RETURN_STATUS (0x3B)
       * Function: 0x04 (Read Input Registers)
       * Register: 0x003B
       * Payload: 0 bytes (read command)
       */
      static std::vector<uint8_t> encode_read_zero_return_status()
      {
        return {}; // No payload for read commands
      }

      /**
       * @brief Read the go back to zero status
       *
       * @details Command::READ_ZERO_RETURN_STATUS (0x3B)
       * Function: 0x04 (Read Input Registers)
       * Response: 1 byte - [status] (0=IN_PROGRESS, 1=SUCCESS, 2=FAIL)
       * Hardware Manual: "Read the go back to zero status"
       * Note: This reads the status of automatic zero return (0_Mode), not endstop homing
       */
      static ZeroReturnStatus decode_zero_return_status(const std::vector<uint8_t> &data)
      {
        if (data.empty())
          return ZeroReturnStatus::FAIL;

        switch (data[0])
        {
        case 0:
          return ZeroReturnStatus::IN_PROGRESS;
          break;
        case 1:
          return ZeroReturnStatus::SUCCESS;
          break;
        case 2:
          return ZeroReturnStatus::FAIL;
        default:
          break;
        }
        return ZeroReturnStatus::FAIL;
      }

      // Legacy alias for backward compatibility
      using HomingStatus = ZeroReturnStatus;
      static HomingStatus decode_homing_status(const std::vector<uint8_t> &data)
      {
        return decode_zero_return_status(data);
      }

      struct ProtectionStatus
      {
        bool protected_state{false};
      };

      /**
       * @brief Decode protection status
       *
       * @details Command::READ_PROTECTION_STATUS (0x3E)
       * Function: 0x04 (Read Input Registers)
       * Response: 1 byte - [status] (0=OK, 1=Protected/Error)
       */
      static ProtectionStatus decode_protection_status(const std::vector<uint8_t> &data)
      {
        ProtectionStatus ps{};
        ps.protected_state = (!data.empty() && data[0] != 0);
        return ps;
      }
    };

  } // namespace servoxxd
} // namespace esphome
