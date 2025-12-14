#pragma once

#include "servoxxd.h"
#include "servoxxd_commands.h"
#include <vector>
#include <cstdint>
#include <cmath>

namespace esphome
{
  namespace servoxxd
  {
    /**
     * @brief Modbus response decoder for ServoXxd
     *
     * Contains only decode functions for parsing hardware responses.
     * Each decoder validates that the correct Commandtype is provided.
     * Encode functions have been moved to CommandFactory.
     */
    class CommandDecoder
    {
    private:
      static const char *TAG;

      /**
       * @brief Validate command type for decoder
       *
       * @param cmd Command object to validate
       * @param expected Expected command type
       * @return true if command type matches, false otherwise
       */
      static bool validate_command_type(const Command &cmd, Commandtype expected)
      {
        if (cmd.command_type != expected)
        {
          ESP_LOGE(TAG, "Invalid command type: expected 0x%04X, got 0x%04X",
                   static_cast<uint16_t>(expected), cmd.register_address());
          return false;
        }
        return true;
      }

    public:
      // ============================================================================
      // RESPONSE DECODERS
      // ============================================================================

      /**
       * @brief Decode current motor speed
       *
       * @details Commandtype::READ_CURRENT_SPEED (0x32)
       * Function: 0x04 (Read Input Registers)
       * Response: 2 bytes - [speed_hi][speed_lo] (int16_t RPM)
       *
       * @param cmd Command object with command_type=READ_CURRENT_SPEED and response data
       * @return Speed object with current motor speed in RPM, or default Speed on error
       */
      static Speed read_current_speed(const Command &cmd)
      {
        if (!validate_command_type(cmd, Commandtype::READ_CURRENT_SPEED))
          return Speed(nullptr);

        const auto &data = cmd.response;
        if (data.size() < 2)
          return Speed(nullptr);
        int16_t rpm = static_cast<int16_t>((static_cast<int16_t>(data[0]) << 8) | data[1]);
        return Speed::from_rpm(rpm, nullptr);
      }

      /**
       * @brief Decode pulse count
       *
       * @details Commandtype::READ_PULSE_COUNT (0x33)
       * Function: 0x04 (Read Input Registers)
       * Response: 4 bytes - [count_b3][count_b2][count_b1][count_b0] (int32_t)
       *
       * @param cmd Command object with command_type=READ_PULSE_COUNT and response data
       * @return Position object with pulse count in ticks, or default Position on error
       */
      static Position read_pulse_count(const Command &cmd)
      {
        if (!validate_command_type(cmd, Commandtype::READ_PULSE_COUNT))
          return Position(nullptr);

        const auto &data = cmd.response;
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
       * @details Commandtype::READ_ENCODER_ADDITION (0x31)
       * Function: 0x04 (Read Input Registers)
       * Response: 6 bytes - int48_t position (signed)
       *
       * @param cmd Command object with command_type=READ_ENCODER_ADDITION and response data
       * @return Position object with encoder position in ticks (int48_t), or default Position on error
       */
      static Position read_encoder_addition(const Command &cmd)
      {
        if (!validate_command_type(cmd, Commandtype::READ_ENCODER_ADDITION))
          return Position(nullptr);

        const auto &data = cmd.response;
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
       * @details Commandtype::READ_ANGLE_ERROR (0x39)
       * Function: 0x04 (Read Input Registers)
       * Response: Same format as pulse count (4 bytes)
       *
       * @param cmd Command object with command_type=READ_ANGLE_ERROR and response data
       * @return Position object with angle error in ticks, or default Position on error
       */
      static Position read_angle_error(const Command &cmd)
      {
        if (!validate_command_type(cmd, Commandtype::READ_ANGLE_ERROR))
          return Position(nullptr);

        const auto &data = cmd.response;
        if (data.size() < 4)
          return Position(nullptr);
        int32_t ticks = (static_cast<int32_t>(data[0]) << 24) |
                        (static_cast<int32_t>(data[1]) << 16) |
                        (static_cast<int32_t>(data[2]) << 8) |
                        static_cast<int32_t>(data[3]);
        return Position::from_ticks(ticks);
      }

      /**
       * @brief Decode motor enable status (unused)
       *
       * @details Commandtype::READ_ENABLE_STATUS (0x3D)
       * Function: 0x04 (Read Input Registers)
       * Response: 2 bytes - [0x00][status] (0=disabled, 1=enabled)
       *
       * @param data Response data vector (2 bytes minimum)
       * @return true if motor is enabled, false otherwise
       */
      static bool read_enable_status(const std::vector<uint8_t> &data)
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
       * @details Commandtype::READ_IO_STATUS (0x34)
       * Function: 0x04 (Read Input Registers)
       * Response: 2 bytes - [0x00][status] (bit0=IN1, bit1=IN2, bit2=OUT1, bit3=OUT2)
       *
       * @param data Response data vector (2 bytes minimum)
       * @return IOPortStatus struct with in1, in2, out1, out2 boolean values
       */
      static IOPortStatus read_io_port_status(const std::vector<uint8_t> &data)
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
       * @details Commandtype::READ_ZEROING_STATUS (0x3C)
       * Function: 0x04 (Read Input Registers)
       * Response: 2 bytes - [0x00][status] (0=GOING_TO_ZERO, 1=SUCCESS, 2=FAILED)
       *
       * @param data Response data vector (2 bytes minimum)
       * @return ZeroingStatus struct with state (GOING_TO_ZERO, SUCCESS, or FAILED)
       */
      static ZeroingStatus read_zeroing_status(const std::vector<uint8_t> &data)
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
       * @details Commandtype::READ_DETAILED_STATUS (0x35)
       * Function: 0x04 (Read Input Registers)
       * Response: 2 bytes - [0x00][status] (0=FAIL, 1=STOP, 2=SPEED_UP, 3=SPEED_DOWN, 4=FULL_SPEED, 5=HOMING, 6=CALIBRATING)
       *
       * @param data Response data vector (2 bytes minimum)
       * @return DetailedMotorStatus struct with state (FAIL, STOP, SPEED_UP, SPEED_DOWN, FULL_SPEED, HOMING, or CALIBRATING)
       */
      static DetailedMotorStatus read_detailed_motor_status(const std::vector<uint8_t> &data)
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
       *
       * @param data Response data vector (1 byte minimum)
       * @return CommandResponse struct with status (FAIL, SUCCESS, RUNNING, or ENDLIMIT_STOPPED)
       */
      static CommandResponse read_command_response(const std::vector<uint8_t> &data)
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
       * Commandtype::READ_ENCODER_CARRY (0x30)
       * Function: 0x04 (Read Input Registers)
       * Response: 6 bytes - [carry_b3][carry_b2][carry_b1][carry_b0][value_hi][value_lo]
       *
       * Hardware returns carry (int32_t) + value (uint16_t, 0-0x3FFF).
       * Absolute position = carry × 0x4000 + value
       *
       * Example: carry=5, value=0x1234 → position = 0x14234 encoder ticks
       *
       * Note: Use read_encoder_addition() for direct int48_t position (Commandtype 0x31).
       *
       * @param cmd Command object with command_type=READ_ENCODER_CARRY and response data
       * @param parent Optional ServoXxd parent for position conversion context
       * @return Position object with absolute encoder position in ticks, or default Position on error
       */
      static Position read_encoder_carry(const Command &cmd, const ServoXxd *parent = nullptr)
      {
        if (!validate_command_type(cmd, Commandtype::READ_ENCODER_CARRY))
          return Position(parent);

        const auto &data = cmd.response;
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

      /**
       * @brief Motor status states
       *
       * Hardware response values for READ_MOTOR_STATUS (0x3A):
       * 0 = FAIL       - Motor read fail
       * 1 = STOP       - Motor is stopped
       * 2 = SPEED_UP   - Motor is accelerating
       * 3 = SPEED_DOWN - Motor is decelerating
       * 4 = FULL_SPEED - Motor at full speed
       * 5 = HOMING     - Motor is executing homing sequence
       * 6 = CALIBRATING - Motor is calibrating
       */
      enum class MotorStatus : uint8_t
      {
        FAIL = 0,
        STOP = 1,
        SPEED_UP = 2,
        SPEED_DOWN = 3,
        FULL_SPEED = 4,
        HOMING = 5,
        CALIBRATING = 6
      };

      /**
       * @brief Decode motor status
       *
       * @details Commandtype::READ_MOTOR_STATUS (0x3A)
       * Function: 0x04 (Read Input Registers)
       * Register: 0x003A
       * Response: 1 byte - [status] (0=FAIL, 1=STOP, 2=SPEED_UP, 3=SPEED_DOWN, 4=FULL_SPEED, 5=HOMING, 6=CALIBRATING)
       *
       * Hardware Manual: "Read motor motion status"
       * - FAIL (0): Motor read fail
       * - STOP (1): Motor standstill, ready for commands
       * - SPEED_UP (2): Motor accelerating
       * - SPEED_DOWN (3): Motor decelerating
       * - FULL_SPEED (4): Motor at constant full speed
       * - HOMING (5): Motor executing homing/zeroing sequence
       * - CALIBRATING (6): Motor calibrating
       *
       * @param cmd Command object with command_type=READ_MOTOR_STATUS and response data
       * @return MotorStatus enum (FAIL, STOP, SPEED_UP, SPEED_DOWN, FULL_SPEED, HOMING, or CALIBRATING)
       */
      static MotorStatus read_motor_status(const Command &cmd)
      {
        if (!validate_command_type(cmd, Commandtype::READ_MOTOR_STATUS))
          return MotorStatus::FAIL;

        const auto &data = cmd.response;
        if (data.empty())
          return MotorStatus::FAIL;

        switch (data[0])
        {
        case 0:
          return MotorStatus::FAIL;
        case 1:
          return MotorStatus::STOP;
        case 2:
          return MotorStatus::SPEED_UP;
        case 3:
          return MotorStatus::SPEED_DOWN;
        case 4:
          return MotorStatus::FULL_SPEED;
        case 5:
          return MotorStatus::HOMING;
        case 6:
          return MotorStatus::CALIBRATING;
        default:
          ESP_LOGW(TAG, "Unknown motor status value: %u", data[0]);
          return MotorStatus::FAIL;
        }
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
       * @details Commandtype::READ_ZERO_RETURN_STATUS (0x3B)
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
       * @details Commandtype::READ_ZERO_RETURN_STATUS (0x3B)
       * Function: 0x04 (Read Input Registers)
       * Response: 1 byte - [status] (0=IN_PROGRESS, 1=SUCCESS, 2=FAIL)
       * Hardware Manual: "Read the go back to zero status"
       * Note: This reads the status of automatic zero return (0_Mode), not endstop homing
       *
       * @param cmd Command object with command_type=READ_ZERO_RETURN_STATUS and response data
       * @return ZeroReturnStatus enum (IN_PROGRESS, SUCCESS, or FAIL)
       */
      static ZeroReturnStatus read_zero_return_status(const Command &cmd)
      {
        if (!validate_command_type(cmd, Commandtype::READ_ZERO_RETURN_STATUS))
          return ZeroReturnStatus::FAIL;

        const auto &data = cmd.response;
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
      /**
       * @brief Legacy alias for read_zero_return_status
       * @param cmd Command object with command_type=READ_ZERO_RETURN_STATUS and response data
       * @return HomingStatus enum (same as ZeroReturnStatus)
       */
      static HomingStatus read_homing_status(const Command &cmd)
      {
        return read_zero_return_status(cmd);
      }

      struct ProtectionStatus
      {
        bool protected_state{false};
      };

      /**
       * @brief Decode protection status
       *
       * @details Commandtype::READ_PROTECTION_STATUS (0x3E)
       * Function: 0x04 (Read Input Registers)
       * Response: 1 byte - [status] (0=OK, 1=Protected/Error)
       *
       * @param cmd Command object with command_type=READ_PROTECTION_STATUS and response data
       * @return ProtectionStatus struct with protected_state boolean (true if protected/error)
       */
      static ProtectionStatus read_protection_status(const Command &cmd)
      {
        ProtectionStatus ps{};
        if (!validate_command_type(cmd, Commandtype::READ_PROTECTION_STATUS))
          return ps;

        const auto &data = cmd.response;
        ps.protected_state = (!data.empty() && data[0] != 0);
        return ps;
      }

      struct AllConfigData
      {
        ControlMode mode{ControlMode::SR_OPEN};
        uint8_t holding_current_percent{50};
        uint16_t working_current_ma{2000};
        uint8_t subdivision{16};
        EnPinActive en_pin_active{EnPinActive::EN_LOW};
        bool shaft_reversed{false};
        bool auto_screen_off{true};
        uint8_t protect_enable{0};
        uint8_t mplyer{0};
        uint8_t baud_rate{1};
        uint8_t slave_address{1};
        uint8_t group_address{0};
        bool respond_enable{true};
        bool active_enable{false};
        bool modbus_enable{true};
        bool key_lock{false};
        EndstopTrigger homing_trigger{EndstopTrigger::TRIGGER_LOW};
        Direction homing_direction{Direction::CW};
        uint16_t homing_speed_rpm{0};
        bool endlimit_enable{false};
        uint32_t nolimit_reverse_angle_ticks{0};
        bool nolimit_mode{false};
        uint16_t nolimit_current_ma{1000};
        bool limit_port_remap{false};
        uint8_t zero_mode{0};
        uint8_t zero_task{0};
        uint8_t zero_speed{2};
        Direction zero_direction{Direction::CW};
      };

      /**
       * @brief Decode all configuration parameters
       *
       * @details Commandtype::READ_ALL_CONFIG (0x1147)
       * Function: 0x04 (Read Input Registers)
       * Response: 38 bytes (19 registers) containing all motor configuration
       * 
       * Register breakdown (matching write_all_config):
       * - REG1 (2B): Mode [mode][reserved]
       * - REG2 (2B): Hold current [hw_hold][reserved]
       * - REG3 (2B): Work current [hi][lo]
       * - REG4 (2B): Subdivision [subdivision][reserved]
       * - REG5 (2B): En + Dir [en_pin_active][shaft_reversed]
       * - REG6 (2B): AutoSDD + Protect [auto_screen_off][protect_enable]
       * - REG7 (2B): Mplyer + NULL [mplyer][reserved]
       * - REG8 (2B): Baud + Slave [baud_rate][slave_address]
       * - REG9 (2B): Group + Respond [group_address][respond_active]
       * - REG10 (2B): MODBUS + Key [modbus_enable][key_lock]
       * - REG11-13 (6B): Homing params [trigger][direction][speed_hi][speed_lo][null][endlimit]
       * - REG14-16 (8B): No-limit homing [reverse_angle(4)][mode(2)][current_ma(2)]
       * - REG17 (2B): Remap [null][limit_port_remap]
       * - REG18-19 (4B): 0_Mode [zero_mode][zero_task][zero_speed][zero_direction]
       *
       * @param cmd Command object with command_type=READ_ALL_CONFIG and response data
       * @return AllConfigData struct with decoded configuration parameters
       */
      static AllConfigData read_all_config(const Command &cmd)
      {
        AllConfigData config{};
        if (!validate_command_type(cmd, Commandtype::READ_ALL_CONFIG))
          return config;

        const auto &data = cmd.response;
        if (data.size() < 38)
        {
          ESP_LOGW(TAG, "Invalid READ_ALL_CONFIG response size: %zu bytes (expected 38)", data.size());
          return config;
        }

        size_t idx = 0;

        // REG1: Mode (2 bytes)
        config.mode = static_cast<ControlMode>(data[idx++]);
        idx++; // Reserved

        // REG2: Hold current (2 bytes)
        uint8_t hw_hold = data[idx++];
        config.holding_current_percent = (hw_hold >= 8) ? 90 : ((hw_hold + 1) * 10);
        idx++; // Reserved

        // REG3: Work current (2 bytes)
        config.working_current_ma = (static_cast<uint16_t>(data[idx]) << 8) | data[idx + 1];
        idx += 2;

        // REG4: Subdivision (2 bytes)
        config.subdivision = data[idx++];
        idx++; // Reserved

        // REG5: En + Dir (2 bytes)
        config.en_pin_active = static_cast<EnPinActive>(data[idx++]);
        config.shaft_reversed = (data[idx++] != 0);

        // REG6: AutoSDD + Protect (2 bytes)
        config.auto_screen_off = (data[idx++] != 0);
        config.protect_enable = data[idx++];

        // REG7: Mplyer + NULL (2 bytes)
        config.mplyer = data[idx++];
        idx++; // Reserved

        // REG8: Baud rate + Slave address (2 bytes)
        config.baud_rate = data[idx++];
        config.slave_address = data[idx++];

        // REG9: Group address + Respond/Active (2 bytes)
        config.group_address = data[idx++];
        uint8_t respond_active = data[idx++];
        config.respond_enable = (respond_active & 0x01) != 0;
        config.active_enable = ((respond_active >> 1) & 0x01) != 0;

        // REG10: MODBUS + Key lock (2 bytes)
        config.modbus_enable = (data[idx++] != 0);
        config.key_lock = (data[idx++] != 0);

        // REG11-13: Homing parameters (6 bytes)
        config.homing_trigger = static_cast<EndstopTrigger>(data[idx++]);
        config.homing_direction = static_cast<Direction>(data[idx++]);
        config.homing_speed_rpm = (static_cast<uint16_t>(data[idx]) << 8) | data[idx + 1];
        idx += 2;
        idx++; // NULL
        config.endlimit_enable = (data[idx++] != 0);

        // REG14-16: No-limit homing (8 bytes)
        config.nolimit_reverse_angle_ticks = (static_cast<uint32_t>(data[idx]) << 24) |
                                              (static_cast<uint32_t>(data[idx + 1]) << 16) |
                                              (static_cast<uint32_t>(data[idx + 2]) << 8) |
                                              static_cast<uint32_t>(data[idx + 3]);
        idx += 4;
        config.nolimit_mode = ((static_cast<uint16_t>(data[idx]) << 8) | data[idx + 1]) != 0;
        idx += 2;
        config.nolimit_current_ma = (static_cast<uint16_t>(data[idx]) << 8) | data[idx + 1];
        idx += 2;

        // REG17: Remap + NULL (2 bytes)
        idx++; // NULL
        config.limit_port_remap = (data[idx++] != 0);

        // REG18-19: 0_Mode parameters (4 bytes)
        config.zero_mode = data[idx++];
        config.zero_task = data[idx++];
        config.zero_speed = data[idx++];
        config.zero_direction = static_cast<Direction>(data[idx++]);

        return config;
      }
    };

  } // namespace servoxxd
} // namespace esphome
