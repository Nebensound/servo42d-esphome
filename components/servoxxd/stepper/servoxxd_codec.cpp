#include "servoxxd_codec.h"
#include "servoxxd.h"
#include "servoxxd_position.h"
#include "servoxxd_speed.h"
#include "servoxxd_acceleration.h"

namespace esphome
{
  namespace servoxxd
  {

    // ============================================================================
    // Movement Encoders
    // ============================================================================

    std::vector<uint8_t> ServoCommandCodec::encode_move_position_mode_2(
        int32_t position, uint16_t speed, uint8_t accel, Direction direction)
    {
      // Validate speed range (0-3000 RPM typical)
      if (speed > 3000)
      {
        return {};
      }

      std::vector<uint8_t> data(8);

      // Byte 0: Direction (0=CW, 1=CCW)
      data[0] = static_cast<uint8_t>(direction);

      // Bytes 1-2: Speed (big-endian)
      data[1] = static_cast<uint8_t>((speed >> 8) & 0xFF);
      data[2] = static_cast<uint8_t>(speed & 0xFF);

      // Byte 3: Acceleration
      data[3] = accel;

      // Bytes 4-7: Position (big-endian, signed)
      data[4] = static_cast<uint8_t>((position >> 24) & 0xFF);
      data[5] = static_cast<uint8_t>((position >> 16) & 0xFF);
      data[6] = static_cast<uint8_t>((position >> 8) & 0xFF);
      data[7] = static_cast<uint8_t>(position & 0xFF);

      return data;
    }

    std::vector<uint8_t> ServoCommandCodec::encode_move_position_mode_3(
        const Position &target,
        const Speed &speed,
        const Acceleration &accel,
        bool absolute)
    {
      std::vector<uint8_t> data(10);

      // Get internal values from the classes
      int32_t revs = target.revolutions();
      uint16_t angle = target.angle_ticks();
      int16_t rpm = speed.rpm_internal();
      uint8_t acc = accel.acc_internal();

      // Validate revolution range - hardware uses int16_t for revolutions
      // Range: -32768 to 32767 revolutions
      if (revs < -32768 || revs > 32767)
      {
        return {}; // Out of range
      }

      // Determine direction from RPM sign (or position for absolute)
      Direction dir = (rpm >= 0) ? Direction::CW : Direction::CCW;
      uint16_t abs_rpm = static_cast<uint16_t>(rpm >= 0 ? rpm : -rpm);

      // Validate speed range
      if (abs_rpm > 3000)
      {
        return {};
      }

      // Byte 0: Direction
      data[0] = static_cast<uint8_t>(dir);

      // Bytes 1-2: Speed (big-endian)
      data[1] = static_cast<uint8_t>((abs_rpm >> 8) & 0xFF);
      data[2] = static_cast<uint8_t>(abs_rpm & 0xFF);

      // Byte 3: Acceleration
      data[3] = acc;

      // Hardware format for mode 3:
      // Bytes 4-5: Revolutions (int16_t, big-endian)
      // Bytes 6-7: Angle ticks (uint16_t, big-endian)
      int16_t revs_16 = static_cast<int16_t>(revs);

      data[4] = static_cast<uint8_t>((revs_16 >> 8) & 0xFF);
      data[5] = static_cast<uint8_t>(revs_16 & 0xFF);
      data[6] = static_cast<uint8_t>((angle >> 8) & 0xFF);
      data[7] = static_cast<uint8_t>(angle & 0xFF);

      // Byte 8: Reserved (0x00)
      data[8] = 0x00;

      // Byte 9: Absolute flag (0x01 = absolute, 0x00 = relative)
      data[9] = absolute ? 0x01 : 0x00;

      return data;
    }

    std::vector<uint8_t> ServoCommandCodec::encode_stop_position_mode_2(uint8_t decel)
    {
      // Stop command uses position=0, speed=0 with specified deceleration
      return encode_move_position_mode_2(0, 0, decel, Direction::CW);
    }

    std::vector<uint8_t> ServoCommandCodec::encode_move_speed_mode(
        uint16_t speed, uint8_t accel, Direction direction)
    {
      // Validate speed range
      if (speed > 3000)
      {
        return {};
      }

      std::vector<uint8_t> data(4);

      // Byte 0: Direction
      data[0] = static_cast<uint8_t>(direction);

      // Bytes 1-2: Speed (big-endian)
      data[1] = static_cast<uint8_t>((speed >> 8) & 0xFF);
      data[2] = static_cast<uint8_t>(speed & 0xFF);

      // Byte 3: Acceleration
      data[3] = accel;

      return data;
    }

    std::vector<uint8_t> ServoCommandCodec::encode_emergency_stop()
    {
      return {0x01};
    }

    // ============================================================================
    // Configuration Encoders
    // ============================================================================

    std::vector<uint8_t> ServoCommandCodec::encode_enable_motor(bool enable)
    {
      return {enable ? static_cast<uint8_t>(0x01) : static_cast<uint8_t>(0x00)};
    }

    std::vector<uint8_t> ServoCommandCodec::encode_set_working_current(uint16_t mA)
    {
      // Validate current range (0-5000mA typical)
      if (mA > 5000)
      {
        return {};
      }

      std::vector<uint8_t> data(2);
      // Big-endian
      data[0] = static_cast<uint8_t>((mA >> 8) & 0xFF);
      data[1] = static_cast<uint8_t>(mA & 0xFF);

      return data;
    }

    std::vector<uint8_t> ServoCommandCodec::encode_set_subdivision(uint16_t microsteps)
    {
      // Validate microstepping value (1-256)
      if (microsteps == 0 || microsteps > 256)
      {
        return {};
      }

      // Value 256 is encoded as 0
      uint8_t encoded = (microsteps == 256) ? 0 : static_cast<uint8_t>(microsteps);
      return {encoded};
    }

    std::vector<uint8_t> ServoCommandCodec::encode_release_protection()
    {
      return {0x01};
    }

    std::vector<uint8_t> ServoCommandCodec::encode_start_homing(Direction direction, uint8_t speed_level)
    {
      // Validate speed level (0-4)
      if (speed_level > 4)
      {
        speed_level = 4;
      }

      std::vector<uint8_t> data(2);
      data[0] = static_cast<uint8_t>(direction);
      data[1] = speed_level;

      return data;
    }

    // ============================================================================
    // Response Decoders
    // ============================================================================

    EncoderValue ServoCommandCodec::decode_encoder_carry(const std::vector<uint8_t> &data)
    {
      EncoderValue result;

      if (data.size() < 6)
      {
        return result; // Return default values on error
      }

      // Bytes 0-3: Carry (int32_t, big-endian)
      result.carry = static_cast<int32_t>(
          (static_cast<uint32_t>(data[0]) << 24) |
          (static_cast<uint32_t>(data[1]) << 16) |
          (static_cast<uint32_t>(data[2]) << 8) |
          static_cast<uint32_t>(data[3]));

      // Bytes 4-5: Value (uint16_t, big-endian)
      result.value = static_cast<uint16_t>(
          (static_cast<uint16_t>(data[4]) << 8) |
          static_cast<uint16_t>(data[5]));

      return result;
    }

    Position ServoCommandCodec::decode_encoder_position(
        const std::vector<uint8_t> &data,
        const ServoXxd *parent)
    {
      if (data.size() < 6 || parent == nullptr)
      {
        return Position(parent);
      }

      EncoderValue enc = decode_encoder_carry(data);

      // Create Position from ticks (carry = revolutions, value = angle_ticks)
      // Total ticks = revolutions * 16384 + angle_ticks
      int64_t total_ticks = static_cast<int64_t>(enc.carry) * 16384 + enc.value;

      return Position(total_ticks, PositionUnit::TICKS, parent);
    }

    int16_t ServoCommandCodec::decode_current_speed(const std::vector<uint8_t> &data)
    {
      if (data.size() < 2)
      {
        return 0;
      }

      // Big-endian signed int16
      return static_cast<int16_t>(
          (static_cast<uint16_t>(data[0]) << 8) |
          static_cast<uint16_t>(data[1]));
    }

    Speed ServoCommandCodec::decode_motor_speed(
        const std::vector<uint8_t> &data,
        const ServoXxd *parent)
    {
      if (parent == nullptr)
      {
        return Speed(nullptr);
      }

      int16_t rpm = decode_current_speed(data);
      return Speed(static_cast<float>(rpm), SpeedUnit::RPM, parent);
    }

    int32_t ServoCommandCodec::decode_pulse_count(const std::vector<uint8_t> &data)
    {
      if (data.size() < 4)
      {
        return 0;
      }

      // Big-endian signed int32
      return static_cast<int32_t>(
          (static_cast<uint32_t>(data[0]) << 24) |
          (static_cast<uint32_t>(data[1]) << 16) |
          (static_cast<uint32_t>(data[2]) << 8) |
          static_cast<uint32_t>(data[3]));
    }

    MotorStatus ServoCommandCodec::decode_motor_status(const std::vector<uint8_t> &data)
    {
      MotorStatus status;

      if (data.empty())
      {
        return status;
      }

      uint8_t byte = data[0];

      // Bits 0-2: State
      status.state = static_cast<MotorStatus::State>(byte & 0x07);

      // Bit 3: Enabled
      status.enabled = (byte & 0x08) != 0;

      // Bit 4: Shaft protected
      status.shaft_protected = (byte & 0x10) != 0;

      // Bit 5: Stall flag
      status.stall_flag = (byte & 0x20) != 0;

      return status;
    }

    ProtectionStatus ServoCommandCodec::decode_protection_status(const std::vector<uint8_t> &data)
    {
      ProtectionStatus status;

      if (data.empty())
      {
        return status;
      }

      uint8_t byte = data[0];

      // Bit 0: Protected state
      status.protected_state = (byte & 0x01) != 0;

      // Bits 1-7: Protection code
      status.protection_code = (byte >> 1) & 0x7F;

      return status;
    }

    uint8_t ServoCommandCodec::decode_homing_status(const std::vector<uint8_t> &data)
    {
      if (data.empty())
      {
        return 0;
      }
      return data[0];
    }

    uint8_t ServoCommandCodec::decode_io_status(const std::vector<uint8_t> &data)
    {
      if (data.empty())
      {
        return 0;
      }
      return data[0];
    }

  } // namespace servoxxd
} // namespace esphome
