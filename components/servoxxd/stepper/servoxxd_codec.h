#pragma once

#include <cstdint>
#include <vector>

namespace esphome
{
  namespace servoxxd
  {

    // Forward declarations
    class ServoXxd;
    class Position;
    class Speed;
    class Acceleration;

    // Direction enum is already defined in servoxxd.h
    // We forward-declare it here for header-only usage
    enum class Direction : uint8_t;

    /**
     * @brief Motor status from READ_MOTOR_STATUS response
     */
    struct MotorStatus
    {
      enum class State : uint8_t
      {
        STOP = 0,
        ACCEL = 1,
        DECEL = 2,
        FULL_SPEED = 3,
        HOMING = 4,
        CALIBRATING = 5
      };
      State state{State::STOP};
      bool enabled{false};
      bool shaft_protected{false};
      bool stall_flag{false};
    };

    /**
     * @brief Protection status from READ_PROTECTION_STATUS response
     */
    struct ProtectionStatus
    {
      bool protected_state{false};
      uint8_t protection_code{0};
    };

    /**
     * @brief Encoder value with carry (revolutions) and value (angle ticks)
     */
    struct EncoderValue
    {
      int32_t carry{0};     // Full revolutions (signed)
      uint16_t value{0};    // Angle ticks (0-16383)
    };

    /**
     * @brief Encode/decode command payloads for ServoXxd motor communication
     *
     * From spec (02d-layer4-transport.md):
     * - Pure static functions (no state)
     * - Encoders: Validate ranges, return empty {} on error
     * - Decoders: Check buffer size, return default values on error
     * - Byte ordering: Big-endian (MSB first)
     *
     * @see 02d-layer4-transport.md section "Component 3: ServoCommandCodec"
     */
    class ServoCommandCodec
    {
    public:
      // ============================================================================
      // Movement Encoders
      // ============================================================================

      /**
       * @brief Encode move_position_mode_2 command (0xFE)
       *
       * Format: [dir:1][speed_hi:1][speed_lo:1][accel:1][pos3:1][pos2:1][pos1:1][pos0:1]
       * Position: 32-bit signed pulse count (big-endian)
       *
       * @param position Target position (absolute pulse count)
       * @param speed Speed in motor RPM units (0-3000)
       * @param accel Acceleration value (0-255)
       * @param direction Movement direction (CW/CCW)
       * @return Encoded bytes (8 bytes), empty on error
       */
      static std::vector<uint8_t> encode_move_position_mode_2(
          int32_t position, uint16_t speed, uint8_t accel, Direction direction);

      /**
       * @brief Encode move_position_mode_3 command using Position/Speed/Acceleration objects (0xF4)
       *
       * Format: [dir:1][speed_hi:1][speed_lo:1][accel:1][pos3:1][pos2:1][pos1:1][pos0:1][0:1][abs:1]
       * Position: Using Position class (converted to ticks internally)
       *
       * @param target Target position
       * @param speed Movement speed
       * @param accel Acceleration
       * @param absolute True for absolute positioning, false for relative
       * @return Encoded bytes (10 bytes), empty on error
       */
      static std::vector<uint8_t> encode_move_position_mode_3(
          const Position &target,
          const Speed &speed,
          const Acceleration &accel,
          bool absolute);

      /**
       * @brief Encode stop_position_mode_2 command (0xFE with position=0)
       *
       * @param decel Deceleration value (0-255)
       * @return Encoded bytes (8 bytes)
       */
      static std::vector<uint8_t> encode_stop_position_mode_2(uint8_t decel);

      /**
       * @brief Encode move_speed_mode command (0xF6)
       *
       * Format: [dir:1][speed_hi:1][speed_lo:1][accel:1]
       *
       * @param speed Speed in RPM (0-3000)
       * @param accel Acceleration value (0-255)
       * @param direction Movement direction
       * @return Encoded bytes (4 bytes), empty on error
       */
      static std::vector<uint8_t> encode_move_speed_mode(
          uint16_t speed, uint8_t accel, Direction direction);

      /**
       * @brief Encode emergency_stop command (0xF7)
       *
       * @return Encoded bytes (1 byte: 0x01)
       */
      static std::vector<uint8_t> encode_emergency_stop();

      // ============================================================================
      // Configuration Encoders
      // ============================================================================

      /**
       * @brief Encode enable_motor command (0xF3)
       *
       * @param enable True to enable, false to disable
       * @return Encoded bytes (1 byte)
       */
      static std::vector<uint8_t> encode_enable_motor(bool enable);

      /**
       * @brief Encode set_working_current command (0x44)
       *
       * @param mA Working current in milliamps (0-5000)
       * @return Encoded bytes (2 bytes, big-endian), empty on error
       */
      static std::vector<uint8_t> encode_set_working_current(uint16_t mA);

      /**
       * @brief Encode set_subdivision command (0x84)
       *
       * @param microsteps Microstepping value (1-256)
       * @return Encoded bytes (1 byte), empty on error
       */
      static std::vector<uint8_t> encode_set_subdivision(uint16_t microsteps);

      /**
       * @brief Encode release_protection command (0x0E)
       *
       * @return Encoded bytes (1 byte: 0x01)
       */
      static std::vector<uint8_t> encode_release_protection();

      /**
       * @brief Encode start_homing command (0x9A)
       *
       * @param direction Homing direction
       * @param speed_level Homing speed level (0-4)
       * @return Encoded bytes (2 bytes)
       */
      static std::vector<uint8_t> encode_start_homing(Direction direction, uint8_t speed_level);

      // ============================================================================
      // Response Decoders
      // ============================================================================

      /**
       * @brief Decode encoder carry response (0x30)
       *
       * Response format: [carry3:1][carry2:1][carry1:1][carry0:1][value_hi:1][value_lo:1]
       *
       * @param data Response data (6 bytes expected)
       * @return Decoded encoder value
       */
      static EncoderValue decode_encoder_carry(const std::vector<uint8_t> &data);

      /**
       * @brief Decode encoder position to Position object
       *
       * @param data Response data from READ_ENCODER_CARRY
       * @param parent Parent ServoXxd (for unit conversions)
       * @return Decoded Position object
       */
      static Position decode_encoder_position(
          const std::vector<uint8_t> &data,
          const ServoXxd *parent);

      /**
       * @brief Decode current speed response (0x32)
       *
       * Response format: [speed_hi:1][speed_lo:1] (signed int16)
       *
       * @param data Response data (2 bytes expected)
       * @return Speed in RPM (signed, negative = CCW)
       */
      static int16_t decode_current_speed(const std::vector<uint8_t> &data);

      /**
       * @brief Decode motor speed to Speed object
       *
       * @param data Response data from READ_CURRENT_SPEED
       * @param parent Parent ServoXxd (for unit conversions)
       * @return Decoded Speed object
       */
      static Speed decode_motor_speed(
          const std::vector<uint8_t> &data,
          const ServoXxd *parent);

      /**
       * @brief Decode pulse count response (0x33)
       *
       * Response format: [count3:1][count2:1][count1:1][count0:1] (signed int32)
       *
       * @param data Response data (4 bytes expected)
       * @return Pulse count (signed)
       */
      static int32_t decode_pulse_count(const std::vector<uint8_t> &data);

      /**
       * @brief Decode motor status response (0x3A)
       *
       * Response format: [status_byte:1]
       * - Bits 0-2: State (0=stop, 1=accel, 2=decel, 3=full_speed, 4=homing, 5=calibrating)
       * - Bit 3: Enabled flag
       * - Bit 4: Shaft protected
       * - Bit 5: Stall flag
       *
       * @param data Response data (1 byte expected)
       * @return Decoded motor status
       */
      static MotorStatus decode_motor_status(const std::vector<uint8_t> &data);

      /**
       * @brief Decode protection status response (0x3E)
       *
       * Response format: [protection_byte:1]
       * - Bit 0: Protected state
       * - Bits 1-7: Protection code
       *
       * @param data Response data (1 byte expected)
       * @return Decoded protection status
       */
      static ProtectionStatus decode_protection_status(const std::vector<uint8_t> &data);

      /**
       * @brief Decode homing status response (0x3B)
       *
       * Response format: [status_byte:1]
       * - 0: Not started
       * - 1: In progress
       * - 2: Completed successfully
       * - 3: Failed
       *
       * @param data Response data (1 byte expected)
       * @return Homing status code
       */
      static uint8_t decode_homing_status(const std::vector<uint8_t> &data);

      /**
       * @brief Decode IO ports status response (0x34)
       *
       * Response format: [io_byte:1]
       *
       * @param data Response data (1 byte expected)
       * @return IO port status byte
       */
      static uint8_t decode_io_status(const std::vector<uint8_t> &data);
    };

  } // namespace servoxxd
} // namespace esphome
