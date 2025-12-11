/**
 * @file servoxxd_commands.h
 * @brief Type-safe command identifiers for ServoXxd servo driver communication
 *
 * This header defines the Commandtype enum which maps 1:1 to command codes used by
 * the ServoXxd closed-loop servo driver. Commands are categorized into:
 * - Read Commands (0x30-0x3F): Query motor state and sensors
 * - Configuration Commands (0x40-0x9A): Setup motor parameters
 * - Movement Commands (0xF0-0xFF): Control motor motion
 * - Special Commands: Commands outside the standard range
 *
 * Note: Some command codes represent multiple operations depending on the data
 * payload. The CommandDecoder class handles encoding/decoding to differentiate
 * these cases.
 *
 * @see docs/specification/02d-layer4-transport.md for protocol details
 */

#pragma once

#include <cstdint>
#include <vector>

namespace esphome
{
  namespace servoxxd
  {

    /**
     * @brief Type-safe hardware command identifiers
     *
     * Maps directly to ServoXxd command codes.
     * Used with ITransport and CommandDecoder for protocol-agnostic communication.
     */
    enum class Commandtype : uint8_t
    {
      // ==================== Read Commands (0x30-0x3F) ====================
      READ_ENCODER_CARRY = 0x30,      /// Read encoder carry value (upper 32 bits of position)
      READ_ENCODER_ADDITION = 0x31,   /// Read encoder addition value (lower 16 bits of position)
      READ_CURRENT_SPEED = 0x32,      /// Read current motor speed in RPM
      READ_PULSE_COUNT = 0x33,        /// Read pulse count (step counter)
      READ_IO_STATUS = 0x34,          /// Read IO port status (limit switches, inputs)
      READ_ANGLE_ERROR = 0x39,        /// Read angle error (position deviation)
      READ_ENABLE_STATUS = 0x3A,      /// Read enable pin status (0=disabled, 1=enabled)
      READ_ZERO_RETURN_STATUS = 0x3B, /// Read the go back to zero status (0_Mode auto-return)
      READ_PROTECTION_STATUS = 0x3E,  /// Read protection status (over-current, stall, etc.)
      RESTART_CONTROLLER = 0x3F,      /// Restart/reset the controller

      // ==================== Configuration Commands (0x40-0x9A) ====================
      SET_WORKING_CURRENT = 0x44,         /// Set working current in mA (configuration)
      SET_HOME_PARAMS = 0x4A,             /// Set home parameters (direction, speed, etc.)
      CALIBRATE_ENCODER = 0x80,           /// Calibrate encoder (zero position)
      SET_WORK_MODE = 0x82,               /// Set work mode (CR_OPEN, SR_VFOC, etc.)
      SET_WORKING_CURRENT_RUNTIME = 0x83, /// Set working current in mA (runtime change)
      SET_SUBDIVISION = 0x84,             /// Set subdivision (microstepping: 1, 2, 4, 8, 16, 32, 64, etc.)
      SET_EN_PIN_ACTIVE = 0x85,           /// Set EN pin active level (0=LOW, 1=HIGH, 2=ALWAYS/Hold)
      SET_AUTO_SCREEN_OFF = 0x87,         /// Set auto screen off (0=disabled, 1=enabled)
      SET_LOCK_KEYS = 0x8F,               /// Set key lock (0=unlock, 1=lock)
      SET_HOLDING_CURRENT_PERCENT = 0x9B, /// Set holding current percentage (0-8 for 10%-90%)
      SET_HOMING_PARAMETERS = 0x90,       /// Set ENDSTOP homing parameters (Fn 0x10, Reg 0x0090, 5 bytes)
      GO_HOME = 0x91,                     /// Start homing sequence (go to home/zero position)
      SET_ZERO = 0x92,                    /// Set current encoder position as zero reference
      SET_NOLIMIT_HOMING_PARAMS = 0x94,   /// Set the parameter of "noLimit" go home (Fn 0x10, Reg 0x0094, 8 bytes)
      SET_LIMIT_PORT_REMAP = 0x95,        /// Remap limit switch ports - swap IN1/IN2 (Fn 0x10, Reg 0x0095, 1 byte)
      SET_ZERO_MODE = 0x9A,               /// Set 0_Mode auto-return parameters (Fn 0x10, Reg 0x009A, 4 bytes)

      // ==================== Movement Commands (0xF0-0xFF) ====================
      READ_MOTOR_STATUS = 0xF1,    /// Read motor status (0=fail, 1=stop, 2=speed_up, 3=speed_down, 4=full_speed, 5=homing, 6=calibrating)
      ENABLE_MOTOR = 0xF3,         /// Enable or disable motor (0x01=enable, 0x00=disable)
      MOVE_POSITION_MODE_3 = 0xF4, /// Position Mode 3: Move to absolute/relative position
      MOVE_POSITION_MODE_4 = 0xF5, /// Position Mode 4: Move to position with multi-segment profile
      MOVE_SPEED_MODE = 0xF6,      /// Speed Mode: Constant velocity rotation (Speed=0 stops motor)
      EMERGENCY_STOP = 0xF7,       /// Emergency stop - immediate halt
      MOVE_POSITION_MODE_1 = 0xFD, /// Position Mode 1: Move by pulse count (pulses=0 stops motor)
      MOVE_POSITION_MODE_2 = 0xFE, /// Position Mode 2: Move to absolute position
      STOP_POSITION_MODE_2 = 0xFF, /// Stop in Position Mode 2 (deceleration stop)

      // ==================== Special Commands (outside standard range) ====================
      RELEASE_PROTECTION = 0x3D, /// Release protection state (clear error) - Register 0x003D
      RESTART = 0x41,            /// Restart/reset the motor controller
    };

    /**
     * @brief Commandtype metadata and payload container
     *
     * Encapsulates all information needed to execute a Modbus command.
     * All metadata (function_code, register_address, expected_response_length)
     * is derived from the command enum value.
     */
    struct Command
    {
      const Commandtype command_type; /// Command type from Commandtype enum (immutable)
      std::vector<uint8_t> payload;   /// Payload data for write commands (empty for reads)
      std::vector<uint8_t> response;  /// Response data received from motor (populated after execution)

      Command(Commandtype cmd, const std::vector<uint8_t> &payload_data = {})
          : command_type(cmd), payload(payload_data)
      {
      }

      /// Get Modbus function code (computed from command type)
      uint8_t function_code() const;

      /// Get register address (same as command code)
      uint16_t register_address() const
      {
        return static_cast<uint16_t>(command_type);
      }

      /// Get expected response length (computed from command type)
      uint8_t expected_response_length() const;
    };

    // ============================================================================
    // Inline implementations (must be in header for inline expansion)
    // ============================================================================

    inline uint8_t Command::function_code() const
    {
      // Read commands use Function 0x04 (Read Input Registers)
      if (command_type >= Commandtype::READ_ENCODER_CARRY && command_type <= Commandtype::READ_ZERO_RETURN_STATUS)
      {
        return 0x04;
      }

      // Write commands: Function code depends on command type (not payload size!)
      // - Function 0x06 (Write Single Register): Simple configuration commands
      // - Function 0x10 (Write Multiple Registers): Complex multi-parameter commands
      switch (command_type)
      {
      // Function 0x10 (Write Multiple Registers) - Complex commands with multiple parameters
      case Commandtype::SET_HOMING_PARAMETERS:     // 0x90: 5 bytes (trigger, direction, speed, endlimit)
      case Commandtype::SET_NOLIMIT_HOMING_PARAMS: // 0x94: 8 bytes
      case Commandtype::SET_ZERO_MODE:             // 0x9A: 4 bytes (mode, enable, speed, direction)
      case Commandtype::MOVE_POSITION_MODE_1:      // 0xFD: 8 bytes (dir, acc, speed, pulses)
      case Commandtype::MOVE_POSITION_MODE_2:      // 0xFE: 8 bytes (acc, speed, absPulses)
      case Commandtype::MOVE_POSITION_MODE_3:      // 0xF4: variable bytes
      case Commandtype::MOVE_POSITION_MODE_4:      // 0xF5: variable bytes (multi-segment)
      case Commandtype::MOVE_SPEED_MODE:           // 0xF6: 4 bytes (dir, acc, speed)
        return 0x10;

      // Function 0x06 (Write Single Register) - All other write commands
      default:
        return 0x06;
      }
    }

    inline uint8_t Command::expected_response_length() const
    {
      // Only read commands have response data
      if (command_type < Commandtype::READ_ENCODER_CARRY || command_type > Commandtype::READ_ZERO_RETURN_STATUS)
      {
        return 0; // Write commands have no payload response
      }

      // Read command payload lengths
      switch (command_type)
      {
      case Commandtype::READ_ENCODER_CARRY:
        return 6; // carry (4 bytes) + value (2 bytes)
      case Commandtype::READ_ENCODER_ADDITION:
        return 6; // int48_t position
      case Commandtype::READ_PULSE_COUNT:
      case Commandtype::READ_ANGLE_ERROR:
        return 4; // int32_t
      case Commandtype::READ_CURRENT_SPEED:
      case Commandtype::READ_MOTOR_STATUS:
      case Commandtype::READ_PROTECTION_STATUS:
      case Commandtype::READ_IO_STATUS:
      case Commandtype::READ_ZERO_RETURN_STATUS:
        return 2; // uint16_t / int16_t
      default:
        return 2; // Default to 1 register = 2 bytes
      }
    }

  } // namespace servoxxd
} // namespace esphome
