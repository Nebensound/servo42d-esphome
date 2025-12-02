/**
 * @file servoxxd_commands.h
 * @brief Type-safe command identifiers for ServoXxd servo driver communication
 *
 * This header defines the Command enum which maps 1:1 to command codes used by
 * the ServoXxd closed-loop servo driver. Commands are categorized into:
 * - Read Commands (0x30-0x3F): Query motor state and sensors
 * - Configuration Commands (0x40-0x9A): Setup motor parameters
 * - Movement Commands (0xF0-0xFF): Control motor motion
 * - Special Commands: Commands outside the standard range
 *
 * Note: Some command codes represent multiple operations depending on the data
 * payload. The ServoCommandCodec class handles encoding/decoding to differentiate
 * these cases.
 *
 * @see docs/specification/02d-layer4-transport.md for protocol details
 */

#pragma once

#include <cstdint>

namespace esphome {
namespace servoxxd {

/**
 * @brief Type-safe hardware command identifiers
 *
 * Maps directly to ServoXxd command codes.
 * Used with ITransport and ServoCommandCodec for protocol-agnostic communication.
 */
enum class Command : uint8_t {
  // ==================== Read Commands (0x30-0x3F) ====================
  /// Read encoder carry value (upper 32 bits of position)
  READ_ENCODER_CARRY = 0x30,

  /// Read encoder addition value (lower 16 bits of position)
  READ_ENCODER_ADDITION = 0x31,

  /// Read current motor speed in RPM
  READ_CURRENT_SPEED = 0x32,

  /// Read pulse count (step counter)
  READ_PULSE_COUNT = 0x33,

  /// Read IO port status (limit switches, inputs)
  READ_IO_STATUS = 0x34,

  /// Read motor status (moving, stopped, homing, etc.)
  READ_MOTOR_STATUS = 0x3A,

  /// Read homing status
  READ_HOMING_STATUS = 0x3B,

  /// Read protection status (over-current, stall, etc.)
  READ_PROTECTION_STATUS = 0x3E,

  /// Restart/reset the controller
  RESTART_CONTROLLER = 0x3F,

  // ==================== Configuration Commands (0x40-0x9A) ====================
  /// Set working current in mA (configuration)
  SET_WORKING_CURRENT = 0x44,

  /// Set home parameters (direction, speed, etc.)
  SET_HOME_PARAMS = 0x4A,

  /// Calibrate encoder (zero position)
  CALIBRATE_ENCODER = 0x80,

  /// Set work mode (CR_OPEN, SR_VFOC, etc.)
  SET_WORK_MODE = 0x82,

  /// Set working current in mA (runtime change)
  /// @see docs/specification/02-cpp-interface.md line 202
  SET_WORKING_CURRENT_RUNTIME = 0x83,

  /// Set subdivision (microstepping: 1, 2, 4, 8, 16, 32, 64, etc.)
  SET_SUBDIVISION = 0x84,

  /// Set homing current threshold for sensorless homing
  /// @see docs/specification/02-cpp-interface.md line 221
  SET_HOMING_CURRENT = 0x94,

  /// Start homing sequence
  START_HOMING = 0x9A,

  // ==================== Movement Commands (0xF0-0xFF) ====================
  /// Enable or disable motor
  /// Data: 0x01 = enable, 0x00 = disable
  ENABLE_MOTOR = 0xF3,

  /// Position Mode 3: Move to absolute/relative position
  /// Data: direction, speed, acceleration, position (4 bytes)
  MOVE_POSITION_MODE_3 = 0xF4,

  /// Position Mode 4: Move to position with multi-segment profile
  MOVE_POSITION_MODE_4 = 0xF5,

  /// Speed Mode: Constant velocity rotation
  /// Data: direction, speed, acceleration
  /// Note: Speed=0 stops the motor
  MOVE_SPEED_MODE = 0xF6,

  /// Emergency stop - immediate halt
  EMERGENCY_STOP = 0xF7,

  /// Position Mode 1: Move by pulse count
  /// Data: direction, speed, acceleration, pulses (4 bytes)
  /// Note: pulses=0 stops the motor
  MOVE_POSITION_MODE_1 = 0xFD,

  /// Position Mode 2: Move to absolute position
  /// Data: direction, speed, acceleration, position (4 bytes)
  /// Note: position=0 can be used to stop
  MOVE_POSITION_MODE_2 = 0xFE,

  // ==================== Special Commands (outside standard range) ====================
  /// Release protection state (clear error)
  /// Note: This command uses 0x0E, which is outside the standard command range.
  /// This is intentional per the hardware protocol specification.
  RELEASE_PROTECTION = 0x0E,
};

}  // namespace servoxxd
}  // namespace esphome
