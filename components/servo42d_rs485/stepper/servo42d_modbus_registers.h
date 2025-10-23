#pragma once

#include <cstdint>

namespace esphome {
namespace servo42d_rs485 {

// MODBUS-RTU Register Addresses (from Manual Part 8)
namespace ModbusRegisters {

  // Read Registers (Function 0x04)
  namespace Read {
    constexpr uint16_t ENCODER_VALUE_CARRY = 0x0030;  // 3 registers: carry(int32) + value(uint16)
    constexpr uint16_t MOTOR_SPEED = 0x0032;          // 1 register: speed in RPM (int16)
    constexpr uint16_t PULSE_COUNT = 0x0033;          // 2 registers: pulse count (int32)
    constexpr uint16_t IO_STATUS = 0x0034;            // 1 register: I/O port status (uint8)
    constexpr uint16_t ANGLE_ERROR = 0x0039;          // 2 registers: angle error (int32)
    constexpr uint16_t EN_PIN_STATUS = 0x003A;        // 1 register: EN pin status (uint8)
    constexpr uint16_t ZERO_STATUS = 0x003B;          // 1 register: go-to-zero status (uint8)
    constexpr uint16_t PROTECTION_STATUS = 0x003E;    // 1 register: protection status (uint8)
    constexpr uint16_t MOTOR_STATUS = 0x00F1;         // 1 register: motor status (uint8)
  }

  // Write Single Register (Function 0x06)
  namespace Write {
    constexpr uint16_t RELEASE_PROTECTION = 0x003D;   // Value: 0x0001
    constexpr uint16_t RESTART_MOTOR = 0x0041;        // Value: 0x0001
    constexpr uint16_t CALIBRATE_MOTOR = 0x0080;      // Value: 0x0001
    constexpr uint16_t WORK_MODE = 0x0082;            // Value: 0-5 (SR_OPEN/CLOSE/vFOC etc.)
    constexpr uint16_t WORKING_CURRENT = 0x0083;      // Value: 0-5200mA
    constexpr uint16_t SUBDIVISION = 0x0084;          // Value: 1-256 microsteps
    constexpr uint16_t EN_PIN_MODE = 0x0085;          // Value: 0-2 (L/H/Hold)
    constexpr uint16_t DIRECTION = 0x0086;            // Value: 0-1 (CW/CCW)
    constexpr uint16_t PROTECTION_ENABLE = 0x0088;    // Value: 0-1 (Disable/Enable)
    constexpr uint16_t MODBUS_RTU_ENABLE = 0x008E;    // Value: 0-1 (Disable/Enable) - CRITICAL!
    constexpr uint16_t LIMIT_REMAP = 0x009E;          // Value: 0-1 (Disable/Enable)
    constexpr uint16_t MOTOR_ENABLE = 0x00F3;         // Value: 0-1 (Disable/Enable)
    constexpr uint16_t EMERGENCY_STOP = 0x00F7;       // Value: 0x0001
  }

  // Write Multiple Registers (Function 0x10)
  namespace MultiWrite {
    constexpr uint16_t ZERO_MODE_PARAMS = 0x009A;     // 2 registers: mode, enable, speed, dir
    constexpr uint16_t POSITION_MODE_1 = 0x00FD;      // 4 registers: dir, acc, speed, pulses
    constexpr uint16_t POSITION_MODE_2 = 0x00FE;      // 4 registers: acc, speed, abs_pulses
    constexpr uint16_t POSITION_MODE_3 = 0x00F4;      // 4 registers: acc, speed, rel_axis
    constexpr uint16_t POSITION_MODE_4 = 0x00F5;      // 4 registers: acc, speed, abs_axis
  }

  // Work Mode Values
  namespace WorkMode {
    constexpr uint16_t CR_OPEN = 0;    // Pulse interface Open mode
    constexpr uint16_t CR_CLOSE = 1;   // Pulse interface Close mode  
    constexpr uint16_t CR_VFOC = 2;    // Pulse interface FOC mode
    constexpr uint16_t SR_OPEN = 3;    // Serial interface Open mode
    constexpr uint16_t SR_CLOSE = 4;   // Serial interface Close mode
    constexpr uint16_t SR_VFOC = 5;    // Serial interface FOC mode (recommended)
  }

  // Motor Status Values
  namespace MotorStatus {
    constexpr uint8_t READ_FAIL = 0;
    constexpr uint8_t MOTOR_STOP = 1;
    constexpr uint8_t MOTOR_SPEED_UP = 2;
    constexpr uint8_t MOTOR_SPEED_DOWN = 3;
    constexpr uint8_t MOTOR_FULL_SPEED = 4;
    constexpr uint8_t MOTOR_IS_HOMING = 5;
    constexpr uint8_t MOTOR_IS_CALIBRATING = 6;
  }

} // namespace ModbusRegisters

} // namespace servo42d_rs485
} // namespace esphome