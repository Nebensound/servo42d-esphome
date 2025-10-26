#pragma once

#include <cstdint>

namespace esphome
{
  namespace servo42d_rs485
  {

    // MODBUS-RTU Register Addresses (from Manual Part 8)
    namespace ModbusRegisters
    {

      // Read Registers (Function 0x04)
      namespace Read
      {
        constexpr uint16_t ENCODER_VALUE_CARRY = 0x0030;    // 3 registers: carry(int32) + value(uint16)
        constexpr uint16_t ENCODER_VALUE_ADDITION = 0x0031; // 3 registers: int48_t (addition mode)
        constexpr uint16_t MOTOR_SPEED = 0x0032;            // 1 register: speed in RPM (int16)
        constexpr uint16_t PULSE_COUNT = 0x0033;            // 2 registers: pulse count (int32)
        constexpr uint16_t IO_STATUS = 0x0034;              // 1 register: I/O port status (uint8)
        constexpr uint16_t ANGLE_ERROR = 0x0039;            // 2 registers: angle error (int32)
        constexpr uint16_t EN_PIN_STATUS = 0x003A;          // 1 register: EN pin status (uint8)
        constexpr uint16_t ZERO_STATUS = 0x003B;            // 1 register: go-to-zero status (uint8)
        constexpr uint16_t PROTECTION_STATUS = 0x003E;      // 1 register: protection status (uint8)
        constexpr uint16_t MOTOR_STATUS = 0x00F1;           // 1 register: motor status (uint8)
      }

      // Write Single Register (Function 0x06)
      namespace Write
      {
        constexpr uint16_t RELEASE_PROTECTION_STATE = 0x003D;  // Value: 0x0001
        constexpr uint16_t SYSTEM_RESET = 0x0041;              // Value: 0x0001 - DANGEROUS: Reboots motor!
        constexpr uint16_t CALIBRATION_COMMAND = 0x0080;       // Value: 0x0001
        constexpr uint16_t SET_CURRENT_POSITION_ZERO = 0x0092; // Value: 0x0001 - Sets position to 0 (no movement!)
        constexpr uint16_t WORK_MODE = 0x0082;                 // Value: 0-5 (SR_OPEN/CLOSE/vFOC etc.)
        constexpr uint16_t WORKING_CURRENT = 0x0083;           // Value: 0-5200mA
        constexpr uint16_t SUBDIVISION = 0x0084;               // Value: 1-256 microsteps
        constexpr uint16_t EN_ACTIVE = 0x0085;                 // Value: 0-2 (L/H/Hold)
        constexpr uint16_t DIRECTION = 0x0086;                 // Value: 0-1 (CW/CCW)
        constexpr uint16_t AUTO_SCREEN_OFF = 0x0087;           // Value: 0-1 (Disable/Enable)
        constexpr uint16_t PROTECTION_ENABLE = 0x0088;         // Value: 0-1 (Disable/Enable)
        constexpr uint16_t KEY_LOCK = 0x008F;                  // Value: 0-1 (Unlock/Lock)
        constexpr uint16_t HOLDING_CURRENT_PERCENT = 0x009B;   // Value: 10-90%
        constexpr uint16_t MODBUS_RTU_ENABLE = 0x008E;         // Value: 0-1 (Disable/Enable) - CRITICAL!
        constexpr uint16_t LIMIT_REMAP = 0x009E;               // Value: 0-1 (Disable/Enable)
        constexpr uint16_t EN_CONTROL = 0x00F3;                // Value: 0-1 (Disable/Enable)
        constexpr uint16_t SPEED_MODE = 0x00F6;                // Value: RPM (int16, signed for direction)
        constexpr uint16_t EMERGENCY_STOP = 0x00F7;            // Value: 0x0098
      }

      // Write Multiple Registers (Function 0x10)
      namespace MultiWrite
      {
        // Per manual Part 8 (Set the parameter of home): start address 0x0090, quantity 0x0003 (4 bytes)
        constexpr uint16_t HOMING_PARAMS = 0x0090; // 4 registers: HmTrig, HmDir, HmSpeed, EndLimit
        // Note: 0x009A is used for 0_Mode parameters (power-on go-to-zero), not instant homing
        constexpr uint16_t ZERO_MODE_PARAMS = 0x009A; // 4 registers: mode, enable, speed, dir
        constexpr uint16_t POSITION_MODE_1 = 0x00FD;  // 4 registers: dir, acc, speed, pulses
        constexpr uint16_t POSITION_MODE_2 = 0x00FE;  // 4 registers: acc, speed, abs_pulses
        constexpr uint16_t POSITION_MODE_3 = 0x00F4;  // 4 registers: acc, speed, rel_axis
        constexpr uint16_t POSITION_MODE_4 = 0x00F5;  // 4 registers: acc, speed, abs_axis
        constexpr uint16_t SPEED_MODE = 0x00F6;       // 3 registers: dir, acc, speed (per manual 8.3.3)
      }

      // Payload Structures for MultiWrite commands
      namespace Payload
      {
        // Homing parameters (0x0090) - Manual 8.3.2
        struct HomingParams
        {
          uint16_t hm_trig;   // 0=Low active, 1=High active
          uint16_t hm_dir;    // 0=CW, 1=CCW
          uint16_t hm_speed;  // Speed in RPM (0-3000)
          uint16_t end_limit; // 0=Disabled, 1=Enabled
        };

        // Zero Mode parameters (0x009A) - Power-on go-to-zero
        struct ZeroModeParams
        {
          uint16_t mode;   // Angle in degrees (0-360)
          uint16_t enable; // 0=Disabled, 1=Enabled
          uint16_t speed;  // Speed in RPM (0-3000)
          uint16_t dir;    // 0=CW, 1=CCW
        };

        // Speed Mode (0x00F6) - Manual 8.3.3
        struct SpeedMode
        {
          uint16_t dir;   // 0=CW, 1=CCW
          uint16_t acc;   // Acceleration 0-255 (internal units)
          uint16_t speed; // Speed in RPM (0-3000)
        };

        // Position Mode 1 (0x00FD) - Relative pulses
        struct PositionMode1
        {
          uint16_t dir;    // 0=CW, 1=CCW
          uint16_t acc;    // Acceleration 0-255
          uint16_t speed;  // Speed in RPM (0-3000)
          uint16_t pulses; // Pulse count (relative)
        };

        // Position Mode 2 (0x00FE) - Absolute pulses
        struct PositionMode2
        {
          uint16_t acc;             // Acceleration 0-255
          uint16_t speed;           // Speed in RPM (0-3000)
          uint16_t abs_pulses_high; // High 16 bits of absolute pulse position
          uint16_t abs_pulses_low;  // Low 16 bits of absolute pulse position
        };

        // Position Mode 3 (0x00F4) - Relative axis
        struct PositionMode3
        {
          uint16_t acc;           // Acceleration 0-255
          uint16_t speed;         // Speed in RPM (0-3000)
          uint16_t rel_axis_high; // High 16 bits of relative axis movement
          uint16_t rel_axis_low;  // Low 16 bits of relative axis movement
        };

        // Position Mode 4 (0x00F5) - Absolute axis
        struct PositionMode4
        {
          uint16_t acc;           // Acceleration 0-255
          uint16_t speed;         // Speed in RPM (0-3000)
          uint16_t abs_axis_high; // High 16 bits of absolute axis position
          uint16_t abs_axis_low;  // Low 16 bits of absolute axis position
        };
      }

      // Work Mode Values
      namespace WorkMode
      {
        constexpr uint16_t CR_OPEN = 0;  // Pulse interface Open mode
        constexpr uint16_t CR_CLOSE = 1; // Pulse interface Close mode
        constexpr uint16_t CR_VFOC = 2;  // Pulse interface FOC mode
        constexpr uint16_t SR_OPEN = 3;  // Serial interface Open mode
        constexpr uint16_t SR_CLOSE = 4; // Serial interface Close mode
        constexpr uint16_t SR_VFOC = 5;  // Serial interface FOC mode (recommended)
      }

      // Motor Status Values
      namespace MotorStatus
      {
        constexpr uint8_t READ_FAIL = 0;
        constexpr uint8_t MOTOR_STOP = 1;
        constexpr uint8_t MOTOR_SPEED_UP = 2;
        constexpr uint8_t MOTOR_SPEED_DOWN = 3;
        constexpr uint8_t MOTOR_FULL_SPEED = 4;
        constexpr uint8_t MOTOR_IS_HOMING = 5;
        constexpr uint8_t MOTOR_IS_CALIBRATING = 6;
      }

      // Additional single-write actions (Function 0x06)
      namespace Write
      {
        // Go Home trigger (per manual 8.3.2) - write 0x0001 to start
        constexpr uint16_t GO_HOME = 0x0091;
      }

    } // namespace ModbusRegisters

  } // namespace servo42d_rs485
} // namespace esphome