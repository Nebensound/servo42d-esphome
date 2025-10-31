#pragma once

#include "esphome/core/component.h"
#include "esphome/components/modbus/modbus.h"
#include <vector>
#include <functional>
#include <memory>

namespace esphome
{
  namespace servoxxd
  {

    /**
     * @brief Command state machine for Modbus commands
     * 
     * State transitions:
     * - (created) → PENDING: Command queued
     * - PENDING → EXECUTING: Command sent to hardware
     * - EXECUTING → COMPLETED: Valid response received
     * - EXECUTING → FAILED: Modbus error received
     * - EXECUTING → TIMEOUT: No response within timeout
     */
    enum class CommandState
    {
      PENDING,   // Command in queue, not yet sent
      EXECUTING, // Command sent to hardware, waiting for response
      COMPLETED, // Response received and processed successfully
      FAILED,    // Modbus error received (exception code)
      TIMEOUT    // No response within timeout period
    };

    /**
     * @brief MKS SERVO42D/57D Modbus Register Addresses
     * 
     * Based on: MKS_SERVO42D57D_RS485_User_Manual_V1.0.5
     * These register addresses are hardware-specific to the MKS servo motors.
     * 
     * Protocol: MODBUS-RTU (standard compliant)
     * - Function 0x04: Read Input Registers
     * - Function 0x06: Write Single Register
     * - Function 0x10: Write Multiple Registers
     */
    namespace ModbusRegisters
    {
      // ========================================================================
      // READ REGISTERS (Function 0x04 - Read Input Registers)
      // ========================================================================

      /** @brief Encoder value (carry + value, 3 registers: carry[int32] + value[uint16])
       *  carry: encoder overflow counter
       *  value: current encoder position (0~0x3FFF)
       *  When value > 0x3FFF: carry += 1
       *  When value < 0: carry -= 1
       */
      constexpr uint16_t ENCODER_CARRY = 0x30;

      /** @brief Encoder value (addition, 3 registers: int48_t)
       *  Accumulated encoder position
       *  After one turn CW: value += 0x4000
       *  After one turn CCW: value -= 0x4000
       */
      constexpr uint16_t ENCODER_ADDITION = 0x31;

      /** @brief Real-time motor speed (1 register: int16_t RPM)
       *  > 0: CCW rotation
       *  < 0: CW rotation
       */
      constexpr uint16_t SPEED_REALTIME = 0x32;

      /** @brief Number of pulses received (2 registers: uint32_t) */
      constexpr uint16_t PULSES_COUNT = 0x33;

      /** @brief IO Ports status (1 register: uint8_t bitmask)
       *  Bit layout: [reserved][OUT_2][OUT_1][IN_2][IN_1]
       *  Note: With limit remap enabled, IN_1→En, IN_2→Dir
       */
      constexpr uint16_t IO_STATUS = 0x34;

      /** @brief Angle error (2 registers: int32_t)
       *  Error = target_angle - actual_angle
       *  Range: 0~51200 corresponds to 0~360°
       *  Example: 1° error = 142.222 units
       */
      constexpr uint16_t ANGLE_ERROR = 0x39;

      /** @brief En pin status (1 register: uint8_t)
       *  1 = Enabled
       *  0 = Disabled
       */
      constexpr uint16_t EN_PIN_STATUS = 0x3A;

      /** @brief Homing status (1 register: uint8_t)
       *  0 = Going to zero
       *  1 = Go to zero success
       *  2 = Go to zero fail
       */
      constexpr uint16_t HOMING_STATUS = 0x3B;

      /** @brief Motor shaft protection status (1 register: uint8_t)
       *  1 = Protected (locked-rotor detected)
       *  0 = Not protected
       */
      constexpr uint16_t PROTECTION_STATUS = 0x3E;

      /** @brief Motor status (1 register: uint8_t)
       *  0 = Query fail
       *  1 = Motor stop
       *  2 = Motor speed up
       *  3 = Motor speed down
       *  4 = Motor full speed
       *  5 = Motor homing
       *  6 = Motor calibrating
       */
      constexpr uint16_t MOTOR_STATUS = 0xF1;

      // ========================================================================
      // WRITE REGISTERS (Function 0x06 - Write Single Register)
      // ========================================================================

      /** @brief Release locked-rotor protection (write 0x01) */
      constexpr uint16_t RELEASE_PROTECTION = 0x3D;

      /** @brief Restore default parameters (write 0x01) */
      constexpr uint16_t RESTORE_DEFAULTS = 0x3F;

      /** @brief Restart motor (write 0x01) */
      constexpr uint16_t RESTART_MOTOR = 0x41;

      /** @brief Calibrate motor (write 0x01) - motor must be unloaded */
      constexpr uint16_t CALIBRATE_MOTOR = 0x80;

      /** @brief Set work mode (write mode 0-5)
       *  0 = CR_OPEN, 1 = CR_CLOSE, 2 = CR_vFOC
       *  3 = SR_OPEN, 4 = SR_CLOSE, 5 = SR_vFOC
       */
      constexpr uint16_t WORK_MODE = 0x82;

      /** @brief Set work current (write mA)
       *  SERVO42D/28D/35D: Max 3000mA
       *  SERVO57D: Max 5200mA
       */
      constexpr uint16_t WORK_CURRENT = 0x83;

      /** @brief Set subdivision/microstepping (write 1-256) */
      constexpr uint16_t SUBDIVISION = 0x84;

      /** @brief Set En pin active level (write 0-2)
       *  0 = Active low, 1 = Active high, 2 = Always enabled
       */
      constexpr uint16_t EN_PIN_ACTIVE = 0x85;

      /** @brief Set motor direction (write 0-1)
       *  0 = CW, 1 = CCW
       *  Note: Only for pulse interface
       */
      constexpr uint16_t MOTOR_DIRECTION = 0x86;

      /** @brief Auto turn off screen (write 0-1) */
      constexpr uint16_t AUTO_SCREEN_OFF = 0x87;

      /** @brief Motor shaft protection enable (write 0-1) */
      constexpr uint16_t SHAFT_PROTECTION = 0x88;

      /** @brief Subdivision interpolation enable (write 0-1) */
      constexpr uint16_t SUBDIVISION_INTERP = 0x89;

      /** @brief Baud rate (write 1-7)
       *  1=9600, 2=19200, 3=25000, 4=38400, 5=57600, 6=115200, 7=256000
       */
      constexpr uint16_t BAUD_RATE = 0x8A;

      /** @brief Slave address (write 0-255, 0=broadcast) */
      constexpr uint16_t SLAVE_ADDRESS = 0x8B;

      /** @brief MODBUS-RTU enable (write 0-1) */
      constexpr uint16_t MODBUS_RTU_ENABLE = 0x8E;

      /** @brief Key lock (write 0-1) */
      constexpr uint16_t KEY_LOCK = 0x8F;

      /** @brief Set current axis to zero (write 0x01) */
      constexpr uint16_t SET_AXIS_ZERO = 0x92;

      /** @brief Holding current percentage (write 0-8)
       *  0=10%, 1=20%, ..., 8=90%
       *  Note: Only for OPEN/CLOSE modes, not vFOC
       */
      constexpr uint16_t HOLDING_CURRENT_PCT = 0x9B;

      /** @brief Limit port remap enable (write 0-1)
       *  Remaps: Left limit → En port, Right limit → Dir port
       */
      constexpr uint16_t LIMIT_PORT_REMAP = 0x9E;

      /** @brief Serial mode motor enable (write 0-1)
       *  Only for SR_OPEN/SR_CLOSE/SR_vFOC modes
       */
      constexpr uint16_t MOTOR_ENABLE = 0xF3;

      /** @brief Emergency stop (write 0x01)
       *  Warning: Don't use above 1000 RPM!
       */
      constexpr uint16_t EMERGENCY_STOP = 0xF7;

      /** @brief Save/Clean speed mode parameters (write 0xC8/0xCA)
       *  0xC8 = Save, 0xCA = Clean
       */
      constexpr uint16_t SAVE_SPEED_PARAMS = 0xFF;

      // ========================================================================
      // MULTI-WRITE REGISTERS (Function 0x10 - Write Multiple Registers)
      // ========================================================================

      /** @brief Home parameters (3 registers)
       *  Register layout: [HmTrig][HmDir][HmSpeed(2 bytes)][EndLimit]
       *  HmTrig: 0=Low, 1=High
       *  HmDir: 0=CW, 1=CCW
       *  HmSpeed: 0-3000 RPM
       *  EndLimit: 0=Disable, 1=Enable
       */
      constexpr uint16_t HOME_PARAMS = 0x90;

      /** @brief Zero mode parameters (2 registers)
       *  Register layout: [0_Mode][Set0][0_Speed][0_Dir]
       *  0_Mode: 0=Disable, 1=DirMode, 2=NearMode
       *  Set0: 0=Clean zero, 1=Set zero
       *  0_Speed: 0-4 (speed level)
       *  0_Dir: 0=CW, 1=CCW
       */
      constexpr uint16_t ZERO_MODE_PARAMS = 0x9A;

      /** @brief Position mode3: Relative motion by axis (4 registers)
       *  Register layout: [Speed(2)][Acc(2)][RelAxis(4)]
       *  Speed: 0-3000 RPM (uint16_t)
       *  Acc: 0-255 (uint16_t)
       *  RelAxis: Relative axis position (int32_t)
       */
      constexpr uint16_t POSITION_RELATIVE_AXIS = 0xF4;

      /** @brief Position mode4: Absolute motion by axis (4 registers)
       *  Register layout: [Speed(2)][Acc(2)][AbsAxis(4)]
       *  Speed: 0-3000 RPM (uint16_t)
       *  Acc: 0-255 (uint16_t)
       *  AbsAxis: Absolute axis position (int32_t)
       *  Note: Supports real-time updates
       */
      constexpr uint16_t POSITION_ABSOLUTE_AXIS = 0xF5;

      /** @brief Speed mode (2 registers)
       *  Register layout: [Dir+Speed(12bit)][Acc]
       *  Dir: Bit 15 (0=CCW, 1=CW)
       *  Speed: Bits 0-11 (0-3000 RPM)
       *  Acc: 0-255
       */
      constexpr uint16_t SPEED_MODE = 0xF6;

      /** @brief Position mode1: Relative motion by pulses (4 registers)
       *  Register layout: [Dir+Speed(12bit)][Acc][Pulses(4)]
       *  Dir: Bit 15 (0=CCW, 1=CW)
       *  Speed: Bits 0-11 (0-3000 RPM)
       *  Acc: 0-255 (uint8_t)
       *  Pulses: Number of steps (uint32_t)
       */
      constexpr uint16_t POSITION_RELATIVE_PULSES = 0xFD;

      /** @brief Position mode2: Absolute motion by pulses (4 registers)
       *  Register layout: [Speed(2)][Acc(2)][AbsPulses(4)]
       *  Speed: 0-3000 RPM (uint16_t)
       *  Acc: 0-255 (uint16_t)
       *  AbsPulses: Absolute pulse position (int32_t)
       */
      constexpr uint16_t POSITION_ABSOLUTE_PULSES = 0xFE;

    } // namespace ModbusRegisters

    /**
     * @brief Abstract base class for Modbus commands
     * 
     * Provides common functionality for command lifecycle:
     * - State machine (PENDING → EXECUTING → terminal states)
     * - Timeout tracking
     * - Callback mechanism (completion + optional data callback)
     */
    class BaseCommand
    {
    public:
      BaseCommand(uint16_t register_address, uint32_t timeout_ms = 1000)
          : register_address_(register_address), timeout_ms_(timeout_ms) {}

      virtual ~BaseCommand() = default;

      // Pure virtual methods - must be implemented by derived classes
      virtual void execute(modbus::ModbusDevice *device) = 0;
      virtual bool process_response(const std::vector<uint8_t> &data) = 0;
      virtual uint8_t get_function_code() const = 0;
      virtual const char *get_command_name() const = 0;

      // State management
      CommandState get_state() const { return state_; }
      void set_state(CommandState state) { state_ = state; }

      // Timeout management
      void start_timeout(uint32_t current_time_ms)
      {
        start_time_ms_ = current_time_ms;
        state_ = CommandState::EXECUTING;
      }

      bool is_timeout(uint32_t current_time_ms) const
      {
        if (state_ != CommandState::EXECUTING)
          return false;
        return (current_time_ms - start_time_ms_) > timeout_ms_;
      }

      uint32_t get_elapsed_time(uint32_t current_time_ms) const
      {
        return current_time_ms - start_time_ms_;
      }

      // Callback management
      void set_completion_callback(std::function<void(bool success)> callback)
      {
        completion_callback_ = std::move(callback);
      }

      void set_data_callback(std::function<void(const std::vector<uint8_t> &)> callback)
      {
        data_callback_ = std::move(callback);
      }

      void invoke_completion_callback(bool success)
      {
        if (completion_callback_)
        {
          completion_callback_(success);
        }
      }

      void invoke_data_callback(const std::vector<uint8_t> &data)
      {
        if (data_callback_)
        {
          data_callback_(data);
        }
      }

      // Accessors
      uint16_t get_register_address() const { return register_address_; }
      uint32_t get_timeout_ms() const { return timeout_ms_; }

    protected:
      CommandState state_{CommandState::PENDING};
      uint16_t register_address_;
      uint32_t timeout_ms_;
      uint32_t start_time_ms_{0};
      std::function<void(bool success)> completion_callback_;
      std::function<void(const std::vector<uint8_t> &)> data_callback_;
    };

    /**
     * @brief Read Input Registers command (Modbus function code 0x04)
     * 
     * Reads multiple input registers from the servo.
     * Response format: [byte_count, reg1_hi, reg1_lo, reg2_hi, reg2_lo, ...]
     */
    class ReadCommand : public BaseCommand
    {
    public:
      ReadCommand(uint16_t register_address, uint16_t quantity, uint32_t timeout_ms = 1000)
          : BaseCommand(register_address, timeout_ms), quantity_(quantity) {}

      void execute(modbus::ModbusDevice *device) override;
      bool process_response(const std::vector<uint8_t> &data) override;
      uint8_t get_function_code() const override { return 0x04; }
      const char *get_command_name() const override { return "ReadInputRegisters"; }

      uint16_t get_quantity() const { return quantity_; }

    private:
      uint16_t quantity_;
    };

    /**
     * @brief Write Single Register command (Modbus function code 0x06)
     * 
     * Writes a single register to the servo.
     * Response format: [reg_addr_hi, reg_addr_lo, value_hi, value_lo] (echo)
     */
    class WriteCommand : public BaseCommand
    {
    public:
      WriteCommand(uint16_t register_address, uint16_t value, uint32_t timeout_ms = 1000)
          : BaseCommand(register_address, timeout_ms), value_(value) {}

      void execute(modbus::ModbusDevice *device) override;
      bool process_response(const std::vector<uint8_t> &data) override;
      uint8_t get_function_code() const override { return 0x06; }
      const char *get_command_name() const override { return "WriteSingleRegister"; }

      uint16_t get_value() const { return value_; }

    private:
      uint16_t value_;
    };

    /**
     * @brief Write Multiple Registers command (Modbus function code 0x10)
     * 
     * Writes multiple consecutive registers to the servo.
     * Response format: [reg_addr_hi, reg_addr_lo, quantity_hi, quantity_lo]
     */
    class MultiWriteCommand : public BaseCommand
    {
    public:
      MultiWriteCommand(uint16_t register_address, const std::vector<uint16_t> &values,
                        uint32_t timeout_ms = 1000)
          : BaseCommand(register_address, timeout_ms), values_(values) {}

      void execute(modbus::ModbusDevice *device) override;
      bool process_response(const std::vector<uint8_t> &data) override;
      uint8_t get_function_code() const override { return 0x10; }
      const char *get_command_name() const override { return "WriteMultipleRegisters"; }

      const std::vector<uint16_t> &get_values() const { return values_; }
      uint16_t get_quantity() const { return values_.size(); }

    private:
      std::vector<uint16_t> values_;
    };

  } // namespace servoxxd
} // namespace esphome
