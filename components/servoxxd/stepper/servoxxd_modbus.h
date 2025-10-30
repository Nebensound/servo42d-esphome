#pragma once

#include "esphome/core/component.h"
#include "esphome/components/modbus/modbus.h"
#include <vector>
#include <functional>

namespace esphome
{
  namespace servoxxd
  {

    // TODO: Implement Modbus command classes for ServoXxd component
    // 
    // This file should contain the Modbus-specific command implementations:
    // - BaseCommand abstract class
    // - ReadCommand (0x04 - Read Input Registers)
    // - WriteCommand (0x06 - Write Single Register)
    // - MultiWriteCommand (0x10 - Write Multiple Registers)
    // 
    // See spec: docs/specification/02-cpp-interface.md Lines 1045-1150
    // 
    // Key features to implement:
    // 1. Command state machine (PENDING → EXECUTING → COMPLETED/FAILED/TIMEOUT)
    // 2. Timeout tracking with millisecond precision
    // 3. Response validation and parsing
    // 4. Completion callbacks (success/failure)
    // 5. Data callbacks for read operations
    // 
    // Example structure:
    /*
    enum class CommandState {
      PENDING,
      EXECUTING,
      COMPLETED,
      FAILED,
      TIMEOUT
    };

    class BaseCommand {
     public:
      virtual void execute(modbus::ModbusDevice* device) = 0;
      virtual void process_response(const std::vector<uint8_t>& data) = 0;
      virtual uint8_t get_function_code() const = 0;
      virtual const char* get_command_name() const = 0;
      
      CommandState state_{CommandState::PENDING};
      uint16_t register_address_;
      uint32_t timeout_ms_{1000};
      uint32_t start_time_ms_{0};
      std::function<void(bool success)> completion_callback_;
      std::function<void(const std::vector<uint8_t>&)> data_callback_;
    };

    class ReadCommand : public BaseCommand {
      // Function code 0x04 - Read Input Registers
    };

    class WriteCommand : public BaseCommand {
      // Function code 0x06 - Write Single Register
    };

    class MultiWriteCommand : public BaseCommand {
      // Function code 0x10 - Write Multiple Registers
    };
    */

  } // namespace servoxxd
} // namespace esphome
