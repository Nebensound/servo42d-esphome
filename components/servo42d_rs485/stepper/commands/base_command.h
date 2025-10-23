#pragma once

#include "esphome/core/component.h"
#include <vector>
#include <functional>
#include <cstdint>

namespace esphome
{
  namespace servo42d_rs485
  {

    enum class CommandState
    {
      PENDING,   // Waiting to be executed
      EXECUTING, // Currently running
      COMPLETED, // Successfully finished
      FAILED,    // Error occurred
      TIMEOUT    // Timed out
    };

    enum class ModbusFunction : uint8_t
    {
      READ_INPUT_REGISTERS = 0x04,
      WRITE_SINGLE_REGISTER = 0x06,
      WRITE_MULTIPLE_REGISTERS = 0x10
    };

  } // namespace servo42d_rs485

  // Forward declaration
  namespace modbus
  {
    class ModbusDevice;
  }

  namespace servo42d_rs485
  {

    class BaseCommand
    {
    public:
      using CompletionCallback = std::function<void(BaseCommand *, bool success)>;
      using DataCallback = std::function<void(const std::vector<uint8_t> &)>;

      BaseCommand(uint16_t register_address, uint32_t timeout_ms = 1000);
      virtual ~BaseCommand() = default;

      // Command execution
      virtual bool execute(modbus::ModbusDevice *device) = 0;
      virtual void process_response(const std::vector<uint8_t> &data) = 0;
      virtual void on_error(uint8_t function_code, uint8_t exception_code);

      // State management
      CommandState get_state() const { return state_; }
      void set_state(CommandState state) { state_ = state; }
      bool is_finished() const { return state_ == CommandState::COMPLETED || state_ == CommandState::FAILED || state_ == CommandState::TIMEOUT; }

      // Timeout handling
      bool check_timeout();
      void reset_timeout();
      uint32_t get_timeout_ms() const { return timeout_ms_; }

      // Callbacks
      void set_completion_callback(CompletionCallback callback) { completion_callback_ = callback; }
      void set_data_callback(DataCallback callback) { data_callback_ = callback; }

      // Getters
      uint16_t get_register_address() const { return register_address_; }
      virtual ModbusFunction get_function_code() const = 0;
      virtual const char *get_command_name() const = 0;

    protected:
      void trigger_completion(bool success);
      void trigger_data_callback(const std::vector<uint8_t> &data);

    private:
      uint16_t register_address_;
      CommandState state_;
      uint32_t timeout_ms_;
      uint32_t start_time_;
      CompletionCallback completion_callback_;
      DataCallback data_callback_;
    };

  } // namespace servo42d_rs485
} // namespace esphome