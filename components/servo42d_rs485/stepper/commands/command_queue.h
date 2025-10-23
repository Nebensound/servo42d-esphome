#pragma once

#include "base_command.h"
#include <deque>
#include <memory>

namespace esphome
{
  namespace servo42d_rs485
  {
    namespace servo42d_rs485
    {

      // Forward declaration
      class ModbusDevice;

      class CommandQueue
      {
      public:
        CommandQueue();
        ~CommandQueue() = default;

        // Queue management
        void enqueue(std::unique_ptr<BaseCommand> command);
        void process_next();
        void clear();
        bool is_empty() const;
        size_t size() const;

        // Current command access
        BaseCommand *get_current_command();
        bool has_executing_command() const;

        // Response handling
        void handle_response(const std::vector<uint8_t> &data);
        void handle_error(uint8_t function_code, uint8_t exception_code);

        // Timeout checking
        void check_timeouts();

        // Device association
        void set_device(ModbusDevice *device) { device_ = device; }

      private:
        std::deque<std::unique_ptr<BaseCommand>> queue_;
      };

    } // namespace servo42d_rs485
  } // namespace esphome
  std::unique_ptr<BaseCommand> current_command_;
  ModbusDevice *device_;

  void complete_current_command(bool success);
  void execute_current_command();
};

} // namespace servo42d_rs485
} // namespace esphome