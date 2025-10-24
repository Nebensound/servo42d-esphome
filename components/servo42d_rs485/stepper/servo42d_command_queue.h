#pragma once

#include "servo42d_base_command.h"
#include <deque>
#include <memory>

namespace esphome
{
  namespace servo42d_rs485
  {

    class CommandQueue
    {
    public:
      CommandQueue();
      ~CommandQueue() = default;

      // Queue management
      void enqueue(std::unique_ptr<BaseCommand> command);
      // Enqueue with priority (placed at the front of the queue)
      void enqueue_front(std::unique_ptr<BaseCommand> command);
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
      void handle_timeout();

      // Execute commands with device
      void execute_next(esphome::modbus::ModbusDevice *device);

    private:
      std::deque<std::unique_ptr<BaseCommand>> queue_;
      std::unique_ptr<BaseCommand> current_command_{nullptr};
    };

  } // namespace servo42d_rs485
} // namespace esphome
