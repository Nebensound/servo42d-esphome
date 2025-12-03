#pragma once

#include "esphome/core/component.h"
#include "esphome/components/modbus/modbus.h"
#include "servoxxd_transport.h"
#include "servoxxd_commands.h"
#include <vector>
#include <functional>
#include <queue>

namespace esphome
{
  namespace servoxxd
  {

    /**
     * @brief Modbus-RTU implementation of ITransport for ServoXxd communication
     *
     * Implements the ITransport interface using Modbus-RTU protocol over RS485.
     * Command enum values map directly to Modbus register addresses.
     *
     * Protocol details:
     * - Read: Function 0x04 (Read Input Registers)
     * - Write: Function 0x06 (Write Single Register) or 0x10 (Write Multiple Registers)
     * - CRC16-MODBUS for frame integrity
     * - Configurable timeout (default 1000ms)
     *
     * @see docs/specification/02d-layer4-transport.md
     */
    class ModbusTransport : public ITransport
    {
    public:
      /**
       * @brief Construct a new Modbus Transport object
       *
       * @param device ESPHome Modbus device instance
       * @param slave_address Modbus slave address (1-247)
       */
      ModbusTransport(modbus::ModbusDevice *device, uint8_t slave_address);

      // ITransport interface implementation
      Result execute_command(Command cmd, const std::vector<uint8_t> &data = {}) override;
      Result read_command(Command cmd) override;
      bool is_busy() const override;
      void update() override;
      void set_response_callback(std::function<void(Command, const std::vector<uint8_t> &)> cb) override;
      void set_error_callback(std::function<void(Command, ErrorCode)> cb) override;

      /**
       * @brief Set command timeout in milliseconds
       * @param timeout_ms Timeout value (default: 1000ms)
       */
      void set_timeout(uint32_t timeout_ms) { timeout_ms_ = timeout_ms; }

    private:
      enum class State
      {
        IDLE,          // No pending operation
        WAITING_WRITE, // Waiting for write command response
        WAITING_READ   // Waiting for read command response
      };

      modbus::ModbusDevice *device_;
      uint8_t slave_address_;
      State state_{State::IDLE};
      Command pending_command_;
      uint32_t timeout_ms_{1000};
      uint32_t timeout_start_ms_{0};

      std::function<void(Command, const std::vector<uint8_t> &)> response_callback_;
      std::function<void(Command, ErrorCode)> error_callback_;

      /**
       * @brief Map Command to Modbus register address
       */
      uint16_t command_to_register(Command cmd) const
      {
        return static_cast<uint16_t>(cmd);
      }

      /**
       * @brief Check for timeout and invoke error callback
       */
      bool check_timeout();

      /**
       * @brief Handle Modbus read response
       */
      void handle_read_response(const std::vector<uint8_t> &data);

      /**
       * @brief Handle Modbus write response
       */
      void handle_write_response();
    };

  } // namespace servoxxd
} // namespace esphome
