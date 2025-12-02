#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace esphome {
namespace servoxxd {

/**
 * @brief Transport error codes
 *
 * From spec (02d-layer4-transport.md):
 * Error codes for transport layer operations
 */
enum class TransportError : uint8_t {
  NONE = 0,             ///< No error, operation successful
  TIMEOUT = 1,          ///< Command timed out waiting for response
  CRC_ERROR = 2,        ///< CRC validation failed on response
  INVALID_RESPONSE = 3, ///< Response format was invalid
  BUS_ERROR = 4         ///< Bus communication error
};

/**
 * @brief Transport interface for protocol-agnostic command execution
 *
 * From spec (02d-layer4-transport.md):
 * - Abstract protocol details (Modbus-RTU, Serial FA/FB)
 * - Provide unified Command-based API for upper layers
 * - Manage framing, addressing, CRC calculation
 * - Handle request-response state machine (half-duplex)
 *
 * State Machine:
 * IDLE → (execute/read) → WAITING_RESPONSE → (response/timeout) → IDLE
 *
 * Contracts:
 * - execute_command() / read_command(): Non-blocking, return immediately
 * - Precondition: is_busy() == false
 * - Responses processed asynchronously in update()
 * - Callbacks invoked on success or error
 *
 * @see 02d-layer4-transport.md Component 2: ITransport Interface
 */
class ITransport {
 public:
  virtual ~ITransport() = default;

  /**
   * @brief Execute a write command (Modbus 0x06/0x10)
   *
   * Non-blocking. Callback will be invoked when operation completes.
   *
   * @param cmd Command code (0x00-0xFF) to execute
   * @param data Data payload for the command
   * @param on_success Callback invoked on successful completion
   * @param on_error Callback invoked on error with error code
   */
  virtual void execute_command(uint8_t cmd, const std::vector<uint8_t> &data,
                               std::function<void()> on_success,
                               std::function<void(TransportError)> on_error) = 0;

  /**
   * @brief Execute a read command (Modbus 0x04)
   *
   * Non-blocking. Callback will be invoked when operation completes.
   *
   * @param cmd Command code (0x00-0xFF) to execute
   * @param expected_bytes Expected number of bytes in response
   * @param on_success Callback invoked with response data on success
   * @param on_error Callback invoked on error with error code
   */
  virtual void read_command(uint8_t cmd, size_t expected_bytes,
                            std::function<void(const std::vector<uint8_t> &)> on_success,
                            std::function<void(TransportError)> on_error) = 0;

  /**
   * @brief Check if transport is currently busy
   *
   * @return true if a command is being executed (waiting for response)
   * @return false if transport is idle and ready for new commands
   */
  virtual bool is_busy() const = 0;

  /**
   * @brief Process transport state machine
   *
   * Called from loop() to:
   * - Check for timeouts
   * - Process incoming data
   * - Invoke callbacks when operations complete
   */
  virtual void update() = 0;
};

}  // namespace servoxxd
}  // namespace esphome
