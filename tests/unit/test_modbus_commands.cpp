/**
 * @file test_modbus_commands.cpp
 * @brief Unit tests for Modbus command classes
 * 
 * Tests command state machine, timeout tracking, and response validation
 * for ReadCommand, WriteCommand, and MultiWriteCommand.
 */

#include <iostream>
#include <cassert>
#include <vector>
#include <cstring>
#include "servoxxd_modbus.h"

using namespace esphome::servoxxd;

// ============================================================================
// Mock ModbusDevice for testing
// ============================================================================

class MockModbusDevice : public esphome::modbus::ModbusDevice
{
public:
  struct SentCommand
  {
    uint8_t function_code;
    uint16_t address;
    uint16_t value_or_count;
    std::vector<uint8_t> raw_payload;
  };

  std::vector<SentCommand> sent_commands;

  void send(uint8_t function_code, uint16_t address, uint16_t value_or_count) override
  {
    SentCommand cmd;
    cmd.function_code = function_code;
    cmd.address = address;
    cmd.value_or_count = value_or_count;
    sent_commands.push_back(cmd);
  }

  void send_raw(const std::vector<uint8_t> &payload) override
  {
    SentCommand cmd;
    if (payload.size() > 0)
    {
      cmd.function_code = payload[0];
      if (payload.size() >= 3)
      {
        cmd.address = (payload[1] << 8) | payload[2];
      }
      if (payload.size() >= 5)
      {
        cmd.value_or_count = (payload[3] << 8) | payload[4];
      }
    }
    cmd.raw_payload = payload;
    sent_commands.push_back(cmd);
  }

  void clear()
  {
    sent_commands.clear();
  }

  // Unused ModbusDevice interface methods
  void on_modbus_data(const std::vector<uint8_t> &data) override {}
  void on_modbus_error(uint8_t function_code, uint8_t exception_code) override {}
};

// ============================================================================
// Helper functions
// ============================================================================

uint32_t mock_time_ms = 0;

void advance_time(uint32_t ms)
{
  mock_time_ms += ms;
}

void reset_time()
{
  mock_time_ms = 0;
}

// ============================================================================
// Test: ReadCommand basic functionality
// ============================================================================

void test_read_command_execute()
{
  std::cout << "Testing ReadCommand::execute()..." << std::endl;

  MockModbusDevice device;
  ReadCommand cmd(0x30, 2); // Read 2 registers from address 0x30

  assert(cmd.get_state() == CommandState::PENDING);
  assert(cmd.get_function_code() == 0x04);
  assert(cmd.get_register_address() == 0x30);
  assert(cmd.get_quantity() == 2);
  assert(std::string(cmd.get_command_name()) == "ReadInputRegisters");

  cmd.execute(&device);

  assert(device.sent_commands.size() == 1);
  assert(device.sent_commands[0].function_code == 0x04);
  assert(device.sent_commands[0].address == 0x30);
  assert(device.sent_commands[0].value_or_count == 2);

  std::cout << "  ✓ ReadCommand executes correctly" << std::endl;
}

void test_read_command_process_response()
{
  std::cout << "Testing ReadCommand::process_response()..." << std::endl;

  ReadCommand cmd(0x30, 2);

  // Valid response: byte_count=4, reg1=0x1234, reg2=0x5678
  std::vector<uint8_t> valid_response = {0x04, 0x12, 0x34, 0x56, 0x78};

  bool callback_invoked = false;
  bool callback_success = false;
  std::vector<uint8_t> callback_data;

  cmd.set_completion_callback([&](bool success)
                               {
    callback_invoked = true;
    callback_success = success; });

  cmd.set_data_callback([&](const std::vector<uint8_t> &data)
                        { callback_data = data; });

  bool result = cmd.process_response(valid_response);

  assert(result == true);
  assert(cmd.get_state() == CommandState::COMPLETED);
  assert(callback_invoked == true);
  assert(callback_success == true);
  assert(callback_data.size() == 4);
  assert(callback_data[0] == 0x12);
  assert(callback_data[1] == 0x34);
  assert(callback_data[2] == 0x56);
  assert(callback_data[3] == 0x78);

  std::cout << "  ✓ ReadCommand processes valid response correctly" << std::endl;
}

void test_read_command_invalid_byte_count()
{
  std::cout << "Testing ReadCommand with invalid byte count..." << std::endl;

  ReadCommand cmd(0x30, 2); // Expect 4 bytes (2 registers * 2)

  // Invalid response: byte_count=2 (should be 4)
  std::vector<uint8_t> invalid_response = {0x02, 0x12, 0x34};

  bool callback_invoked = false;
  bool callback_success = false;

  cmd.set_completion_callback([&](bool success)
                               {
    callback_invoked = true;
    callback_success = success; });

  bool result = cmd.process_response(invalid_response);

  assert(result == false);
  assert(cmd.get_state() == CommandState::FAILED);
  assert(callback_invoked == true);
  assert(callback_success == false);

  std::cout << "  ✓ ReadCommand rejects invalid byte count" << std::endl;
}

void test_read_command_empty_response()
{
  std::cout << "Testing ReadCommand with empty response..." << std::endl;

  ReadCommand cmd(0x30, 2);
  std::vector<uint8_t> empty_response;

  bool callback_invoked = false;
  bool callback_success = false;

  cmd.set_completion_callback([&](bool success)
                               {
    callback_invoked = true;
    callback_success = success; });

  bool result = cmd.process_response(empty_response);

  assert(result == false);
  assert(cmd.get_state() == CommandState::FAILED);
  assert(callback_invoked == true);
  assert(callback_success == false);

  std::cout << "  ✓ ReadCommand handles empty response" << std::endl;
}

// ============================================================================
// Test: WriteCommand basic functionality
// ============================================================================

void test_write_command_execute()
{
  std::cout << "Testing WriteCommand::execute()..." << std::endl;

  MockModbusDevice device;
  WriteCommand cmd(0xFD, 0x1234); // Write 0x1234 to register 0xFD

  assert(cmd.get_state() == CommandState::PENDING);
  assert(cmd.get_function_code() == 0x06);
  assert(cmd.get_register_address() == 0xFD);
  assert(cmd.get_value() == 0x1234);
  assert(std::string(cmd.get_command_name()) == "WriteSingleRegister");

  cmd.execute(&device);

  assert(device.sent_commands.size() == 1);
  assert(device.sent_commands[0].function_code == 0x06);
  assert(device.sent_commands[0].address == 0xFD);
  assert(device.sent_commands[0].value_or_count == 0x1234);

  std::cout << "  ✓ WriteCommand executes correctly" << std::endl;
}

void test_write_command_process_response()
{
  std::cout << "Testing WriteCommand::process_response()..." << std::endl;

  WriteCommand cmd(0xFD, 0x1234);

  // Valid response: echo of address + value
  std::vector<uint8_t> valid_response = {0x00, 0xFD, 0x12, 0x34};

  bool callback_invoked = false;
  bool callback_success = false;

  cmd.set_completion_callback([&](bool success)
                               {
    callback_invoked = true;
    callback_success = success; });

  bool result = cmd.process_response(valid_response);

  assert(result == true);
  assert(cmd.get_state() == CommandState::COMPLETED);
  assert(callback_invoked == true);
  assert(callback_success == true);

  std::cout << "  ✓ WriteCommand processes valid response correctly" << std::endl;
}

void test_write_command_address_mismatch()
{
  std::cout << "Testing WriteCommand with address mismatch..." << std::endl;

  WriteCommand cmd(0xFD, 0x1234);

  // Invalid response: wrong address
  std::vector<uint8_t> invalid_response = {0x00, 0xFC, 0x12, 0x34}; // 0xFC instead of 0xFD

  bool callback_invoked = false;
  bool callback_success = false;

  cmd.set_completion_callback([&](bool success)
                               {
    callback_invoked = true;
    callback_success = success; });

  bool result = cmd.process_response(invalid_response);

  assert(result == false);
  assert(cmd.get_state() == CommandState::FAILED);
  assert(callback_invoked == true);
  assert(callback_success == false);

  std::cout << "  ✓ WriteCommand detects address mismatch" << std::endl;
}

void test_write_command_value_mismatch()
{
  std::cout << "Testing WriteCommand with value mismatch..." << std::endl;

  WriteCommand cmd(0xFD, 0x1234);

  // Invalid response: wrong value
  std::vector<uint8_t> invalid_response = {0x00, 0xFD, 0x12, 0x35}; // 0x1235 instead of 0x1234

  bool callback_invoked = false;
  bool callback_success = false;

  cmd.set_completion_callback([&](bool success)
                               {
    callback_invoked = true;
    callback_success = success; });

  bool result = cmd.process_response(invalid_response);

  assert(result == false);
  assert(cmd.get_state() == CommandState::FAILED);
  assert(callback_invoked == true);
  assert(callback_success == false);

  std::cout << "  ✓ WriteCommand detects value mismatch" << std::endl;
}

// ============================================================================
// Test: MultiWriteCommand basic functionality
// ============================================================================

void test_multiwrite_command_execute()
{
  std::cout << "Testing MultiWriteCommand::execute()..." << std::endl;

  MockModbusDevice device;
  std::vector<uint16_t> values = {0x1234, 0x5678, 0x9ABC};
  MultiWriteCommand cmd(0x00, values);

  assert(cmd.get_state() == CommandState::PENDING);
  assert(cmd.get_function_code() == 0x10);
  assert(cmd.get_register_address() == 0x00);
  assert(cmd.get_quantity() == 3);
  assert(std::string(cmd.get_command_name()) == "WriteMultipleRegisters");

  cmd.execute(&device);

  assert(device.sent_commands.size() == 1);
  assert(device.sent_commands[0].function_code == 0x10);
  assert(device.sent_commands[0].address == 0x00);
  assert(device.sent_commands[0].value_or_count == 3);

  // Verify raw payload contains register values
  const auto &payload = device.sent_commands[0].raw_payload;
  assert(payload.size() >= 11); // header(6) + byte_count(1) + 3 registers(6) = 13 bytes (but we check first 11)

  // Check byte_count
  assert(payload[5] == 6); // 3 registers * 2 bytes

  // Check register values in payload
  assert(payload[6] == 0x12 && payload[7] == 0x34); // First register
  assert(payload[8] == 0x56 && payload[9] == 0x78); // Second register
  assert(payload[10] == 0x9A && payload[11] == 0xBC); // Third register

  std::cout << "  ✓ MultiWriteCommand executes correctly" << std::endl;
}

void test_multiwrite_command_process_response()
{
  std::cout << "Testing MultiWriteCommand::process_response()..." << std::endl;

  std::vector<uint16_t> values = {0x1234, 0x5678, 0x9ABC};
  MultiWriteCommand cmd(0x00, values);

  // Valid response: echo of address + quantity
  std::vector<uint8_t> valid_response = {0x00, 0x00, 0x00, 0x03}; // Address 0x00, quantity 3

  bool callback_invoked = false;
  bool callback_success = false;

  cmd.set_completion_callback([&](bool success)
                               {
    callback_invoked = true;
    callback_success = success; });

  bool result = cmd.process_response(valid_response);

  assert(result == true);
  assert(cmd.get_state() == CommandState::COMPLETED);
  assert(callback_invoked == true);
  assert(callback_success == true);

  std::cout << "  ✓ MultiWriteCommand processes valid response correctly" << std::endl;
}

void test_multiwrite_command_quantity_mismatch()
{
  std::cout << "Testing MultiWriteCommand with quantity mismatch..." << std::endl;

  std::vector<uint16_t> values = {0x1234, 0x5678, 0x9ABC};
  MultiWriteCommand cmd(0x00, values); // 3 registers

  // Invalid response: wrong quantity
  std::vector<uint8_t> invalid_response = {0x00, 0x00, 0x00, 0x02}; // Quantity 2 instead of 3

  bool callback_invoked = false;
  bool callback_success = false;

  cmd.set_completion_callback([&](bool success)
                               {
    callback_invoked = true;
    callback_success = success; });

  bool result = cmd.process_response(invalid_response);

  assert(result == false);
  assert(cmd.get_state() == CommandState::FAILED);
  assert(callback_invoked == true);
  assert(callback_success == false);

  std::cout << "  ✓ MultiWriteCommand detects quantity mismatch" << std::endl;
}

// ============================================================================
// Test: Timeout tracking
// ============================================================================

void test_timeout_tracking()
{
  std::cout << "Testing timeout tracking..." << std::endl;

  reset_time();

  ReadCommand cmd(0x30, 2, 1000); // 1 second timeout

  assert(cmd.get_state() == CommandState::PENDING);
  assert(cmd.is_timeout(mock_time_ms) == false);

  cmd.start_timeout(mock_time_ms);
  assert(cmd.get_state() == CommandState::EXECUTING);
  assert(cmd.is_timeout(mock_time_ms) == false);

  advance_time(500); // 500ms elapsed
  assert(cmd.is_timeout(mock_time_ms) == false);
  assert(cmd.get_elapsed_time(mock_time_ms) == 500);

  advance_time(600); // 1100ms total elapsed
  assert(cmd.is_timeout(mock_time_ms) == true);
  assert(cmd.get_elapsed_time(mock_time_ms) == 1100);

  std::cout << "  ✓ Timeout tracking works correctly" << std::endl;
}

void test_timeout_only_in_executing_state()
{
  std::cout << "Testing timeout only applies in EXECUTING state..." << std::endl;

  reset_time();

  ReadCommand cmd(0x30, 2, 100); // 100ms timeout

  // PENDING state - no timeout
  assert(cmd.get_state() == CommandState::PENDING);
  advance_time(200);
  assert(cmd.is_timeout(mock_time_ms) == false);

  // EXECUTING state - timeout applies
  cmd.start_timeout(mock_time_ms);
  assert(cmd.get_state() == CommandState::EXECUTING);
  advance_time(50);
  assert(cmd.is_timeout(mock_time_ms) == false);
  advance_time(100); // 150ms total in EXECUTING
  assert(cmd.is_timeout(mock_time_ms) == true);

  // COMPLETED state - no timeout
  cmd.set_state(CommandState::COMPLETED);
  assert(cmd.is_timeout(mock_time_ms) == false);

  std::cout << "  ✓ Timeout only applies in EXECUTING state" << std::endl;
}

// ============================================================================
// Test: Null device handling
// ============================================================================

void test_null_device_handling()
{
  std::cout << "Testing null device handling..." << std::endl;

  ReadCommand read_cmd(0x30, 2);
  WriteCommand write_cmd(0xFD, 0x1234);
  std::vector<uint16_t> values = {0x1234};
  MultiWriteCommand multi_cmd(0x00, values);

  bool read_callback_invoked = false;
  bool write_callback_invoked = false;
  bool multi_callback_invoked = false;

  read_cmd.set_completion_callback([&](bool success)
                                   {
    read_callback_invoked = true;
    assert(success == false); });

  write_cmd.set_completion_callback([&](bool success)
                                    {
    write_callback_invoked = true;
    assert(success == false); });

  multi_cmd.set_completion_callback([&](bool success)
                                    {
    multi_callback_invoked = true;
    assert(success == false); });

  read_cmd.execute(nullptr);
  write_cmd.execute(nullptr);
  multi_cmd.execute(nullptr);

  assert(read_cmd.get_state() == CommandState::FAILED);
  assert(write_cmd.get_state() == CommandState::FAILED);
  assert(multi_cmd.get_state() == CommandState::FAILED);

  assert(read_callback_invoked == true);
  assert(write_callback_invoked == true);
  assert(multi_callback_invoked == true);

  std::cout << "  ✓ Null device handling works correctly" << std::endl;
}

// ============================================================================
// Main test runner
// ============================================================================

int main()
{
  std::cout << std::endl;
  std::cout << "=== Modbus Command Classes Unit Tests ===" << std::endl;
  std::cout << std::endl;

  // ReadCommand tests
  test_read_command_execute();
  test_read_command_process_response();
  test_read_command_invalid_byte_count();
  test_read_command_empty_response();

  // WriteCommand tests
  test_write_command_execute();
  test_write_command_process_response();
  test_write_command_address_mismatch();
  test_write_command_value_mismatch();

  // MultiWriteCommand tests
  test_multiwrite_command_execute();
  test_multiwrite_command_process_response();
  test_multiwrite_command_quantity_mismatch();

  // Timeout tests
  test_timeout_tracking();
  test_timeout_only_in_executing_state();

  // Edge case tests
  test_null_device_handling();

  std::cout << std::endl;
  std::cout << "✅ All Modbus Command tests passed!" << std::endl;
  std::cout << std::endl;

  return 0;
}
