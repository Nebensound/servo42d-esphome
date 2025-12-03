/**
 * @file test_command_queue.cpp
 * @brief Unit tests for CommandQueue class
 *
 * Tests verify:
 * - Basic enqueue and execution
 * - FIFO order preservation
 * - Error handling
 * - Clear operation (emergency stop)
 */

#include <cstdint>
#include <cassert>
#include <iostream>
#include <vector>

// Define millis() mock BEFORE including any ESPHome headers
static uint32_t test_millis = 0;
uint32_t millis() { return test_millis; }

#include "servoxxd_command_queue.h"
#include "servoxxd_transport.h"
#include "servoxxd_commands.h"

using namespace esphome::servoxxd;

// Mock Transport for testing
class MockTransport : public ITransport
{
public:
  MockTransport() : execute_count_(0), read_count_(0), busy_(false) {}

  // Track calls
  int execute_count_;
  int read_count_;
  std::vector<Command> executed_commands_;
  std::vector<Command> read_commands_;
  bool busy_;

  // Callbacks
  std::function<void(Command, const std::vector<uint8_t> &)> response_callback_;
  std::function<void(Command, ErrorCode)> error_callback_;

  // Override ITransport interface
  Result execute_command(Command cmd, const std::vector<uint8_t> &data = {}) override
  {
    (void)data; // Unused in mock
    execute_count_++;
    executed_commands_.push_back(cmd);
    busy_ = true;
    Result r;
    r.success = true;
    r.error_code = ErrorCode::OK;
    return r;
  }

  Result read_command(Command cmd) override
  {
    read_count_++;
    read_commands_.push_back(cmd);
    busy_ = true;
    Result r;
    r.success = true;
    r.error_code = ErrorCode::OK;
    return r;
  }

  bool is_busy() const override { return busy_; }

  void update() override {}

  void set_response_callback(std::function<void(Command, const std::vector<uint8_t> &)> cb) override
  {
    response_callback_ = cb;
  }

  void set_error_callback(std::function<void(Command, ErrorCode)> cb) override
  {
    error_callback_ = cb;
  }

  // Helper methods for tests
  void simulate_response(Command cmd, const std::vector<uint8_t> &data)
  {
    busy_ = false;
    if (response_callback_)
    {
      response_callback_(cmd, data);
    }
  }

  void simulate_error(Command cmd, ErrorCode error)
  {
    busy_ = false;
    if (error_callback_)
    {
      error_callback_(cmd, error);
    }
  }
};

// Test helpers
struct TestResult
{
  bool callback_called = false;
  bool success = false;
  std::vector<uint8_t> data;
};

// ============================================================================
// TEST 1: Basic Enqueue and Execution
// ============================================================================
void test_basic_enqueue()
{
  std::cout << "TEST 1: Basic enqueue and execution..." << std::endl;

  MockTransport transport;
  CommandQueue queue(&transport);

  // Initially empty
  assert(queue.is_empty());
  assert(queue.size() == 0);

  TestResult result;
  queue.enqueue_read(Command::READ_ENCODER_CARRY, [&](bool success, const std::vector<uint8_t> &data)
                     {
    result.callback_called = true;
    result.success = success;
    result.data = data; });

  // After enqueue: size should be 1
  assert(queue.size() == 1);
  assert(!queue.is_empty());

  // Trigger execution
  queue.update();

  // Should have called read_command
  assert(transport.read_count_ == 1);
  assert(transport.read_commands_[0] == Command::READ_ENCODER_CARRY);

  // Simulate response via CommandQueue (not transport callback, since that's not set up yet)
  std::vector<uint8_t> response_data = {0xAA, 0xBB, 0xCC};
  queue.on_response(Command::READ_ENCODER_CARRY, response_data);

  // Callback should be invoked
  assert(result.callback_called);
  assert(result.success);
  assert(result.data == response_data);
  assert(queue.is_empty());

  std::cout << "  ✓ Basic enqueue works" << std::endl;
  std::cout << "  ✓ Command execution triggered" << std::endl;
  std::cout << "  ✓ Response handled correctly" << std::endl;
}

// ============================================================================
// TEST 2: FIFO Order
// ============================================================================
void test_fifo_order()
{
  std::cout << "\nTEST 2: FIFO order preservation..." << std::endl;

  MockTransport transport;
  CommandQueue queue(&transport);

  std::vector<int> callback_order;

  // Enqueue 3 commands
  queue.enqueue_read(Command::READ_ENCODER_CARRY, [&](bool, const std::vector<uint8_t> &)
                     { callback_order.push_back(1); });
  queue.enqueue_read(Command::READ_CURRENT_SPEED, [&](bool, const std::vector<uint8_t> &)
                     { callback_order.push_back(2); });
  queue.enqueue_read(Command::READ_MOTOR_STATUS, [&](bool, const std::vector<uint8_t> &)
                     { callback_order.push_back(3); });

  assert(queue.size() == 3);

  // Execute and respond to first command
  queue.update();
  assert(transport.read_commands_[0] == Command::READ_ENCODER_CARRY);
  queue.on_response(Command::READ_ENCODER_CARRY, {});
  assert(callback_order.size() == 1 && callback_order[0] == 1);
  assert(queue.size() == 2);

  // Execute and respond to second command
  queue.update();
  assert(transport.read_commands_[1] == Command::READ_CURRENT_SPEED);
  queue.on_response(Command::READ_CURRENT_SPEED, {});
  assert(callback_order.size() == 2 && callback_order[1] == 2);
  assert(queue.size() == 1);

  // Execute and respond to third command
  queue.update();
  assert(transport.read_commands_[2] == Command::READ_MOTOR_STATUS);
  queue.on_response(Command::READ_MOTOR_STATUS, {});
  assert(callback_order.size() == 3 && callback_order[2] == 3);
  assert(queue.is_empty());

  std::cout << "  ✓ FIFO order preserved" << std::endl;
  std::cout << "  ✓ Queue size updates correctly" << std::endl;
}

// ============================================================================
// TEST 3: Error Handling
// ============================================================================
void test_error_handling()
{
  std::cout << "\nTEST 3: Error handling..." << std::endl;

  MockTransport transport;
  CommandQueue queue(&transport);

  TestResult result;
  queue.enqueue_read(Command::READ_ENCODER_CARRY, [&](bool success, const std::vector<uint8_t> &data)
                     {
    result.callback_called = true;
    result.success = success;
    result.data = data; });

  queue.update();

  // Simulate error via CommandQueue
  queue.on_error(Command::READ_ENCODER_CARRY, ErrorCode::TIMEOUT);

  // Callback should be invoked with success=false
  assert(result.callback_called);
  assert(!result.success);
  assert(result.data.empty());
  assert(queue.is_empty());

  std::cout << "  ✓ Error handled correctly" << std::endl;
  std::cout << "  ✓ Callback invoked with success=false" << std::endl;
}

// ============================================================================
// TEST 4: Queue Clear (Emergency Stop)
// ============================================================================
void test_clear()
{
  std::cout << "\nTEST 4: Queue clear (emergency stop)..." << std::endl;

  MockTransport transport;
  CommandQueue queue(&transport);

  int callbacks_invoked = 0;

  // Enqueue 3 commands
  queue.enqueue_read(Command::READ_ENCODER_CARRY, [&](bool success, const std::vector<uint8_t> &)
                     {
                       callbacks_invoked++;
                       assert(!success); // Should be called with failure
                     });
  queue.enqueue_read(Command::READ_CURRENT_SPEED, [&](bool success, const std::vector<uint8_t> &)
                     {
    callbacks_invoked++;
    assert(!success); });
  queue.enqueue_read(Command::READ_MOTOR_STATUS, [&](bool success, const std::vector<uint8_t> &)
                     {
    callbacks_invoked++;
    assert(!success); });

  // First command is already being executed (enqueue_read calls execute_next)
  // So we have: 1 executing + 2 pending = 3 total
  assert(queue.size() == 3);

  // Clear queue - should clear only the 2 pending commands
  queue.clear();

  std::cout << "  callbacks_invoked = " << callbacks_invoked << std::endl;

  // Only 2 callbacks should be invoked (pending commands)
  // The executing command is NOT cleared
  assert(callbacks_invoked == 2);
  assert(queue.size() == 1); // 1 executing command remains

  std::cout << "  ✓ Queue cleared (pending commands only)" << std::endl;
  std::cout << "  ✓ Executing command preserved" << std::endl;
  std::cout << "  ✓ Pending callbacks invoked with failure" << std::endl;
}

// ============================================================================
// TEST 5: Deduplication of Read Commands
// ============================================================================
void test_deduplication()
{
  std::cout << "\nTEST 5: Deduplication of read commands..." << std::endl;

  MockTransport transport;
  CommandQueue queue(&transport);

  int callback1_count = 0;
  int callback2_count = 0;
  int callback3_count = 0;
  std::vector<uint8_t> received_data1;
  std::vector<uint8_t> received_data2;
  std::vector<uint8_t> received_data3;

  // Enqueue a different command first (will become EXECUTING)
  queue.enqueue_read(Command::READ_MOTOR_STATUS, [](bool, const std::vector<uint8_t> &) {});

  assert(queue.size() == 1);

  // Now enqueue two identical commands (both will be PENDING)
  queue.enqueue_read(Command::READ_ENCODER_CARRY, [&](bool, const std::vector<uint8_t> &data)
                     {
    callback1_count++;
    received_data1 = data; });

  assert(queue.size() == 2);

  // Enqueue duplicate - should deduplicate with the PENDING one
  queue.enqueue_read(Command::READ_ENCODER_CARRY, [&](bool, const std::vector<uint8_t> &data)
                     {
    callback2_count++;
    received_data2 = data; });

  // Should still be size 2 (first command + deduplicated second/third)
  assert(queue.size() == 2);

  // Add a third duplicate
  queue.enqueue_read(Command::READ_ENCODER_CARRY, [&](bool, const std::vector<uint8_t> &data)
                     {
    callback3_count++;
    received_data3 = data; });

  assert(queue.size() == 2); // Still 2

  // Complete first command
  queue.update();
  queue.on_response(Command::READ_MOTOR_STATUS, {});

  // Now execute the merged command
  queue.update();
  std::vector<uint8_t> response_data = {0x12, 0x34, 0x56, 0x78};
  queue.on_response(Command::READ_ENCODER_CARRY, response_data);

  // All three callbacks should be invoked with same data
  assert(callback1_count == 1);
  assert(callback2_count == 1);
  assert(callback3_count == 1);
  assert(received_data1 == response_data);
  assert(received_data2 == response_data);
  assert(received_data3 == response_data);
  assert(queue.is_empty());

  std::cout << "  ✓ Duplicate read commands merged (3 callbacks)" << std::endl;
  std::cout << "  ✓ All callbacks invoked with same data" << std::endl;
}

// ============================================================================
// TEST 6: No Deduplication for Different Commands
// ============================================================================
void test_no_deduplication_different()
{
  std::cout << "\nTEST 6: No deduplication for different commands..." << std::endl;

  MockTransport transport;
  CommandQueue queue(&transport);

  // Enqueue two different read commands
  queue.enqueue_read(Command::READ_ENCODER_CARRY, [](bool, const std::vector<uint8_t> &) {});
  queue.enqueue_read(Command::READ_CURRENT_SPEED, [](bool, const std::vector<uint8_t> &) {});

  assert(queue.size() == 2);

  std::cout << "  ✓ Different commands NOT deduplicated" << std::endl;
}

// ============================================================================
// TEST 7: No Deduplication for Write Commands
// ============================================================================
void test_no_deduplication_writes()
{
  std::cout << "\nTEST 7: No deduplication for write commands..." << std::endl;

  MockTransport transport;
  CommandQueue queue(&transport);

  std::vector<uint8_t> data1 = {0x12, 0x34};
  std::vector<uint8_t> data2 = {0x12, 0x34}; // Same data

  // Enqueue two identical write commands
  queue.enqueue(Command::SET_ZERO, data1, [](bool, const std::vector<uint8_t> &) {});
  queue.enqueue(Command::SET_ZERO, data2, [](bool, const std::vector<uint8_t> &) {});

  // Should NOT deduplicate (writes must execute in order)
  assert(queue.size() == 2);

  std::cout << "  ✓ Write commands NOT deduplicated" << std::endl;
}

// ============================================================================
// TEST 8: Priority Queue (Front Insertion)
// ============================================================================
void test_priority_queue()
{
  std::cout << "\nTEST 8: Priority queue handling..." << std::endl;

  MockTransport transport;
  CommandQueue queue(&transport);

  std::vector<int> callback_order;

  // Enqueue normal commands
  queue.enqueue_read(Command::READ_ENCODER_CARRY, [&](bool, const std::vector<uint8_t> &)
                     { callback_order.push_back(1); });

  queue.enqueue_read(Command::READ_CURRENT_SPEED, [&](bool, const std::vector<uint8_t> &)
                     { callback_order.push_back(2); });

  // Enqueue priority command (use READ_MOTOR_STATUS as priority, not EMERGENCY_STOP)
  queue.enqueue_read(Command::READ_MOTOR_STATUS, [&](bool, const std::vector<uint8_t> &)
                     { callback_order.push_back(99); });

  // Manually promote it to priority by re-enqueuing with priority flag
  // (enqueue_read doesn't have priority parameter, so we use the base enqueue)
  std::vector<uint8_t> empty_data;

  // Actually, let's test differently: enqueue a write command with priority
  queue.enqueue(Command::RELEASE_PROTECTION, empty_data, [&](bool, const std::vector<uint8_t> &)
                { callback_order.push_back(99); },
                true); // priority=true

  // Should be: 1 EXECUTING + 3 PENDING = 4 total
  // (READ_ENCODER_CARRY executing, READ_CURRENT_SPEED + READ_MOTOR_STATUS + RELEASE_PROTECTION pending)
  assert(queue.size() == 4);

  // First command starts executing
  queue.update();
  queue.on_response(Command::READ_ENCODER_CARRY, {});
  assert(callback_order[0] == 1);

  // Priority command (RELEASE_PROTECTION) should execute next (jumped to front)
  queue.update();
  queue.on_response(Command::RELEASE_PROTECTION, {});
  assert(callback_order[1] == 99);

  // Then READ_CURRENT_SPEED
  queue.update();
  queue.on_response(Command::READ_CURRENT_SPEED, {});
  assert(callback_order[2] == 2);

  // Finally READ_MOTOR_STATUS
  queue.update();
  queue.on_response(Command::READ_MOTOR_STATUS, {});
  assert(callback_order[3] == 99); // Same callback ID

  std::cout << "  ✓ Priority commands jump to front" << std::endl;
  std::cout << "  ✓ Executing command not interrupted" << std::endl;
}

// ============================================================================
// TEST 9: Timeout Detection
// ============================================================================
void test_timeout_detection()
{
  std::cout << "\nTEST 9: Timeout detection..." << std::endl;

  test_millis = 0; // Reset time

  MockTransport transport;
  CommandQueue queue(&transport, 500); // 500ms timeout

  bool timeout_detected = false;
  std::vector<uint8_t> timeout_data;

  queue.enqueue_read(Command::READ_MOTOR_STATUS, [&](bool success, const std::vector<uint8_t> &data)
                     {
    timeout_detected = !success;
    timeout_data = data; });

  // Command starts executing
  queue.update();
  assert(transport.read_count_ == 1);
  assert(queue.size() == 1);

  // Advance time but not past timeout
  test_millis = 400;
  queue.update();
  assert(!timeout_detected); // Should not timeout yet
  assert(queue.size() == 1);

  // Advance time past timeout
  test_millis = 600; // 600 > 500 timeout
  queue.update();

  // Command should timeout
  assert(timeout_detected);
  assert(timeout_data.empty());
  assert(queue.is_empty());

  std::cout << "  ✓ Timeout detected at correct time" << std::endl;
  std::cout << "  ✓ Callback invoked with success=false" << std::endl;

  test_millis = 0; // Reset for other tests
}

// ============================================================================
// TEST 10: Timeout Allows Next Command
// ============================================================================
void test_timeout_recovery()
{
  std::cout << "\nTEST 10: Timeout recovery and queue continuation..." << std::endl;

  test_millis = 0;

  MockTransport transport;
  CommandQueue queue(&transport, 500);

  bool first_timeout = false;
  bool second_success = false;

  // Enqueue two commands
  queue.enqueue_read(Command::READ_MOTOR_STATUS, [&](bool success, const std::vector<uint8_t> &)
                     { first_timeout = !success; });

  queue.enqueue_read(Command::READ_CURRENT_SPEED, [&](bool success, const std::vector<uint8_t> &)
                     { second_success = success; });

  assert(queue.size() == 2);

  // First command executes
  queue.update();
  assert(transport.read_count_ == 1);

  // Advance past timeout for first command
  test_millis = 600;
  queue.update();

  // First command should timeout
  assert(first_timeout);
  assert(queue.size() == 1); // Second command still in queue

  // Second command should execute automatically
  assert(transport.read_count_ == 2); // Second command sent

  // Respond to second command
  queue.on_response(Command::READ_CURRENT_SPEED, {0xAA, 0xBB});

  assert(second_success);
  assert(queue.is_empty());

  std::cout << "  ✓ Timeout clears execution guard" << std::endl;
  std::cout << "  ✓ Next command executes automatically" << std::endl;

  test_millis = 0;
}

// ============================================================================
// TEST 11: Multiple Callbacks on Same Command (Deduplication)
// ============================================================================
void test_multiple_callbacks()
{
  std::cout << "\nTEST 11: Multiple callbacks on deduplicated command..." << std::endl;

  MockTransport transport;
  CommandQueue queue(&transport);

  int callback_count = 0;
  std::vector<std::vector<uint8_t>> received_data;

  // Enqueue a blocker command first (will be EXECUTING)
  queue.enqueue_read(Command::READ_MOTOR_STATUS, [](bool, const std::vector<uint8_t> &) {});

  // Now enqueue same command 3 times (all will be PENDING and deduplicated)
  for (int i = 0; i < 3; i++)
  {
    queue.enqueue_read(Command::READ_ENCODER_CARRY, [&](bool, const std::vector<uint8_t> &data)
                       {
      callback_count++;
      received_data.push_back(data); });
  }

  // Should be: 1 EXECUTING (READ_MOTOR_STATUS) + 1 PENDING (deduplicated READ_ENCODER_CARRY) = 2
  assert(queue.size() == 2);

  // Complete blocker command
  queue.update();
  queue.on_response(Command::READ_MOTOR_STATUS, {});

  // Execute deduplicated command
  queue.update();
  std::vector<uint8_t> response = {0xFF, 0xFF};
  queue.on_response(Command::READ_ENCODER_CARRY, response);

  // All 3 callbacks should be invoked
  assert(callback_count == 3);
  assert(received_data.size() == 3);
  for (const auto &data : received_data)
  {
    assert(data == response);
  }

  std::cout << "  ✓ All callbacks invoked" << std::endl;
  std::cout << "  ✓ All receive same data" << std::endl;
}

// ============================================================================
// TEST 12: Update on Empty Queue (No Crash)
// ============================================================================
void test_update_empty_queue()
{
  std::cout << "\nTEST 12: Update on empty queue..." << std::endl;

  MockTransport transport;
  CommandQueue queue(&transport);

  assert(queue.is_empty());

  // Should not crash or cause issues
  queue.update();
  queue.update();
  queue.update();

  assert(queue.is_empty());
  assert(transport.read_count_ == 0);

  std::cout << "  ✓ No crash on empty update" << std::endl;
}

// ============================================================================
// TEST 13: Error After Timeout (Ignore Late Response)
// ============================================================================
void test_late_response_ignored()
{
  std::cout << "\nTEST 13: Late response ignored after timeout..." << std::endl;

  test_millis = 0;

  MockTransport transport;
  CommandQueue queue(&transport, 500);

  bool callback_called = false;
  int callback_count = 0;

  queue.enqueue_read(Command::READ_MOTOR_STATUS, [&](bool success, const std::vector<uint8_t> &)
                     {
                       callback_count++;
                       callback_called = !success; // Timeout = !success
                     });

  queue.update();

  // Trigger timeout
  test_millis = 600;
  queue.update();

  assert(callback_called);
  assert(callback_count == 1);
  assert(queue.is_empty());

  // Late response arrives (should be ignored)
  queue.on_response(Command::READ_MOTOR_STATUS, {0x11, 0x22});

  // Callback should NOT be called again
  assert(callback_count == 1); // Still 1, not 2

  std::cout << "  ✓ Late response ignored" << std::endl;
  std::cout << "  ✓ Callback not invoked twice" << std::endl;

  test_millis = 0;
}

// ============================================================================
// TEST 14: Mixed Read and Write Commands
// ============================================================================
void test_mixed_read_write()
{
  std::cout << "\nTEST 14: Mixed read and write commands..." << std::endl;

  MockTransport transport;
  CommandQueue queue(&transport);

  std::vector<int> order;

  // Mix of reads and writes
  queue.enqueue_read(Command::READ_ENCODER_CARRY, [&](bool, const std::vector<uint8_t> &)
                     { order.push_back(1); });

  std::vector<uint8_t> data = {0x00, 0x00};
  queue.enqueue(Command::SET_ZERO, data, [&](bool, const std::vector<uint8_t> &)
                { order.push_back(2); });

  queue.enqueue_read(Command::READ_CURRENT_SPEED, [&](bool, const std::vector<uint8_t> &)
                     { order.push_back(3); });

  assert(queue.size() == 3);

  // Execute in FIFO order
  queue.update();
  queue.on_response(Command::READ_ENCODER_CARRY, {});
  assert(order[0] == 1);

  queue.update();
  queue.on_response(Command::SET_ZERO, {});
  assert(order[1] == 2);

  queue.update();
  queue.on_response(Command::READ_CURRENT_SPEED, {});
  assert(order[2] == 3);

  std::cout << "  ✓ Mixed commands execute in order" << std::endl;
}

// ============================================================================
// Main
// ============================================================================
int main()
{
  std::cout << "========================================" << std::endl;
  std::cout << "CommandQueue Unit Tests" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << std::endl;

  try
  {
    test_basic_enqueue();
    test_fifo_order();
    test_error_handling();
    test_clear();
    test_deduplication();
    test_no_deduplication_different();
    test_no_deduplication_writes();
    test_priority_queue();
    test_timeout_detection();
    test_timeout_recovery();
    test_multiple_callbacks();
    test_update_empty_queue();
    test_late_response_ignored();
    test_mixed_read_write();

    std::cout << "\n========================================" << std::endl;
    std::cout << "✅ All CommandQueue Tests Passed (14/14)!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "\n❌ Test failed with exception: " << e.what() << std::endl;
    return 1;
  }
  catch (...)
  {
    std::cerr << "\n❌ Test failed with unknown exception" << std::endl;
    return 1;
  }
}
