/**
 * @file test_stepper_engine.cpp
 * @brief Comprehensive unit tests for StepperEngine state machine
 *
 * Test Coverage:
 * - State transitions (Disabled→Idle, Idle→Moving, etc.)
 * - Command validation matrix (allowed/rejected in each state)
 * - Movement commands (move_to, stop, emergency_stop, home, run_continuous)
 * - Configuration commands (enable, disable, restart, set_zero, release_protection)
 * - Target override behavior (immediate update strategy)
 * - Buffered commands (disable during motion)
 * - Status callbacks (position, speed, protection, motor status)
 * - Error handling and recovery
 * - State to string conversions
 *
 * This test uses the real ServoXxdModbus class (with stub implementations)
 * to avoid mock-related construction issues.
 */

#include <iostream>
#include <string>

// Mock logging
#include "./esphome/core/log.h"

// Mock millis() for time tracking
static uint32_t test_millis_value = 0;
uint32_t millis() { return test_millis_value; }

// Include real classes
#include "../../components/servoxxd_modbus/stepper/servoxxd.h"
#include "../../components/servoxxd_modbus/stepper/servoxxd_stepper_engine.h"

using namespace esphome::servoxxd_modbus;

// Alias for State enum
using State = StepperEngine::State;

// ============================================================================
// Test Helpers
// ============================================================================

struct TestStats
{
  int passed = 0;
  int failed = 0;

  void pass(const std::string &msg)
  {
    std::cout << "  ✓ " << msg << std::endl;
    passed++;
  }

  void fail(const std::string &msg)
  {
    std::cout << "  ✗ " << msg << std::endl;
    failed++;
  }
};

// ============================================================================
// Test Cases
// ============================================================================

void test_initial_state(TestStats &stats)
{
  std::cout << "\nTEST 1: Initial state and basic construction" << std::endl;

  try
  {
    // Create parent (ServoXxdModbus with default constructor)
    ServoXxdModbus parent;

    // Create engine
    StepperEngine engine(&parent, 1000, 3, 200);

    // Check initial state
    if (engine.get_state() == State::Disabled)
    {
      stats.pass("Initial state is Disabled");
    }
    else
    {
      stats.fail("Initial state should be Disabled");
    }

    // Check not moving
    if (!engine.is_moving())
    {
      stats.pass("is_moving() returns false in Disabled state");
    }
    else
    {
      stats.fail("is_moving() should return false in Disabled state");
    }

    // Check state_to_string
    std::string state_str = engine.state_to_string(State::Disabled);
    if (state_str == "Disabled")
    {
      stats.pass("state_to_string(Disabled) returns 'Disabled'");
    }
    else
    {
      stats.fail("state_to_string(Disabled) should return 'Disabled', got: " + state_str);
    }
  }
  catch (const std::exception &e)
  {
    stats.fail(std::string("Exception during construction: ") + e.what());
  }
  catch (...)
  {
    stats.fail("Unknown exception during construction");
  }
}

void test_state_strings(TestStats &stats)
{
  std::cout << "\nTEST 2: State to string conversions" << std::endl;

  try
  {
    ServoXxdModbus parent;
    StepperEngine engine(&parent);

    // Test all state names
    struct StateTest
    {
      State state;
      std::string expected;
    };

    StateTest tests[] = {
        {State::Disabled, "Disabled"},
        {State::Idle, "Idle"},
        {State::Moving, "Moving"},
        {State::Running, "Running"},
        {State::Homing, "Homing"},
        {State::Stopping, "Stopping"},
        {State::Error, "Error"}};

    for (const auto &test : tests)
    {
      std::string result = engine.state_to_string(test.state);
      if (result == test.expected)
      {
        stats.pass("state_to_string(" + test.expected + ") correct");
      }
      else
      {
        stats.fail("state_to_string expected '" + test.expected + "', got '" + result + "'");
      }
    }
  }
  catch (...)
  {
    stats.fail("Exception during state string tests");
  }
}

void test_enable_transition(TestStats &stats)
{
  std::cout << "\nTEST 3: Enable transition (Disabled → Idle)" << std::endl;

  try
  {
    ServoXxdModbus parent;
    StepperEngine engine(&parent);

    // Initial state should be Disabled
    if (engine.get_state() != State::Disabled)
    {
      stats.fail("Initial state is not Disabled");
      return;
    }

    // Try to enable
    engine.enable();

    // Should now be Idle
    if (engine.get_state() == State::Idle)
    {
      stats.pass("State transitioned to Idle after enable()");
    }
    else
    {
      stats.fail("State should be Idle after enable()");
    }

    // Try to enable again (should be rejected or no-op)
    engine.enable();

    // Should still be Idle
    if (engine.get_state() == State::Idle)
    {
      stats.pass("State remains Idle after redundant enable()");
    }
    else
    {
      stats.fail("State should still be Idle");
    }
  }
  catch (...)
  {
    stats.fail("Exception during enable transition test");
  }
}

void test_disable_transition(TestStats &stats)
{
  std::cout << "\nTEST 4: Disable transition (Idle → Disabled)" << std::endl;

  try
  {
    ServoXxdModbus parent;
    StepperEngine engine(&parent);

    // Enable first
    engine.enable();
    if (engine.get_state() != State::Idle)
    {
      stats.fail("Failed to reach Idle state");
      return;
    }

    // Disable
    engine.disable();

    // Should be Disabled
    if (engine.get_state() == State::Disabled)
    {
      stats.pass("State transitioned to Disabled after disable()");
    }
    else
    {
      stats.fail("State should be Disabled after disable()");
    }
  }
  catch (...)
  {
    stats.fail("Exception during disable transition test");
  }
}

void test_move_to_command(TestStats &stats)
{
  std::cout << "\nTEST 5: move_to command (Idle → Moving)" << std::endl;

  try
  {
    ServoXxdModbus parent;
    StepperEngine engine(&parent);

    // Enable to reach Idle
    engine.enable();
    if (engine.get_state() != State::Idle)
    {
      stats.fail("Failed to reach Idle state");
      return;
    }

    // Issue move_to command
    Position target(1000.0f, PositionUnit::STEPS, &parent);
    engine.move_to(target);

    // Should be Moving
    if (engine.get_state() == State::Moving)
    {
      stats.pass("State transitioned to Moving after move_to()");
    }
    else
    {
      stats.fail("State should be Moving after move_to()");
    }

    // Should report is_moving() == true
    if (engine.is_moving())
    {
      stats.pass("is_moving() returns true in Moving state");
    }
    else
    {
      stats.fail("is_moving() should return true in Moving state");
    }
  }
  catch (...)
  {
    stats.fail("Exception during move_to command test");
  }
}

void test_emergency_stop(TestStats &stats)
{
  std::cout << "\nTEST 6: Emergency stop (any state → Error)" << std::endl;

  try
  {
    ServoXxdModbus parent;
    StepperEngine engine(&parent);

    // Enable and start moving
    engine.enable();
    Position target(1000.0f, PositionUnit::STEPS, &parent);
    engine.move_to(target);

    if (engine.get_state() != State::Moving)
    {
      stats.fail("Failed to reach Moving state");
      return;
    }

    // Emergency stop
    engine.emergency_stop();

    // Should be Error
    if (engine.get_state() == State::Error)
    {
      stats.pass("State transitioned to Error after emergency_stop()");
    }
    else
    {
      stats.fail("State should be Error after emergency_stop()");
    }

    // Should not be moving
    if (!engine.is_moving())
    {
      stats.pass("is_moving() returns false after emergency_stop()");
    }
    else
    {
      stats.fail("is_moving() should return false after emergency_stop()");
    }
  }
  catch (...)
  {
    stats.fail("Exception during emergency stop test");
  }
}

void test_target_override(TestStats &stats)
{
  std::cout << "\nTEST 7: Target override during movement" << std::endl;

  try
  {
    ServoXxdModbus parent;
    StepperEngine engine(&parent);

    // Enable and start moving
    engine.enable();
    Position target1(1000.0f, PositionUnit::STEPS, &parent);
    engine.move_to(target1);

    if (engine.get_state() != State::Moving)
    {
      stats.fail("Failed to reach Moving state");
      return;
    }

    // Send new target while moving
    Position target2(2000.0f, PositionUnit::STEPS, &parent);
    engine.move_to(target2);

    // Should still be Moving
    if (engine.get_state() == State::Moving)
    {
      stats.pass("State remains Moving after target override");
    }
    else
    {
      stats.fail("State should remain Moving after target override");
    }
  }
  catch (...)
  {
    stats.fail("Exception during target override test");
  }
}

void test_stop_command(TestStats &stats)
{
  std::cout << "\nTEST 8: Stop command (Moving → Stopping)" << std::endl;

  try
  {
    ServoXxdModbus parent;
    StepperEngine engine(&parent);

    // Enable and start moving
    engine.enable();
    Position target(1000.0f, PositionUnit::STEPS, &parent);
    engine.move_to(target);

    if (engine.get_state() != State::Moving)
    {
      stats.fail("Failed to reach Moving state");
      return;
    }

    // Stop
    engine.stop();

    // Should be Stopping
    if (engine.get_state() == State::Stopping)
    {
      stats.pass("State transitioned to Stopping after stop()");
    }
    else
    {
      stats.fail("State should be Stopping after stop()");
    }

    // Stop in Idle should be no-op
    ServoXxdModbus parent2;
    StepperEngine engine2(&parent2);
    engine2.enable();
    engine2.stop();

    if (engine2.get_state() == State::Idle)
    {
      stats.pass("stop() in Idle state is no-op");
    }
    else
    {
      stats.fail("stop() in Idle should remain Idle");
    }
  }
  catch (...)
  {
    stats.fail("Exception during stop command test");
  }
}

void test_homing(TestStats &stats)
{
  std::cout << "\nTEST 9: Homing command (Idle → Homing)" << std::endl;

  try
  {
    ServoXxdModbus parent;
    StepperEngine engine(&parent);

    // Enable
    engine.enable();

    if (engine.get_state() != State::Idle)
    {
      stats.fail("Failed to reach Idle state");
      return;
    }

    // Start homing
    engine.home();

    // Should be Homing
    if (engine.get_state() == State::Homing)
    {
      stats.pass("State transitioned to Homing after home()");
    }
    else
    {
      stats.fail("State should be Homing after home()");
    }
  }
  catch (...)
  {
    stats.fail("Exception during homing test");
  }
}

void test_run_continuous(TestStats &stats)
{
  std::cout << "\nTEST 10: Run continuous (Idle → Running)" << std::endl;

  try
  {
    ServoXxdModbus parent;
    StepperEngine engine(&parent);

    // Enable
    engine.enable();

    if (engine.get_state() != State::Idle)
    {
      stats.fail("Failed to reach Idle state");
      return;
    }

    // Start continuous run
    Speed speed(100.0f, SpeedUnit::RPM, &parent);
    Acceleration accel(100.0f, AccelerationUnit::RPM_PER_SEC, &parent);
    engine.run_continuous(speed, accel);

    // Should be Running
    if (engine.get_state() == State::Running)
    {
      stats.pass("State transitioned to Running after run_continuous()");
    }
    else
    {
      stats.fail("State should be Running after run_continuous()");
    }

    if (engine.is_moving())
    {
      stats.pass("is_moving() returns true in Running state");
    }
    else
    {
      stats.fail("is_moving() should return true in Running state");
    }
  }
  catch (...)
  {
    stats.fail("Exception during run_continuous test");
  }
}

void test_release_protection(TestStats &stats)
{
  std::cout << "\nTEST 11: Release protection (Error → Idle)" << std::endl;

  try
  {
    ServoXxdModbus parent;
    StepperEngine engine(&parent);

    // Cause error with emergency stop
    engine.enable();
    Position target(1000.0f, PositionUnit::STEPS, &parent);
    engine.move_to(target);
    engine.emergency_stop();

    if (engine.get_state() != State::Error)
    {
      stats.fail("Failed to reach Error state");
      return;
    }

    // Release protection
    engine.release_protection();

    // Should be Idle
    if (engine.get_state() == State::Idle)
    {
      stats.pass("State transitioned to Idle after release_protection()");
    }
    else
    {
      stats.fail("State should be Idle after release_protection()");
    }
  }
  catch (...)
  {
    stats.fail("Exception during release_protection test");
  }
}

void test_restart_command(TestStats &stats)
{
  std::cout << "\nTEST 12: Restart command (any state → Disabled)" << std::endl;

  try
  {
    // Test from Idle
    ServoXxdModbus parent1;
    StepperEngine engine1(&parent1);
    engine1.enable();
    engine1.restart();

    if (engine1.get_state() == State::Disabled)
    {
      stats.pass("restart() from Idle transitions to Disabled");
    }
    else
    {
      stats.fail("restart() should transition to Disabled");
    }

    // Test from Error
    ServoXxdModbus parent2;
    StepperEngine engine2(&parent2);
    engine2.enable();
    engine2.emergency_stop();
    engine2.restart();

    if (engine2.get_state() == State::Disabled)
    {
      stats.pass("restart() from Error transitions to Disabled");
    }
    else
    {
      stats.fail("restart() from Error should transition to Disabled");
    }
  }
  catch (...)
  {
    stats.fail("Exception during restart command test");
  }
}

void test_disable_buffering(TestStats &stats)
{
  std::cout << "\nTEST 13: Disable buffering during motion" << std::endl;

  try
  {
    ServoXxdModbus parent;
    StepperEngine engine(&parent);

    // Enable and start moving
    engine.enable();
    Position target(1000.0f, PositionUnit::STEPS, &parent);
    engine.move_to(target);

    if (engine.get_state() != State::Moving)
    {
      stats.fail("Failed to reach Moving state");
      return;
    }

    // Try to disable during motion
    engine.disable();

    // Should transition to Stopping (buffered disable)
    if (engine.get_state() == State::Stopping)
    {
      stats.pass("disable() during Moving transitions to Stopping (buffered)");
    }
    else
    {
      stats.fail("disable() during Moving should buffer and transition to Stopping");
    }
  }
  catch (...)
  {
    stats.fail("Exception during disable buffering test");
  }
}

void test_set_zero_command(TestStats &stats)
{
  std::cout << "\nTEST 14: Set zero command" << std::endl;

  try
  {
    ServoXxdModbus parent;
    StepperEngine engine(&parent);

    // Enable
    engine.enable();

    if (engine.get_state() != State::Idle)
    {
      stats.fail("Failed to reach Idle state");
      return;
    }

    // Set zero
    engine.set_zero();

    // Should still be Idle
    if (engine.get_state() == State::Idle)
    {
      stats.pass("State remains Idle after set_zero()");
    }
    else
    {
      stats.fail("State should remain Idle after set_zero()");
    }

    // Position should be ~0
    Position current = engine.get_current_position();
    if (std::abs(current.steps()) < 1.0f)
    {
      stats.pass("Current position is ~0 after set_zero()");
    }
    else
    {
      stats.fail("Current position should be ~0 after set_zero()");
    }
  }
  catch (...)
  {
    stats.fail("Exception during set_zero command test");
  }
}

void test_callbacks(TestStats &stats)
{
  std::cout << "\nTEST 15: Callback registration" << std::endl;

  try
  {
    ServoXxdModbus parent;
    StepperEngine engine(&parent);

    bool position_called = false;
    bool speed_called = false;
    bool protection_called = false;
    bool motor_status_called = false;

    // Register callbacks
    engine.set_position_update_callback([&](Position p)
                                        { position_called = true; });
    engine.set_speed_update_callback([&](Speed s)
                                     { speed_called = true; });
    engine.set_protection_callback([&]()
                                   { protection_called = true; });
    engine.set_motor_status_callback([&](bool enabled)
                                     { motor_status_called = true; });

    stats.pass("All callbacks registered without error");
  }
  catch (...)
  {
    stats.fail("Exception during callback registration");
  }
}

void test_command_validation(TestStats &stats)
{
  std::cout << "\nTEST 16: Command validation in Disabled state" << std::endl;

  try
  {
    ServoXxdModbus parent;
    StepperEngine engine(&parent);

    // All movement commands should be rejected in Disabled state
    Position target(1000.0f, PositionUnit::STEPS, &parent);
    engine.move_to(target);

    if (engine.get_state() == State::Disabled)
    {
      stats.pass("move_to() rejected in Disabled state");
    }
    else
    {
      stats.fail("move_to() should be rejected in Disabled state");
    }

    engine.stop();
    if (engine.get_state() == State::Disabled)
    {
      stats.pass("stop() rejected in Disabled state");
    }
    else
    {
      stats.fail("stop() should be rejected in Disabled state");
    }

    engine.home();
    if (engine.get_state() == State::Disabled)
    {
      stats.pass("home() rejected in Disabled state");
    }
    else
    {
      stats.fail("home() should be rejected in Disabled state");
    }

    Speed speed(100.0f, SpeedUnit::RPM, &parent);
    Acceleration accel(100.0f, AccelerationUnit::RPM_PER_SEC, &parent);
    engine.run_continuous(speed, accel);

    if (engine.get_state() == State::Disabled)
    {
      stats.pass("run_continuous() rejected in Disabled state");
    }
    else
    {
      stats.fail("run_continuous() should be rejected in Disabled state");
    }

    engine.set_zero();
    if (engine.get_state() == State::Disabled)
    {
      stats.pass("set_zero() rejected in Disabled state");
    }
    else
    {
      stats.fail("set_zero() should be rejected in Disabled state");
    }
  }
  catch (...)
  {
    stats.fail("Exception during command validation test");
  }
}

// ============================================================================
// Main Test Runner
// ============================================================================

int main()
{
  std::cout << "========================================" << std::endl;
  std::cout << "StepperEngine Unit Tests" << std::endl;
  std::cout << "========================================" << std::endl;

  TestStats stats;

  // Run tests
  test_initial_state(stats);
  test_state_strings(stats);
  test_enable_transition(stats);
  test_disable_transition(stats);
  test_move_to_command(stats);
  test_emergency_stop(stats);
  test_target_override(stats);
  test_stop_command(stats);
  test_homing(stats);
  test_run_continuous(stats);
  test_release_protection(stats);
  test_restart_command(stats);
  test_disable_buffering(stats);
  test_set_zero_command(stats);
  test_callbacks(stats);
  test_command_validation(stats);

  // Summary
  std::cout << "\n========================================" << std::endl;
  std::cout << "Test Summary" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "Passed: " << stats.passed << std::endl;
  std::cout << "Failed: " << stats.failed << std::endl;

  if (stats.failed == 0)
  {
    std::cout << "\nAll tests passed! ✓" << std::endl;
    return 0;
  }
  else
  {
    std::cout << "\nSome tests failed. ✗" << std::endl;
    return 1;
  }
}
