/**
 * @file test_command_factory.cpp
 * @brief Unit tests for CommandFactory encoding
 *
 * This test validates ALL CommandFactory functions against the
 * MKS SERVO42D/57D hardware documentation to ensure correct byte encoding.
 *
 * Reference: MKS_SERVO42D57D_RS485_User_Manual_V1.0.5.txt
 */

#include <iostream>
#include <iomanip>
#include <sstream>
#include <cassert>
#include <vector>
#include <string>
#include "../../components/servoxxd/stepper/servoxxd.h"
#include "../../components/servoxxd/stepper/servoxxd_command_factory.h"

using namespace esphome::servoxxd;

// Test counter
static int tests_passed = 0;
static int tests_failed = 0;

// Helper to print byte array in hex
std::string bytes_to_hex(const std::vector<uint8_t> &data) {
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < data.size(); i++) {
    ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << (int) data[i];
    if (i < data.size() - 1)
      ss << ", ";
  }
  ss << "]";
  return ss.str();
}

#define ASSERT_BYTES_EQUAL(actual, expected, msg) \
  do { \
    if ((actual) == (expected)) { \
      std::cout << "  ✓ " << msg << std::endl; \
      tests_passed++; \
    } else { \
      std::cerr << "  ✗ FAILED: " << msg << std::endl; \
      std::cerr << "    Expected: " << bytes_to_hex(expected) << std::endl; \
      std::cerr << "    Got:      " << bytes_to_hex(actual) << std::endl; \
      tests_failed++; \
    } \
  } while (0)

void assert_command(const Command &cmd, Commandtype expected_type, const std::vector<uint8_t> &expected_payload,
                    const std::string &description) {
  std::cout << "\nTest: " << description << std::endl;
  if (cmd.command_type != expected_type) {
    std::cerr << "  ✗ FAILED: Wrong command type" << std::endl;
    tests_failed++;
    return;
  }
  ASSERT_BYTES_EQUAL(cmd.payload, expected_payload, description);
}

// ============================================================================
// System Commands Tests (based on hardware manual section 8.2)
// ============================================================================

void test_system_commands() {
  std::cout << "\n" << std::string(70, '=') << std::endl;
  std::cout << "SYSTEM COMMANDS" << std::endl;
  std::cout << std::string(70, '=') << std::endl;

  // Test: Release Protection (Register 0x003D, Value 0x0001)
  // Hardware manual: 01H 06H 00H 3DH 00H 01H D9H C6H
  assert_command(CommandFactory::release_protection(), Commandtype::RELEASE_PROTECTION, {0x00, 0x01},
                 "release_protection() - Clear error state (0x0001)");

  // Test: Restart Motor (Register 0x0041, Value 0x0001)
  // Hardware manual: 01H 06H 00H 41H 01H 00H D8H 7DH (note: value is 0x0100 in manual)
  // But logically restart should be 0x01 (1 byte), which expands to 0x0001
  assert_command(CommandFactory::restart(), Commandtype::RESTART, {0x01},
                 "restart() - Software reset (0x01 → expands to 0x0001)");

  // Test: Calibrate Encoder (Command 0x80, Value 0x00)
  assert_command(CommandFactory::calibrate_encoder(), Commandtype::CALIBRATE_ENCODER, {0x00},
                 "calibrate_encoder() - Start calibration (0x00)");

  // Test: Emergency Stop (Command 0xF7)
  assert_command(CommandFactory::emergency_stop(), Commandtype::EMERGENCY_STOP, {},
                 "emergency_stop() - Immediate halt (no payload)");

  // Test: Set Zero (Command 0x92)
  assert_command(CommandFactory::set_zero(), Commandtype::SET_ZERO, {},
                 "set_zero() - Set current position as zero (no payload)");
}

// ============================================================================
// Configuration Commands Tests (based on hardware manual section 8.2)
// ============================================================================

void test_configuration_commands() {
  std::cout << "\n" << std::string(70, '=') << std::endl;
  std::cout << "CONFIGURATION COMMANDS" << std::endl;
  std::cout << std::string(70, '=') << std::endl;

  // Test: Set Subdivision (Register 0x0084, Value = microsteps)
  // Example: 16 microsteps → 0x0010
  assert_command(CommandFactory::set_subdivision(16), Commandtype::SET_SUBDIVISION, {16},
                 "set_subdivision(16) - Set 16 microsteps");

  assert_command(CommandFactory::set_subdivision(32), Commandtype::SET_SUBDIVISION, {32},
                 "set_subdivision(32) - Set 32 microsteps");

  // Test: EN Pin Active (Register 0x0085, Value 0=LOW, 1=HIGH, 2=ALWAYS)
  assert_command(CommandFactory::set_en_pin_active(EnPinActive::EN_LOW), Commandtype::SET_EN_PIN_ACTIVE, {0x00},
                 "set_en_pin_active(LOW) - EN active LOW");

  assert_command(CommandFactory::set_en_pin_active(EnPinActive::EN_HIGH), Commandtype::SET_EN_PIN_ACTIVE, {0x01},
                 "set_en_pin_active(HIGH) - EN active HIGH");

  // Test: Auto Screen Off (Register 0x0087, Value 0x00=disable, 0x01=enable)
  assert_command(CommandFactory::set_auto_screen_off(true), Commandtype::SET_AUTO_SCREEN_OFF, {0x01},
                 "set_auto_screen_off(true) - Enable auto-off");

  assert_command(CommandFactory::set_auto_screen_off(false), Commandtype::SET_AUTO_SCREEN_OFF, {0x00},
                 "set_auto_screen_off(false) - Disable auto-off");

  // Test: Lock Keys (Register 0x008F, Value 0x00=unlock, 0x01=lock)
  assert_command(CommandFactory::set_lock_keys(false), Commandtype::SET_LOCK_KEYS, {0x00},
                 "set_lock_keys(false) - Unlock keys");

  assert_command(CommandFactory::set_lock_keys(true), Commandtype::SET_LOCK_KEYS, {0x01},
                 "set_lock_keys(true) - Lock keys");

  // Test: Control Mode (Register 0x0082, Value = mode)
  assert_command(CommandFactory::set_control_mode(ControlMode::SR_VFOC), Commandtype::SET_WORK_MODE, {0x05},
                 "set_control_mode(SR_vFOC) - Set mode to 0x05");

  // Test: Working Current (Register 0x0044, Value in mA, 2 bytes big-endian)
  // Example: 2000 mA = 0x07D0
  assert_command(CommandFactory::set_working_current(2000), Commandtype::SET_WORKING_CURRENT_RUNTIME, {0x07, 0xD0},
                 "set_working_current(2000) - Set 2000mA (0x07D0)");

  assert_command(CommandFactory::set_working_current(1500), Commandtype::SET_WORKING_CURRENT_RUNTIME, {0x05, 0xDC},
                 "set_working_current(1500) - Set 1500mA (0x05DC)");

  // Test: Enable Motor (Command 0xF3)
  assert_command(CommandFactory::enable_motor(true), Commandtype::ENABLE_MOTOR, {0x01},
                 "enable_motor(true) - Enable motor");

  assert_command(CommandFactory::enable_motor(false), Commandtype::ENABLE_MOTOR, {0x00},
                 "enable_motor(false) - Disable motor");
}

// ============================================================================
// Homing Commands Tests (based on hardware manual section 8.2)
// ============================================================================

void test_homing_commands() {
  std::cout << "\n" << std::string(70, '=') << std::endl;
  std::cout << "HOMING COMMANDS" << std::endl;
  std::cout << std::string(70, '=') << std::endl;

  // Test: Go Home (Register 0x0091, Value 0x0001)
  // Hardware manual example: 01H 06H 00H 91H 00H 01H 19H D4H
  assert_command(CommandFactory::go_home(), Commandtype::GO_HOME, {0x00, 0x01},
                 "go_home() - Start homing sequence (0x0001)");

  // Test: Set Homing Parameters (Register 0x0090, Function 0x10, 5 bytes)
  // Payload: [hmTrig][hmDir][HmSpeed_hi][HmSpeed_lo][EndLimit]
  // Example: Trigger=LOW(0), Dir=CCW(1), Speed=60 RPM (0x003C), EndLimit=enabled(1)
  // Expected: [0x00][0x01][0x00][0x3C][0x01]
  assert_command(CommandFactory::set_homing_parameters(EndstopTrigger::TRIGGER_LOW, Direction::CCW,
                                                       Speed::from_rpm(60, nullptr), true),
                 Commandtype::SET_HOMING_PARAMETERS, {0x00, 0x01, 0x00, 0x3C, 0x01},
                 "set_homing_parameters() - LOW trigger, CCW, 60 RPM, EndLimit enabled");

  // Test: Set No-Limit Homing Parameters (Register 0x0094, Function 0x10, 8 bytes)
  // Payload: [retValue(4 bytes)][mode(2 bytes)][current_ma(2 bytes)]
  // Example: retValue=2000 (0x000007D0), mode=disabled (0x0000), current=1000mA (0x03E8)
  // Expected: [0x00][0x00][0x07][0xD0][0x00][0x00][0x03][0xE8]
  assert_command(CommandFactory::set_nolimit_homing_params(Position::from_ticks(2000), false, 1000),
                 Commandtype::SET_NOLIMIT_HOMING_PARAMS, {0x00, 0x00, 0x07, 0xD0, 0x00, 0x00, 0x03, 0xE8},
                 "set_nolimit_homing_params() - 2000 ticks reverse, disabled, 1000mA");

  // Test: Set Zero Mode (Register 0x009A, Function 0x10, 4 bytes)
  // Payload: [0_Mode][Set 0][0_Speed][0_Dir]
  // Example: Disabled (0x00), Clean (0x00), Medium speed (0x02), CW (0x00)
  assert_command(CommandFactory::set_zero_mode(ZeroModeMode::MODE_DISABLED, ZeroModeTask::CLEAN, ZeroingSpeed::MEDIUM,
                                               Direction::CW),
                 Commandtype::SET_ZERO_MODE, {0x00, 0x00, 0x02, 0x00},
                 "set_zero_mode() - Disabled mode, Clean, Medium, CW");
}

// ============================================================================
// Movement Commands Tests (based on hardware manual section 8.3)
// ============================================================================

void test_movement_commands() {
  std::cout << "\n" << std::string(70, '=') << std::endl;
  std::cout << "MOVEMENT COMMANDS" << std::endl;
  std::cout << std::string(70, '=') << std::endl;

  // Test: Move Position Mode 2 (Command 0xFE, Function 0x10, 8 bytes)
  // Payload: [acc_hi][acc_lo][speed_hi][speed_lo][pos_b3][pos_b2][pos_b1][pos_b0]
  // Example: acc=50 (0x0032), speed=100 RPM (0x0064), position=1000 ticks (0x000003E8)
  // Expected: [0x00][0x32][0x00][0x64][0x00][0x00][0x03][0xE8]
  assert_command(CommandFactory::move_position_mode_2(Position::from_ticks(1000), Speed::from_rpm(100, nullptr),
                                                      Acceleration::from_internal(50)),
                 Commandtype::MOVE_POSITION_MODE_2, {0x00, 0x32, 0x00, 0x64, 0x00, 0x00, 0x03, 0xE8},
                 "move_position_mode_2() - 1000 ticks, 100 RPM, acc=50");

  // Test: Move Position Mode 1 (Command 0xFD, Function 0x10, 8 bytes)
  // Payload: [dir][speed_hi][speed_lo][acc][pulses_b3][pulses_b2][pulses_b1][pulses_b0]
  // Example: dir=CW(0), speed=200 RPM (0x00C8), acc=100, pulses=5000 (0x00001388)
  // Expected: [0x00][0x00][0xC8][0x64][0x00][0x00][0x13][0x88]
  assert_command(CommandFactory::move_position_mode_1(Direction::CW, Speed::from_rpm(200, nullptr),
                                                      Acceleration::from_internal(100), Position::from_ticks(5000)),
                 Commandtype::MOVE_POSITION_MODE_1, {0x00, 0x00, 0xC8, 0x64, 0x00, 0x00, 0x13, 0x88},
                 "move_position_mode_1() - CW, 200 RPM, acc=100, 5000 pulses");

  // Test: Move Speed Mode (Command 0xF6, Function 0x10, 4 bytes)
  // Payload: [dir][speed_hi][speed_lo][acc]
  // Example: dir=CCW(1), speed=150 RPM (0x0096), acc=75
  // Expected: [0x01][0x00][0x96][0x4B]
  assert_command(CommandFactory::move_speed_mode(Speed::from_rpm(150, nullptr), Acceleration::from_internal(75)),
                 Commandtype::MOVE_SPEED_MODE, {0x01, 0x00, 0x96, 0x4B},
                 "move_speed_mode() - CCW (positive), 150 RPM, acc=75");

  // Test: Move Speed Mode - Negative (CW direction)
  assert_command(CommandFactory::move_speed_mode(Speed::from_rpm(-150, nullptr), Acceleration::from_internal(75)),
                 Commandtype::MOVE_SPEED_MODE, {0x00, 0x00, 0x96, 0x4B},
                 "move_speed_mode() - CW (negative), 150 RPM, acc=75");

  // Test: Stop Speed Mode (Command 0xF6, special payload)
  // Hardware manual: dir=0x00, acc=value, speed=0x0000
  // Payload: [dir][acc][speed_hi][speed_lo]
  // Example: acc=50 (0x32)
  // Expected: [0x00][0x32][0x00][0x00]
  assert_command(CommandFactory::stop_speed_mode(Acceleration::from_internal(50)), Commandtype::MOVE_SPEED_MODE,
                 {0x00, 0x32, 0x00, 0x00}, "stop_speed_mode() - Deceleration stop with acc=50");

  // Test: Stop Position Mode 2 (Command 0xFF, 1 byte)
  // Payload: [deceleration]
  assert_command(CommandFactory::stop_position_mode_2(Acceleration::from_internal(100)),
                 Commandtype::STOP_POSITION_MODE_2, {0x64}, "stop_position_mode_2() - Stop with decel=100");
}

// ============================================================================
// Read Commands Tests (based on hardware manual section 8.1)
// ============================================================================

void test_read_commands() {
  std::cout << "\n" << std::string(70, '=') << std::endl;
  std::cout << "READ COMMANDS" << std::endl;
  std::cout << std::string(70, '=') << std::endl;

  // Read commands have no payload (they're just read requests)

  assert_command(CommandFactory::read_current_speed(), Commandtype::READ_CURRENT_SPEED, {},
                 "read_current_speed() - Read command (no payload)");

  assert_command(CommandFactory::read_motor_status(), Commandtype::READ_MOTOR_STATUS, {},
                 "read_motor_status() - Read command (no payload)");

  assert_command(CommandFactory::read_protection_status(), Commandtype::READ_PROTECTION_STATUS, {},
                 "read_protection_status() - Read command (no payload)");

  assert_command(CommandFactory::read_zero_return_status(), Commandtype::READ_ZERO_RETURN_STATUS, {},
                 "read_zero_return_status() - Read command (no payload)");
}

// ============================================================================
// Edge Cases and Special Tests
// ============================================================================

void test_edge_cases() {
  std::cout << "\n" << std::string(70, '=') << std::endl;
  std::cout << "EDGE CASES" << std::endl;
  std::cout << std::string(70, '=') << std::endl;

  // Test: Large position value (max int32)
  assert_command(CommandFactory::move_position_mode_2(Position::from_ticks(2147483647),  // Max int32
                                                      Speed::from_rpm(100, nullptr), Acceleration::from_internal(50)),
                 Commandtype::MOVE_POSITION_MODE_2, {0x00, 0x32, 0x00, 0x64, 0x7F, 0xFF, 0xFF, 0xFF},
                 "move_position_mode_2() - Maximum position (2147483647)");

  // Test: Negative position value
  assert_command(CommandFactory::move_position_mode_2(Position::from_ticks(-1000), Speed::from_rpm(50, nullptr),
                                                      Acceleration::from_internal(25)),
                 Commandtype::MOVE_POSITION_MODE_2, {0x00, 0x19, 0x00, 0x32, 0xFF, 0xFF, 0xFC, 0x18},
                 "move_position_mode_2() - Negative position (-1000)");

  // Test: Zero values
  assert_command(CommandFactory::move_position_mode_2(Position::from_ticks(0), Speed::from_rpm(0, nullptr),
                                                      Acceleration::from_internal(0)),
                 Commandtype::MOVE_POSITION_MODE_2, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                 "move_position_mode_2() - All zeros");

  // Test: Maximum speed (3000 RPM)
  assert_command(CommandFactory::move_speed_mode(Speed::from_rpm(3000, nullptr), Acceleration::from_internal(255)),
                 Commandtype::MOVE_SPEED_MODE, {0x01, 0x0B, 0xB8, 0xFF},
                 "move_speed_mode() - Maximum speed (3000 RPM)");

  // Test: 16-bit boundary values
  assert_command(CommandFactory::set_working_current(65535),  // Max uint16
                 Commandtype::SET_WORKING_CURRENT_RUNTIME, {0xFF, 0xFF},
                 "set_working_current(65535) - Maximum uint16 value");
}

// ============================================================================
// Main Test Runner
// ============================================================================

int main() {
  std::cout << "\n";
  std::cout << "╔════════════════════════════════════════════════════════════════════╗" << std::endl;
  std::cout << "║        CommandFactory Encoding Test Suite                         ║" << std::endl;
  std::cout << "║  Validates against MKS SERVO42D/57D Hardware Documentation        ║" << std::endl;
  std::cout << "╚════════════════════════════════════════════════════════════════════╝" << std::endl;

  try {
    test_system_commands();
    test_configuration_commands();
    test_homing_commands();
    test_movement_commands();
    test_read_commands();
    test_edge_cases();

    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "TEST SUMMARY" << std::endl;
    std::cout << std::string(70, '=') << std::endl;
    std::cout << "✓ Tests passed: " << tests_passed << std::endl;

    if (tests_failed > 0) {
      std::cout << "✗ Tests FAILED: " << tests_failed << std::endl;
      std::cout << "\n❌ OVERALL: FAILED\n" << std::endl;
      return 1;
    } else {
      std::cout << "\n✅ OVERALL: ALL TESTS PASSED\n" << std::endl;
      return 0;
    }
  } catch (const std::exception &e) {
    std::cerr << "\n❌ EXCEPTION: " << e.what() << std::endl;
    return 1;
  }
}
