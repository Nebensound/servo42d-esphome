#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include "../../components/servoxxd/stepper/servoxxd_command_decoder.h"

using namespace esphome::servoxxd;

// Test counter
static int tests_passed = 0;

#define ASSERT_EQUAL(actual, expected, msg)            \
  do                                                   \
  {                                                    \
    if ((actual) == (expected))                        \
    {                                                  \
      std::cout << "  ✓ " << msg << std::endl;         \
      tests_passed++;                                  \
    }                                                  \
    else                                               \
    {                                                  \
      std::cerr << "  ✗ FAILED: " << msg << std::endl; \
      std::cerr << "    Expected: " << (expected)      \
                << ", Got: " << (actual) << std::endl; \
      exit(1);                                         \
    }                                                  \
  } while (0)

#define ASSERT_TRUE(condition, msg)                    \
  do                                                   \
  {                                                    \
    if (condition)                                     \
    {                                                  \
      std::cout << "  ✓ " << msg << std::endl;         \
      tests_passed++;                                  \
    }                                                  \
    else                                               \
    {                                                  \
      std::cerr << "  ✗ FAILED: " << msg << std::endl; \
      exit(1);                                         \
    }                                                  \
  } while (0)

// ============================================================================
// DECODER TESTS - Testing response parsing from hardware
// All tests validated against MKS SERVO42D/57D_RS485 V1.0.6 User Manual
// ============================================================================

/**
 * Test read_current_speed decoder
 * Hardware Doc: Section 8.1.3 - Read the real-time speed of the motor
 * Register: 0x32, Function: 0x04, Response: 2 bytes [hi][lo] (int16_t RPM)
 * Note: CCW rotation = positive RPM, CW rotation = negative RPM
 */
void test_read_current_speed()
{
  std::cout << "\n=== read_current_speed Tests (Register 0x32) ===" << std::endl;

  // Test positive speed (CCW rotation)
  Command cmd1(Commandtype::READ_CURRENT_SPEED);
  cmd1.response = {0x00, 0x64}; // 100 RPM
  Speed speed1 = CommandDecoder::read_current_speed(cmd1);
  ASSERT_EQUAL(speed1.rpm(), 100, "Decode 100 RPM (CCW)");

  // Test negative speed (CW rotation)
  Command cmd2(Commandtype::READ_CURRENT_SPEED);
  cmd2.response = {0xFF, 0x9C}; // -100 RPM (0xFF9C = -100 in int16_t)
  Speed speed2 = CommandDecoder::read_current_speed(cmd2);
  ASSERT_EQUAL(speed2.rpm(), -100, "Decode -100 RPM (CW)");

  // Test zero speed
  Command cmd3(Commandtype::READ_CURRENT_SPEED);
  cmd3.response = {0x00, 0x00};
  Speed speed3 = CommandDecoder::read_current_speed(cmd3);
  ASSERT_EQUAL(speed3.rpm(), 0, "Decode 0 RPM (stopped)");

  // Test high speed (1500 RPM)
  Command cmd4(Commandtype::READ_CURRENT_SPEED);
  cmd4.response = {0x05, 0xDC}; // 1500 RPM
  Speed speed4 = CommandDecoder::read_current_speed(cmd4);
  ASSERT_EQUAL(speed4.rpm(), 1500, "Decode 1500 RPM (max for CLOSE mode)");

  // Test invalid data size
  Command cmd5(Commandtype::READ_CURRENT_SPEED);
  cmd5.response = {0x00}; // Only 1 byte
  Speed speed5 = CommandDecoder::read_current_speed(cmd5);
  ASSERT_EQUAL(speed5.rpm(), 0, "Invalid data returns default Speed");

  // Test wrong command type
  Command cmd6(Commandtype::READ_PULSE_COUNT); // Wrong type
  cmd6.response = {0x00, 0x64};
  Speed speed6 = CommandDecoder::read_current_speed(cmd6);
  ASSERT_EQUAL(speed6.rpm(), 0, "Wrong command type returns default Speed");
}

/**
 * Test read_pulse_count decoder
 * Hardware Doc: Section 8.1.4 - Read the number of pulses
 * Register: 0x33, Function: 0x04, Response: 4 bytes (uint32_t)
 */
void test_read_pulse_count()
{
  std::cout << "\n=== read_pulse_count Tests (Register 0x33) ===" << std::endl;

  // Test positive position
  Command cmd1(Commandtype::READ_PULSE_COUNT);
  cmd1.response = {0x00, 0x00, 0x03, 0xE8}; // 1000 ticks
  Position pos1 = CommandDecoder::read_pulse_count(cmd1);
  ASSERT_EQUAL(pos1.get_ticks(), 1000, "Decode 1000 ticks");

  // Test negative position
  Command cmd2(Commandtype::READ_PULSE_COUNT);
  cmd2.response = {0xFF, 0xFF, 0xFC, 0x18}; // -1000 ticks
  Position pos2 = CommandDecoder::read_pulse_count(cmd2);
  ASSERT_EQUAL(pos2.get_ticks(), -1000, "Decode -1000 ticks");

  // Test zero position
  Command cmd3(Commandtype::READ_PULSE_COUNT);
  cmd3.response = {0x00, 0x00, 0x00, 0x00};
  Position pos3 = CommandDecoder::read_pulse_count(cmd3);
  ASSERT_EQUAL(pos3.get_ticks(), 0, "Decode 0 ticks");

  // Test large position (100,000 ticks)
  Command cmd4(Commandtype::READ_PULSE_COUNT);
  cmd4.response = {0x00, 0x01, 0x86, 0xA0}; // 100000 ticks
  Position pos4 = CommandDecoder::read_pulse_count(cmd4);
  ASSERT_EQUAL(pos4.get_ticks(), 100000, "Decode 100000 ticks");

  // Test invalid data size
  Command cmd5(Commandtype::READ_PULSE_COUNT);
  cmd5.response = {0x00, 0x00}; // Only 2 bytes
  Position pos5 = CommandDecoder::read_pulse_count(cmd5);
  ASSERT_EQUAL(pos5.get_ticks(), 0, "Invalid data returns default Position");
}

/**
 * Test read_encoder_addition decoder
 * Hardware Doc: Section 8.1.2 - Read encoder value (addition)
 * Register: 0x31, Function: 0x04, Response: 6 bytes (int48_t)
 */
void test_read_encoder_addition()
{
  std::cout << "\n=== read_encoder_addition Tests (Register 0x31) ===" << std::endl;

  // Test positive encoder value
  Command cmd1(Commandtype::READ_ENCODER_ADDITION);
  cmd1.response = {0x00, 0x00, 0x00, 0x00, 0x03, 0xE8}; // 1000
  Position pos1 = CommandDecoder::read_encoder_addition(cmd1);
  ASSERT_EQUAL(pos1.get_ticks(), 1000, "Decode +1000 encoder ticks");

  // Test negative encoder value (48-bit sign extension)
  Command cmd2(Commandtype::READ_ENCODER_ADDITION);
  cmd2.response = {0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x18}; // -1000
  Position pos2 = CommandDecoder::read_encoder_addition(cmd2);
  ASSERT_EQUAL(pos2.get_ticks(), -1000, "Decode -1000 encoder ticks (48-bit)");

  // Test zero
  Command cmd3(Commandtype::READ_ENCODER_ADDITION);
  cmd3.response = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  Position pos3 = CommandDecoder::read_encoder_addition(cmd3);
  ASSERT_EQUAL(pos3.get_ticks(), 0, "Decode 0 encoder ticks");

  // Test invalid data size
  Command cmd4(Commandtype::READ_ENCODER_ADDITION);
  cmd4.response = {0x00, 0x00, 0x00, 0x00}; // Only 4 bytes
  Position pos4 = CommandDecoder::read_encoder_addition(cmd4);
  ASSERT_EQUAL(pos4.get_ticks(), 0, "Invalid data returns default Position");
}

/**
 * Test read_angle_error decoder
 * Hardware Doc: Section 8.1.6 - Read the error of angle
 * Register: 0x39, Function: 0x04, Response: 4 bytes (int32_t)
 */
void test_read_angle_error()
{
  std::cout << "\n=== read_angle_error Tests (Register 0x39) ===" << std::endl;

  // Test positive error
  Command cmd1(Commandtype::READ_ANGLE_ERROR);
  cmd1.response = {0x00, 0x00, 0x00, 0x64}; // +100 error
  Position err1 = CommandDecoder::read_angle_error(cmd1);
  ASSERT_EQUAL(err1.get_ticks(), 100, "Decode +100 angle error");

  // Test negative error
  Command cmd2(Commandtype::READ_ANGLE_ERROR);
  cmd2.response = {0xFF, 0xFF, 0xFF, 0x9C}; // -100 error
  Position err2 = CommandDecoder::read_angle_error(cmd2);
  ASSERT_EQUAL(err2.get_ticks(), -100, "Decode -100 angle error");

  // Test zero error
  Command cmd3(Commandtype::READ_ANGLE_ERROR);
  cmd3.response = {0x00, 0x00, 0x00, 0x00};
  Position err3 = CommandDecoder::read_angle_error(cmd3);
  ASSERT_EQUAL(err3.get_ticks(), 0, "Decode 0 angle error");
}

/**
 * Test read_enable_status decoder
 * Hardware Doc: Section 8.1.7 - Read the En pins status
 * Register: 0x3A, Function: 0x04, Response: 2 bytes [0x00][status]
 * status = 1: Enabled, status = 0: Disabled
 */
void test_read_enable_status()
{
  std::cout << "\n=== read_enable_status Tests (Register 0x3A) ===" << std::endl;

  // Test enabled status
  std::vector<uint8_t> data1 = {0x00, 0x01}; // Enabled
  bool enabled1 = CommandDecoder::read_enable_status(data1);
  ASSERT_TRUE(enabled1, "Decode enabled status (0x01)");

  // Test disabled status
  std::vector<uint8_t> data2 = {0x00, 0x00}; // Disabled
  bool enabled2 = CommandDecoder::read_enable_status(data2);
  ASSERT_TRUE(!enabled2, "Decode disabled status (0x00)");

  // Test invalid data size
  std::vector<uint8_t> data3 = {0x00}; // Only 1 byte
  bool enabled3 = CommandDecoder::read_enable_status(data3);
  ASSERT_TRUE(!enabled3, "Invalid data returns false");
}

/**
 * Test read_io_port_status decoder
 * Hardware Doc: Section 8.1.5 - Read the IO Ports status
 * Register: 0x34, Function: 0x04, Response: 2 bytes [0x00][status]
 * status bits: bit0=IN_1, bit1=IN_2, bit2=OUT_1, bit3=OUT_2
 * Note: After limit remapping, IN_1 maps to En, IN_2 maps to Dir
 */
void test_read_io_port_status()
{
  std::cout << "\n=== read_io_port_status Tests (Register 0x34) ===" << std::endl;

  // Test all ports LOW
  std::vector<uint8_t> data1 = {0x00, 0x00}; // 0b0000
  auto io1 = CommandDecoder::read_io_port_status(data1);
  ASSERT_TRUE(!io1.in1 && !io1.in2 && !io1.out1 && !io1.out2, "All ports LOW");

  // Test IN_1 HIGH (bit0 = 1)
  std::vector<uint8_t> data2 = {0x00, 0x01}; // 0b0001
  auto io2 = CommandDecoder::read_io_port_status(data2);
  ASSERT_TRUE(io2.in1 && !io2.in2 && !io2.out1 && !io2.out2, "IN_1 HIGH only");

  // Test IN_2 HIGH (bit1 = 1)
  std::vector<uint8_t> data3 = {0x00, 0x02}; // 0b0010
  auto io3 = CommandDecoder::read_io_port_status(data3);
  ASSERT_TRUE(!io3.in1 && io3.in2 && !io3.out1 && !io3.out2, "IN_2 HIGH only");

  // Test OUT_1 HIGH (bit2 = 1)
  std::vector<uint8_t> data4 = {0x00, 0x04}; // 0b0100
  auto io4 = CommandDecoder::read_io_port_status(data4);
  ASSERT_TRUE(!io4.in1 && !io4.in2 && io4.out1 && !io4.out2, "OUT_1 HIGH only");

  // Test OUT_2 HIGH (bit3 = 1)
  std::vector<uint8_t> data5 = {0x00, 0x08}; // 0b1000
  auto io5 = CommandDecoder::read_io_port_status(data5);
  ASSERT_TRUE(!io5.in1 && !io5.in2 && !io5.out1 && io5.out2, "OUT_2 HIGH only");

  // Test all ports HIGH
  std::vector<uint8_t> data6 = {0x00, 0x0F}; // 0b1111
  auto io6 = CommandDecoder::read_io_port_status(data6);
  ASSERT_TRUE(io6.in1 && io6.in2 && io6.out1 && io6.out2, "All ports HIGH");

  // Test invalid data size
  std::vector<uint8_t> data7 = {0x00}; // Only 1 byte
  auto io7 = CommandDecoder::read_io_port_status(data7);
  ASSERT_TRUE(!io7.in1 && !io7.in2 && !io7.out1 && !io7.out2, "Invalid data returns all false");
}

/**
 * Test read_zeroing_status decoder
 * Hardware Doc: Section 8.1.8 - Read the go back to zero status
 * Register: 0x3B, Function: 0x04, Response: 2 bytes [0x00][status]
 * status = 0: going to zero, status = 1: success, status = 2: fail
 */
void test_read_zeroing_status()
{
  std::cout << "\n=== read_zeroing_status Tests (Register 0x3B) ===" << std::endl;

  // Test GOING_TO_ZERO state
  std::vector<uint8_t> data1 = {0x00, 0x00}; // going to zero
  auto zs1 = CommandDecoder::read_zeroing_status(data1);
  ASSERT_TRUE(zs1.state == CommandDecoder::ZeroingStatus::GOING_TO_ZERO, "Decode GOING_TO_ZERO (0)");

  // Test SUCCESS state
  std::vector<uint8_t> data2 = {0x00, 0x01}; // success
  auto zs2 = CommandDecoder::read_zeroing_status(data2);
  ASSERT_TRUE(zs2.state == CommandDecoder::ZeroingStatus::SUCCESS, "Decode SUCCESS (1)");

  // Test FAILED state
  std::vector<uint8_t> data3 = {0x00, 0x02}; // fail
  auto zs3 = CommandDecoder::read_zeroing_status(data3);
  ASSERT_TRUE(zs3.state == CommandDecoder::ZeroingStatus::FAILED, "Decode FAILED (2)");

  // Test invalid status value (should default to FAILED)
  std::vector<uint8_t> data4 = {0x00, 0x03}; // invalid
  auto zs4 = CommandDecoder::read_zeroing_status(data4);
  ASSERT_TRUE(zs4.state == CommandDecoder::ZeroingStatus::FAILED, "Invalid status defaults to FAILED");

  // Test invalid data size
  std::vector<uint8_t> data5 = {0x00}; // Only 1 byte
  auto zs5 = CommandDecoder::read_zeroing_status(data5);
  ASSERT_TRUE(zs5.state == CommandDecoder::ZeroingStatus::GOING_TO_ZERO, "Invalid data returns default state");
}

/**
 * Test read_detailed_motor_status decoder
 * Hardware Doc: Section 8.1.10 - Read the motor status
 * Register: 0xF1, Function: 0x04, Response: 2 bytes [0x00][status]
 * status: 0=FAIL, 1=STOP, 2=SPEED_UP, 3=SPEED_DOWN, 4=FULL_SPEED, 5=HOMING, 6=CALIBRATING
 */
void test_read_detailed_motor_status()
{
  std::cout << "\n=== read_detailed_motor_status Tests (Register 0xF1) ===" << std::endl;

  // Test FAIL state
  std::vector<uint8_t> data0 = {0x00, 0x00}; // read fail
  auto dms0 = CommandDecoder::read_detailed_motor_status(data0);
  ASSERT_TRUE(dms0.state == CommandDecoder::DetailedMotorStatus::FAIL, "Decode FAIL (0)");

  // Test STOP state
  std::vector<uint8_t> data1 = {0x00, 0x01}; // motor stop
  auto dms1 = CommandDecoder::read_detailed_motor_status(data1);
  ASSERT_TRUE(dms1.state == CommandDecoder::DetailedMotorStatus::STOP, "Decode STOP (1)");

  // Test SPEED_UP state
  std::vector<uint8_t> data2 = {0x00, 0x02}; // motor speed up
  auto dms2 = CommandDecoder::read_detailed_motor_status(data2);
  ASSERT_TRUE(dms2.state == CommandDecoder::DetailedMotorStatus::SPEED_UP, "Decode SPEED_UP (2)");

  // Test SPEED_DOWN state
  std::vector<uint8_t> data3 = {0x00, 0x03}; // motor speed down
  auto dms3 = CommandDecoder::read_detailed_motor_status(data3);
  ASSERT_TRUE(dms3.state == CommandDecoder::DetailedMotorStatus::SPEED_DOWN, "Decode SPEED_DOWN (3)");

  // Test FULL_SPEED state
  std::vector<uint8_t> data4 = {0x00, 0x04}; // motor full speed
  auto dms4 = CommandDecoder::read_detailed_motor_status(data4);
  ASSERT_TRUE(dms4.state == CommandDecoder::DetailedMotorStatus::FULL_SPEED, "Decode FULL_SPEED (4)");

  // Test HOMING state
  std::vector<uint8_t> data5 = {0x00, 0x05}; // motor is homing
  auto dms5 = CommandDecoder::read_detailed_motor_status(data5);
  ASSERT_TRUE(dms5.state == CommandDecoder::DetailedMotorStatus::HOMING, "Decode HOMING (5)");

  // Test CALIBRATING state
  std::vector<uint8_t> data6 = {0x00, 0x06}; // motor is Cal...
  auto dms6 = CommandDecoder::read_detailed_motor_status(data6);
  ASSERT_TRUE(dms6.state == CommandDecoder::DetailedMotorStatus::CALIBRATING, "Decode CALIBRATING (6)");

  // Test invalid status (should default to FAIL)
  std::vector<uint8_t> data7 = {0x00, 0xFF}; // invalid
  auto dms7 = CommandDecoder::read_detailed_motor_status(data7);
  ASSERT_TRUE(dms7.state == CommandDecoder::DetailedMotorStatus::FAIL, "Invalid status defaults to FAIL");

  // Test invalid data size
  std::vector<uint8_t> data8 = {0x00}; // Only 1 byte
  auto dms8 = CommandDecoder::read_detailed_motor_status(data8);
  ASSERT_TRUE(dms8.state == CommandDecoder::DetailedMotorStatus::FAIL, "Invalid data returns default state");
}

/**
 * Test read_all_config decoder
 * Hardware Doc: Section 8.2.10 - Read all configuration parameters
 * Register: 0x1147, Function: 0x04, Response: 38 bytes (19 registers)
 */
void test_read_all_config()
{
  std::cout << "\n=== read_all_config Tests (Register 0x1147) ===" << std::endl;

  // Test valid configuration data (38 bytes)
  Command cmd1(Commandtype::READ_ALL_CONFIG);
  cmd1.response = {
      // REG1: Mode [mode][reserved]
      0x03, 0x00, // SR_OPEN mode
      // REG2: Hold current [hw_hold][reserved]
      0x04, 0x00, // 50% (hw_hold=4 → (4+1)*10 = 50%)
      // REG3: Work current [hi][lo]
      0x07, 0xD0, // 2000 mA
      // REG4: Subdivision [subdivision][reserved]
      0x10, 0x00, // 16 microsteps
      // REG5: En + Dir [en_pin_active][shaft_reversed]
      0x00, 0x00, // LOW, not reversed
      // REG6: AutoSDD + Protect [auto_screen_off][protect_enable]
      0x01, 0x00, // enabled, no protection
      // REG7: Mplyer + NULL [mplyer][reserved]
      0x00, 0x00, // mplyer=0
      // REG8: Baud rate + Slave address [baud_rate][slave_address]
      0x01, 0x01, // baud=1, slave=1
      // REG9: Group address + Respond/Active [group_address][respond_active]
      0x00, 0x01, // group=0, respond=enabled
      // REG10: MODBUS + Key lock [modbus_enable][key_lock]
      0x01, 0x00, // MODBUS enabled, keys unlocked
      // REG11-13: Homing parameters [trigger][direction][speed_hi][speed_lo][null][endlimit]
      0x00, 0x00, 0x00, 0x64, 0x00, 0x01, // LOW trigger, CW, 100 RPM, endlimit enabled
      // REG14-16: No-limit homing [reverse_angle(4)][mode(2)][current_ma(2)]
      0x00, 0x00, 0x07, 0xD0, 0x00, 0x00, 0x03, 0xE8, // 2000 ticks, disabled, 1000 mA
      // REG17: Remap + NULL [null][limit_port_remap]
      0x00, 0x00, // no remap
      // REG18-19: 0_Mode parameters [zero_mode][zero_task][zero_speed][zero_direction]
      0x00, 0x00, 0x02, 0x00 // disabled, clean, medium speed, CW
  };

  auto config1 = CommandDecoder::read_all_config(cmd1);
  ASSERT_EQUAL(static_cast<uint8_t>(config1.mode), 0x03, "Decode control mode (SR_OPEN)");
  ASSERT_EQUAL(config1.holding_current_percent, 50, "Decode holding current (50%)");
  ASSERT_EQUAL(config1.working_current_ma, 2000, "Decode working current (2000 mA)");
  ASSERT_EQUAL(config1.subdivision, 16, "Decode subdivision (16 microsteps)");
  ASSERT_EQUAL(static_cast<uint8_t>(config1.en_pin_active), 0, "Decode EN pin active (LOW)");
  ASSERT_TRUE(!config1.shaft_reversed, "Decode shaft reversed (false)");
  ASSERT_TRUE(config1.auto_screen_off, "Decode auto screen off (true)");
  ASSERT_EQUAL(config1.protect_enable, 0, "Decode protection enable");
  ASSERT_EQUAL(config1.baud_rate, 1, "Decode baud rate");
  ASSERT_EQUAL(config1.slave_address, 1, "Decode slave address");
  ASSERT_TRUE(config1.modbus_enable, "Decode MODBUS enable (true)");
  ASSERT_TRUE(!config1.key_lock, "Decode key lock (false)");
  ASSERT_EQUAL(static_cast<uint8_t>(config1.homing_trigger), 0, "Decode homing trigger (LOW)");
  ASSERT_EQUAL(static_cast<uint8_t>(config1.homing_direction), 0, "Decode homing direction (CW)");
  ASSERT_EQUAL(config1.homing_speed_rpm, 100, "Decode homing speed (100 RPM)");
  ASSERT_TRUE(config1.endlimit_enable, "Decode endlimit enable (true)");
  ASSERT_EQUAL(config1.nolimit_reverse_angle_ticks, 2000u, "Decode nolimit reverse angle");
  ASSERT_TRUE(!config1.nolimit_mode, "Decode nolimit mode (false)");
  ASSERT_EQUAL(config1.nolimit_current_ma, 1000, "Decode nolimit current (1000 mA)");
  ASSERT_TRUE(!config1.limit_port_remap, "Decode limit port remap (false)");
  ASSERT_EQUAL(static_cast<uint8_t>(config1.zero_mode), 0, "Decode zero mode");
  ASSERT_EQUAL(static_cast<uint8_t>(config1.zero_speed), 2, "Decode zero speed");

  // Test invalid data size
  Command cmd2(Commandtype::READ_ALL_CONFIG);
  cmd2.response = {0x00, 0x00}; // Only 2 bytes
  auto config2 = CommandDecoder::read_all_config(cmd2);
  ASSERT_EQUAL(static_cast<uint8_t>(config2.mode), 0x03, "Invalid data returns default config");

  // Test wrong command type
  Command cmd3(Commandtype::READ_CURRENT_SPEED); // Wrong type
  cmd3.response.resize(38, 0x00); // Fill with zeros
  auto config3 = CommandDecoder::read_all_config(cmd3);
  ASSERT_EQUAL(static_cast<uint8_t>(config3.mode), 0x03, "Wrong command type returns default config");

  // Test maximum holding current (90%)
  Command cmd4(Commandtype::READ_ALL_CONFIG);
  cmd4.response = cmd1.response; // Copy from cmd1
  cmd4.response[2] = 0x08; // hw_hold = 8 → 90%
  auto config4 = CommandDecoder::read_all_config(cmd4);
  ASSERT_EQUAL(config4.holding_current_percent, 90, "Decode maximum holding current (90%)");

  // Test SR_vFOC mode
  Command cmd5(Commandtype::READ_ALL_CONFIG);
  cmd5.response = cmd1.response; // Copy from cmd1
  cmd5.response[0] = 0x05; // SR_vFOC
  auto config5 = CommandDecoder::read_all_config(cmd5);
  ASSERT_EQUAL(static_cast<uint8_t>(config5.mode), 0x05, "Decode SR_vFOC mode");
}

/**
 * Test edge cases and boundary conditions
 */
void test_edge_cases()
{
  std::cout << "\n=== Edge Cases and Boundary Tests ===" << std::endl;

  // Test maximum positive int16_t speed (clamped to hardware limit)
  Command cmd_max_speed(Commandtype::READ_CURRENT_SPEED);
  cmd_max_speed.response = {0x7F, 0xFF}; // 32767 RPM (max int16_t)
  Speed speed_max = CommandDecoder::read_current_speed(cmd_max_speed);
  ASSERT_EQUAL(speed_max.rpm(), 3000, "Max positive speed clamped to 3000 RPM");

  // Test minimum negative int16_t speed (clamped to hardware limit)
  Command cmd_min_speed(Commandtype::READ_CURRENT_SPEED);
  cmd_min_speed.response = {0x80, 0x00}; // -32768 RPM (min int16_t)
  Speed speed_min = CommandDecoder::read_current_speed(cmd_min_speed);
  ASSERT_EQUAL(speed_min.rpm(), -3000, "Min negative speed clamped to -3000 RPM");

  // Test maximum positive int32_t position
  Command cmd_max_pos(Commandtype::READ_PULSE_COUNT);
  cmd_max_pos.response = {0x7F, 0xFF, 0xFF, 0xFF}; // 2147483647 (max int32_t)
  Position pos_max = CommandDecoder::read_pulse_count(cmd_max_pos);
  ASSERT_EQUAL(pos_max.get_ticks(), 2147483647, "Max positive position");

  // Test minimum negative int32_t position
  Command cmd_min_pos(Commandtype::READ_PULSE_COUNT);
  cmd_min_pos.response = {0x80, 0x00, 0x00, 0x00}; // -2147483648 (min int32_t)
  Position pos_min = CommandDecoder::read_pulse_count(cmd_min_pos);
  ASSERT_EQUAL(pos_min.get_ticks(), -2147483648LL, "Min negative position");

  // Test empty response data
  Command cmd_empty(Commandtype::READ_CURRENT_SPEED);
  cmd_empty.response = {}; // Empty
  Speed speed_empty = CommandDecoder::read_current_speed(cmd_empty);
  ASSERT_EQUAL(speed_empty.rpm(), 0, "Empty response returns default");

  // Test oversized response data (should still work, using first bytes)
  Command cmd_oversized(Commandtype::READ_CURRENT_SPEED);
  cmd_oversized.response = {0x00, 0x64, 0xFF, 0xFF}; // Extra bytes ignored
  Speed speed_oversized = CommandDecoder::read_current_speed(cmd_oversized);
  ASSERT_EQUAL(speed_oversized.rpm(), 100, "Oversized response uses first bytes");
}

int main()
{
  std::cout << "========================================" << std::endl;
  std::cout << "CommandDecoder Unit Tests" << std::endl;
  std::cout << "Validated against MKS SERVO42D/57D_RS485 V1.0.6 Manual" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << std::endl;

  test_read_current_speed();
  test_read_pulse_count();
  test_read_encoder_addition();
  test_read_angle_error();
  test_read_enable_status();
  test_read_io_port_status();
  test_read_zeroing_status();
  test_read_detailed_motor_status();
  test_read_all_config();
  test_edge_cases();

  std::cout << "\n========================================" << std::endl;
  std::cout << "✅ All CommandDecoder Tests Passed (" << tests_passed << " assertions)!" << std::endl;
  std::cout << "========================================" << std::endl;

  return 0;
}
