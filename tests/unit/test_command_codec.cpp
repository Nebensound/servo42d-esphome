#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include "../../components/servoxxd/stepper/servoxxd_command_codec.h"

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

void test_encode_move_position_mode_2()
{
  std::cout << "\n=== encode_move_position_mode_2 Tests ===" << std::endl;

  // Test basic encoding
  auto data = ServoCommandCodec::encode_move_position_mode_2(1000, 100, 50);
  ASSERT_EQUAL(data.size(), 7u, "Payload size is 7 bytes");
  ASSERT_EQUAL(data[0], 0x00, "Speed MSB");
  ASSERT_EQUAL(data[1], 0x64, "Speed LSB = 100");
  ASSERT_EQUAL(data[2], 50, "Accel = 50");
  ASSERT_EQUAL(data[3], 0x00, "Position byte 0");
  ASSERT_EQUAL(data[4], 0x00, "Position byte 1");
  ASSERT_EQUAL(data[5], 0x03, "Position byte 2");
  ASSERT_EQUAL(data[6], 0xE8, "Position byte 3 = 1000");

  // Test negative position
  data = ServoCommandCodec::encode_move_position_mode_2(-1000, 200, 100);
  ASSERT_EQUAL(data.size(), 7u, "Negative position payload size");
  ASSERT_EQUAL(data[0], 0x00, "Speed MSB");
  ASSERT_EQUAL(data[1], 0xC8, "Speed LSB = 200");
  ASSERT_EQUAL(data[2], 100, "Accel = 100");
  ASSERT_EQUAL(data[3], 0xFF, "Negative position byte 0");
  ASSERT_EQUAL(data[4], 0xFF, "Negative position byte 1");
  ASSERT_EQUAL(data[5], 0xFC, "Negative position byte 2");
  ASSERT_EQUAL(data[6], 0x18, "Negative position byte 3");

  // Test zero values
  data = ServoCommandCodec::encode_move_position_mode_2(0, 0, 0);
  ASSERT_EQUAL(data.size(), 7u, "Zero values payload size");
  ASSERT_EQUAL(data[0], 0x00, "Speed MSB zero");
  ASSERT_EQUAL(data[1], 0x00, "Speed LSB zero");
  ASSERT_EQUAL(data[2], 0x00, "Accel zero");
  ASSERT_EQUAL(data[3], 0x00, "Position byte 0 zero");
  ASSERT_EQUAL(data[6], 0x00, "Position byte 3 zero");

  // Test maximum values
  data = ServoCommandCodec::encode_move_position_mode_2(INT32_MAX, UINT16_MAX, 255);
  ASSERT_EQUAL(data.size(), 7u, "Max values payload size");
  ASSERT_EQUAL(data[0], 0xFF, "Max speed MSB");
  ASSERT_EQUAL(data[1], 0xFF, "Max speed LSB");
  ASSERT_EQUAL(data[2], 255, "Max accel");
  ASSERT_EQUAL(data[3], 0x7F, "Max position byte 0");
  ASSERT_EQUAL(data[4], 0xFF, "Max position byte 1");
  ASSERT_EQUAL(data[5], 0xFF, "Max position byte 2");
  ASSERT_EQUAL(data[6], 0xFF, "Max position byte 3");
}

void test_encode_stop_position_mode_2()
{
  std::cout << "\n=== encode_stop_position_mode_2 Tests ===" << std::endl;

  auto data = ServoCommandCodec::encode_stop_position_mode_2(128);
  ASSERT_EQUAL(data.size(), 1u, "Stop payload size is 1 byte");
  ASSERT_EQUAL(data[0], 128, "Decel value = 128");

  data = ServoCommandCodec::encode_stop_position_mode_2(0);
  ASSERT_EQUAL(data[0], 0, "Zero decel (instant stop)");

  data = ServoCommandCodec::encode_stop_position_mode_2(255);
  ASSERT_EQUAL(data[0], 255, "Max decel value");
}

void test_encode_move_speed_mode()
{
  std::cout << "\n=== encode_move_speed_mode Tests ===" << std::endl;

  auto data = ServoCommandCodec::encode_move_speed_mode(1000, 50, 1);
  ASSERT_EQUAL(data.size(), 4u, "Speed mode payload size is 4 bytes");
  ASSERT_EQUAL(data[0], 1, "Direction = 1 (forward)");
  ASSERT_EQUAL(data[1], 0x03, "Speed MSB");
  ASSERT_EQUAL(data[2], 0xE8, "Speed LSB = 1000");
  ASSERT_EQUAL(data[3], 50, "Accel = 50");

  data = ServoCommandCodec::encode_move_speed_mode(500, 100, 0);
  ASSERT_EQUAL(data[0], 0, "Direction = 0 (reverse)");
  ASSERT_EQUAL(data[1], 0x01, "Speed MSB");
  ASSERT_EQUAL(data[2], 0xF4, "Speed LSB = 500");
}

void test_encode_config_commands()
{
  std::cout << "\n=== Configuration Encoding Tests ===" << std::endl;

  // Test working current
  auto data = ServoCommandCodec::encode_set_working_current(2000);
  ASSERT_EQUAL(data.size(), 2u, "Working current payload size");
  ASSERT_EQUAL(data[0], 0x07, "Current MSB (2000 mA)");
  ASSERT_EQUAL(data[1], 0xD0, "Current LSB");

  data = ServoCommandCodec::encode_set_working_current(500);
  ASSERT_EQUAL(data[0], 0x01, "Current MSB (500 mA)");
  ASSERT_EQUAL(data[1], 0xF4, "Current LSB");

  // Test subdivision (microsteps)
  data = ServoCommandCodec::encode_set_subdivision(16);
  ASSERT_EQUAL(data.size(), 1u, "Subdivision payload size");
  ASSERT_EQUAL(data[0], 16, "Microsteps = 16");

  data = ServoCommandCodec::encode_set_subdivision(256);
  ASSERT_EQUAL(data[0], 0, "Microsteps = 256 (wraps to 0)");

  // Test enable motor
  data = ServoCommandCodec::encode_enable_motor(true);
  ASSERT_EQUAL(data.size(), 1u, "Enable motor payload size");
  ASSERT_EQUAL(data[0], 0x01, "Enable = true");

  data = ServoCommandCodec::encode_enable_motor(false);
  ASSERT_EQUAL(data[0], 0x00, "Enable = false");
}

void test_decode_current_speed()
{
  std::cout << "\n=== decode_current_speed Tests ===" << std::endl;

  // Positive speed
  std::vector<uint8_t> data = {0x00, 0x64}; // 100 RPM
  int16_t speed = ServoCommandCodec::decode_current_speed(data);
  ASSERT_EQUAL(speed, 100, "Decode 100 RPM");

  // Negative speed (two's complement)
  data = {0xFF, 0x9C}; // -100
  speed = ServoCommandCodec::decode_current_speed(data);
  ASSERT_EQUAL(speed, -100, "Decode -100 RPM");

  // Zero speed
  data = {0x00, 0x00};
  speed = ServoCommandCodec::decode_current_speed(data);
  ASSERT_EQUAL(speed, 0, "Decode 0 RPM");

  // Maximum positive
  data = {0x7F, 0xFF}; // INT16_MAX
  speed = ServoCommandCodec::decode_current_speed(data);
  ASSERT_EQUAL(speed, 32767, "Decode max positive speed");

  // Maximum negative
  data = {0x80, 0x00}; // INT16_MIN
  speed = ServoCommandCodec::decode_current_speed(data);
  ASSERT_EQUAL(speed, -32768, "Decode max negative speed");

  // Insufficient data
  data = {0x00};
  speed = ServoCommandCodec::decode_current_speed(data);
  ASSERT_EQUAL(speed, 0, "Insufficient data returns 0");

  // Empty data
  data.clear();
  speed = ServoCommandCodec::decode_current_speed(data);
  ASSERT_EQUAL(speed, 0, "Empty data returns 0");
}

void test_decode_pulse_count()
{
  std::cout << "\n=== decode_pulse_count Tests ===" << std::endl;

  // Positive count
  std::vector<uint8_t> data = {0x00, 0x00, 0x03, 0xE8}; // 1000
  int32_t count = ServoCommandCodec::decode_pulse_count(data);
  ASSERT_EQUAL(count, 1000, "Decode 1000 pulses");

  // Negative count
  data = {0xFF, 0xFF, 0xFC, 0x18}; // -1000
  count = ServoCommandCodec::decode_pulse_count(data);
  ASSERT_EQUAL(count, -1000, "Decode -1000 pulses");

  // Zero
  data = {0x00, 0x00, 0x00, 0x00};
  count = ServoCommandCodec::decode_pulse_count(data);
  ASSERT_EQUAL(count, 0, "Decode 0 pulses");

  // Large positive
  data = {0x00, 0x0F, 0x42, 0x40}; // 1,000,000
  count = ServoCommandCodec::decode_pulse_count(data);
  ASSERT_EQUAL(count, 1000000, "Decode 1,000,000 pulses");

  // Maximum positive
  data = {0x7F, 0xFF, 0xFF, 0xFF}; // INT32_MAX
  count = ServoCommandCodec::decode_pulse_count(data);
  ASSERT_EQUAL(count, INT32_MAX, "Decode INT32_MAX");

  // Insufficient data
  data = {0x00, 0x00, 0x03};
  count = ServoCommandCodec::decode_pulse_count(data);
  ASSERT_EQUAL(count, 0, "Insufficient data returns 0");
}

void test_decode_encoder_carry()
{
  std::cout << "\n=== decode_encoder_carry Tests ===" << std::endl;

  // Normal encoder value
  std::vector<uint8_t> data = {0x00, 0x00, 0x00, 0x05, 0x12, 0x34}; // carry=5, value=0x1234
  auto ev = ServoCommandCodec::decode_encoder_carry(data);
  ASSERT_EQUAL(ev.carry, 5, "Carry = 5");
  ASSERT_EQUAL(ev.value, 0x1234, "Value = 0x1234");

  // Zero values
  data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  ev = ServoCommandCodec::decode_encoder_carry(data);
  ASSERT_EQUAL(ev.carry, 0, "Zero carry");
  ASSERT_EQUAL(ev.value, 0, "Zero value");

  // Negative carry
  data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // carry=-1, value=0xFFFF
  ev = ServoCommandCodec::decode_encoder_carry(data);
  ASSERT_EQUAL(ev.carry, -1, "Negative carry");
  ASSERT_EQUAL(ev.value, 0xFFFF, "Max value");

  // Large carry
  data = {0x00, 0x00, 0x27, 0x10, 0x3F, 0xFF}; // carry=10000, value=16383
  ev = ServoCommandCodec::decode_encoder_carry(data);
  ASSERT_EQUAL(ev.carry, 10000, "Large carry");
  ASSERT_EQUAL(ev.value, 16383, "Max angle value");

  // Insufficient data
  data = {0x00, 0x00, 0x00, 0x05};
  ev = ServoCommandCodec::decode_encoder_carry(data);
  ASSERT_EQUAL(ev.carry, 0, "Insufficient data: carry = 0");
  ASSERT_EQUAL(ev.value, 0, "Insufficient data: value = 0");
}

void test_decode_motor_status()
{
  std::cout << "\n=== decode_motor_status Tests ===" << std::endl;

  // STOP state
  std::vector<uint8_t> data = {0x00};
  auto status = ServoCommandCodec::decode_motor_status(data);
  ASSERT_EQUAL(status.state, ServoCommandCodec::MotorStatus::STOP, "Status = STOP");

  // MOVING state
  data = {0x01};
  status = ServoCommandCodec::decode_motor_status(data);
  ASSERT_EQUAL(status.state, ServoCommandCodec::MotorStatus::MOVING, "Status = MOVING");

  // HOMING state
  data = {0x02};
  status = ServoCommandCodec::decode_motor_status(data);
  ASSERT_EQUAL(status.state, ServoCommandCodec::MotorStatus::HOMING, "Status = HOMING");

  // Unknown state (treated as HOMING)
  data = {0x03};
  status = ServoCommandCodec::decode_motor_status(data);
  ASSERT_EQUAL(status.state, ServoCommandCodec::MotorStatus::HOMING, "Unknown status → HOMING");

  // Empty data
  data.clear();
  status = ServoCommandCodec::decode_motor_status(data);
  ASSERT_EQUAL(status.state, ServoCommandCodec::MotorStatus::STOP, "Empty data → STOP");
}

void test_decode_protection_status()
{
  std::cout << "\n=== decode_protection_status Tests ===" << std::endl;

  // Protected state
  std::vector<uint8_t> data = {0x01};
  auto ps = ServoCommandCodec::decode_protection_status(data);
  ASSERT_TRUE(ps.protected_state, "Protected state = true");

  // Not protected
  data = {0x00};
  ps = ServoCommandCodec::decode_protection_status(data);
  ASSERT_TRUE(!ps.protected_state, "Protected state = false");

  // Non-zero value
  data = {0xFF};
  ps = ServoCommandCodec::decode_protection_status(data);
  ASSERT_TRUE(ps.protected_state, "0xFF → protected");

  // Empty data
  data.clear();
  ps = ServoCommandCodec::decode_protection_status(data);
  ASSERT_TRUE(!ps.protected_state, "Empty data → not protected");
}

void test_roundtrip_encoding()
{
  std::cout << "\n=== Round-Trip Encoding Tests ===" << std::endl;

  // Test move command encode → manual decode
  auto encoded = ServoCommandCodec::encode_move_position_mode_2(12345, 999, 128);
  int32_t pos = (static_cast<int32_t>(encoded[0]) << 24) |
                (static_cast<int32_t>(encoded[1]) << 16) |
                (static_cast<int32_t>(encoded[2]) << 8) |
                static_cast<int32_t>(encoded[3]);
  uint16_t spd = (static_cast<uint16_t>(encoded[4]) << 8) | encoded[5];
  uint8_t acc = encoded[6];

  ASSERT_EQUAL(pos, 12345, "Round-trip position");
  ASSERT_EQUAL(spd, 999, "Round-trip speed");
  ASSERT_EQUAL(acc, 128, "Round-trip accel");

  // Test current reading encode → decode
  std::vector<uint8_t> speed_data = {0x01, 0xF4}; // 500 RPM
  int16_t decoded_speed = ServoCommandCodec::decode_current_speed(speed_data);
  ASSERT_EQUAL(decoded_speed, 500, "Round-trip current speed");

  // Test pulse count encode → decode
  std::vector<uint8_t> pulse_data = {0x00, 0x01, 0x86, 0xA0}; // 100,000
  int32_t decoded_pulses = ServoCommandCodec::decode_pulse_count(pulse_data);
  ASSERT_EQUAL(decoded_pulses, 100000, "Round-trip pulse count");
}

int main()
{
  std::cout << "========================================" << std::endl;
  std::cout << "ServoCommandCodec Unit Tests" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << std::endl;

  test_encode_move_position_mode_2();
  test_encode_stop_position_mode_2();
  test_encode_move_speed_mode();
  test_encode_config_commands();
  test_decode_current_speed();
  test_decode_pulse_count();
  test_decode_encoder_carry();
  test_decode_motor_status();
  test_decode_protection_status();
  test_roundtrip_encoding();

  std::cout << "\n========================================" << std::endl;
  std::cout << "✅ All CommandCodec Tests Passed (" << tests_passed << " assertions)!" << std::endl;
  std::cout << "========================================" << std::endl;

  return 0;
}
