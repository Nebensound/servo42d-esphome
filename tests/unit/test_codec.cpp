/**
 * @file test_codec.cpp
 * @brief Unit tests for ServoCommandCodec encode/decode methods
 *
 * Tests verify:
 * - Encode methods produce correct byte sequences
 * - Decode methods parse bytes correctly
 * - Roundtrip encode/decode preserves values
 * - Error handling for invalid input
 * - Big-endian byte ordering
 */

#include "servoxxd_codec.h"
#include "servoxxd.h"
#include "servoxxd_position.h"
#include "servoxxd_speed.h"
#include "servoxxd_acceleration.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <iomanip>

using namespace esphome::servoxxd;

// Tolerance for float comparisons
constexpr float EPSILON = 0.01f;

bool float_eq(float a, float b)
{
  return std::abs(a - b) < EPSILON;
}

// Mock ServoXxd for testing
class MockServoXxd : public ServoXxd
{
public:
  explicit MockServoXxd(float steps_per_rev, uint16_t microsteps = 16)
      : steps_per_rev_(steps_per_rev), microsteps_(microsteps) {}

  float get_steps_per_revolution() const override { return steps_per_rev_; }
  uint16_t get_microstepping() const override { return microsteps_; }

private:
  float steps_per_rev_;
  uint16_t microsteps_;
};

// Helper to print bytes
void print_bytes(const std::vector<uint8_t> &bytes)
{
  std::cout << "  Bytes: [";
  for (size_t i = 0; i < bytes.size(); ++i)
  {
    std::cout << "0x" << std::hex << std::setfill('0') << std::setw(2)
              << static_cast<int>(bytes[i]) << std::dec;
    if (i < bytes.size() - 1)
      std::cout << ", ";
  }
  std::cout << "]" << std::endl;
}

// ============================================================================
// Encoder Tests
// ============================================================================

void test_encode_move_position_mode_2()
{
  std::cout << "Testing encode_move_position_mode_2..." << std::endl;

  // Test: Position=1000, Speed=100, Accel=50, Direction=CW
  auto bytes = ServoCommandCodec::encode_move_position_mode_2(1000, 100, 50, Direction::CW);

  assert(bytes.size() == 8);
  print_bytes(bytes);

  // Byte 0: Direction (CW=0)
  assert(bytes[0] == 0x00);

  // Bytes 1-2: Speed=100 (0x0064, big-endian)
  assert(bytes[1] == 0x00);
  assert(bytes[2] == 0x64);

  // Byte 3: Accel=50
  assert(bytes[3] == 50);

  // Bytes 4-7: Position=1000 (0x000003E8, big-endian)
  assert(bytes[4] == 0x00);
  assert(bytes[5] == 0x00);
  assert(bytes[6] == 0x03);
  assert(bytes[7] == 0xE8);

  std::cout << "  ✓ Positive position encoded correctly" << std::endl;

  // Test negative position
  bytes = ServoCommandCodec::encode_move_position_mode_2(-1000, 100, 50, Direction::CCW);
  assert(bytes.size() == 8);
  print_bytes(bytes);

  // Byte 0: Direction (CCW=1)
  assert(bytes[0] == 0x01);

  // Bytes 4-7: Position=-1000 (0xFFFFFC18 in two's complement, big-endian)
  assert(bytes[4] == 0xFF);
  assert(bytes[5] == 0xFF);
  assert(bytes[6] == 0xFC);
  assert(bytes[7] == 0x18);

  std::cout << "  ✓ Negative position encoded correctly" << std::endl;
}

void test_encode_move_position_mode_2_validation()
{
  std::cout << "Testing encode_move_position_mode_2 validation..." << std::endl;

  // Test speed validation (> 3000 should fail)
  auto bytes = ServoCommandCodec::encode_move_position_mode_2(1000, 3001, 50, Direction::CW);
  assert(bytes.empty());

  std::cout << "  ✓ Speed > 3000 returns empty" << std::endl;

  // Valid speed should work
  bytes = ServoCommandCodec::encode_move_position_mode_2(1000, 3000, 50, Direction::CW);
  assert(bytes.size() == 8);

  std::cout << "  ✓ Speed = 3000 is valid" << std::endl;
}

void test_encode_move_position_mode_3()
{
  std::cout << "Testing encode_move_position_mode_3..." << std::endl;

  MockServoXxd mock(3200.0f);

  // Test: 180 degrees, 100 RPM, 50 acc, absolute
  Position pos(180.0f, PositionUnit::DEGREES, &mock);
  Speed spd(100.0f, SpeedUnit::RPM, &mock);
  Acceleration acc(50.0f, AccelerationUnit::RPM_PER_SEC, &mock);

  auto bytes = ServoCommandCodec::encode_move_position_mode_3(pos, spd, acc, true);

  assert(bytes.size() == 10);
  print_bytes(bytes);

  // Byte 0: Direction
  assert(bytes[0] == 0x00); // CW for positive RPM

  // Bytes 1-2: Speed (100 = 0x0064)
  assert(bytes[1] == 0x00);
  assert(bytes[2] == 0x64);

  // Byte 3: Acceleration (hardware value from Acceleration class)
  // 50 RPM/s converts to hardware acc value
  std::cout << "  Acceleration byte: 0x" << std::hex << static_cast<int>(bytes[3]) << std::dec << std::endl;

  // Byte 9: Absolute flag
  assert(bytes[9] == 0x01);

  std::cout << "  ✓ Position mode 3 encoded correctly" << std::endl;

  // Test relative mode
  bytes = ServoCommandCodec::encode_move_position_mode_3(pos, spd, acc, false);
  assert(bytes[9] == 0x00);

  std::cout << "  ✓ Relative flag = 0x00" << std::endl;
}

void test_encode_move_speed_mode()
{
  std::cout << "Testing encode_move_speed_mode..." << std::endl;

  // Test: Speed=200, Accel=100, Direction=CCW
  auto bytes = ServoCommandCodec::encode_move_speed_mode(200, 100, Direction::CCW);

  assert(bytes.size() == 4);
  print_bytes(bytes);

  // Byte 0: Direction (CCW=1)
  assert(bytes[0] == 0x01);

  // Bytes 1-2: Speed=200 (0x00C8, big-endian)
  assert(bytes[1] == 0x00);
  assert(bytes[2] == 0xC8);

  // Byte 3: Accel=100
  assert(bytes[3] == 100);

  std::cout << "  ✓ Speed mode encoded correctly" << std::endl;

  // Test validation
  bytes = ServoCommandCodec::encode_move_speed_mode(3001, 100, Direction::CW);
  assert(bytes.empty());

  std::cout << "  ✓ Speed > 3000 returns empty" << std::endl;
}

void test_encode_stop_position_mode_2()
{
  std::cout << "Testing encode_stop_position_mode_2..." << std::endl;

  auto bytes = ServoCommandCodec::encode_stop_position_mode_2(75);

  assert(bytes.size() == 8);
  print_bytes(bytes);

  // Position should be 0
  assert(bytes[4] == 0x00);
  assert(bytes[5] == 0x00);
  assert(bytes[6] == 0x00);
  assert(bytes[7] == 0x00);

  // Speed should be 0
  assert(bytes[1] == 0x00);
  assert(bytes[2] == 0x00);

  // Decel should be 75
  assert(bytes[3] == 75);

  std::cout << "  ✓ Stop command encoded correctly" << std::endl;
}

void test_encode_emergency_stop()
{
  std::cout << "Testing encode_emergency_stop..." << std::endl;

  auto bytes = ServoCommandCodec::encode_emergency_stop();

  assert(bytes.size() == 1);
  assert(bytes[0] == 0x01);

  std::cout << "  ✓ Emergency stop = [0x01]" << std::endl;
}

void test_encode_enable_motor()
{
  std::cout << "Testing encode_enable_motor..." << std::endl;

  auto enable = ServoCommandCodec::encode_enable_motor(true);
  assert(enable.size() == 1);
  assert(enable[0] == 0x01);

  std::cout << "  ✓ Enable = [0x01]" << std::endl;

  auto disable = ServoCommandCodec::encode_enable_motor(false);
  assert(disable.size() == 1);
  assert(disable[0] == 0x00);

  std::cout << "  ✓ Disable = [0x00]" << std::endl;
}

void test_encode_set_working_current()
{
  std::cout << "Testing encode_set_working_current..." << std::endl;

  // Test: 1500mA = 0x05DC
  auto bytes = ServoCommandCodec::encode_set_working_current(1500);

  assert(bytes.size() == 2);
  print_bytes(bytes);

  assert(bytes[0] == 0x05);
  assert(bytes[1] == 0xDC);

  std::cout << "  ✓ 1500mA encoded correctly" << std::endl;

  // Test validation (> 5000 should fail)
  bytes = ServoCommandCodec::encode_set_working_current(5001);
  assert(bytes.empty());

  std::cout << "  ✓ Current > 5000mA returns empty" << std::endl;
}

void test_encode_set_subdivision()
{
  std::cout << "Testing encode_set_subdivision..." << std::endl;

  // Test: 16 microsteps
  auto bytes = ServoCommandCodec::encode_set_subdivision(16);
  assert(bytes.size() == 1);
  assert(bytes[0] == 16);

  std::cout << "  ✓ 16 microsteps = [0x10]" << std::endl;

  // Test: 256 microsteps (encoded as 0)
  bytes = ServoCommandCodec::encode_set_subdivision(256);
  assert(bytes.size() == 1);
  assert(bytes[0] == 0x00);

  std::cout << "  ✓ 256 microsteps = [0x00]" << std::endl;

  // Test: 0 is invalid
  bytes = ServoCommandCodec::encode_set_subdivision(0);
  assert(bytes.empty());

  std::cout << "  ✓ 0 microsteps returns empty" << std::endl;
}

void test_encode_start_homing()
{
  std::cout << "Testing encode_start_homing..." << std::endl;

  auto bytes = ServoCommandCodec::encode_start_homing(Direction::CW, 2);

  assert(bytes.size() == 2);
  assert(bytes[0] == 0x00); // CW
  assert(bytes[1] == 0x02); // Speed level 2

  std::cout << "  ✓ Homing CW, speed 2 encoded correctly" << std::endl;

  bytes = ServoCommandCodec::encode_start_homing(Direction::CCW, 4);
  assert(bytes[0] == 0x01); // CCW
  assert(bytes[1] == 0x04); // Speed level 4

  std::cout << "  ✓ Homing CCW, speed 4 encoded correctly" << std::endl;

  // Test clamping (level > 4 should be clamped to 4)
  bytes = ServoCommandCodec::encode_start_homing(Direction::CW, 10);
  assert(bytes[1] == 0x04);

  std::cout << "  ✓ Speed level clamped to 4" << std::endl;
}

// ============================================================================
// Decoder Tests
// ============================================================================

void test_decode_encoder_carry()
{
  std::cout << "Testing decode_encoder_carry..." << std::endl;

  // Test: carry=5, value=8192 (half revolution)
  std::vector<uint8_t> data = {0x00, 0x00, 0x00, 0x05, 0x20, 0x00};

  auto result = ServoCommandCodec::decode_encoder_carry(data);

  assert(result.carry == 5);
  assert(result.value == 8192);

  std::cout << "  ✓ Carry=5, Value=8192 decoded correctly" << std::endl;

  // Test negative carry
  data = {0xFF, 0xFF, 0xFF, 0xFB, 0x10, 0x00}; // carry=-5, value=4096

  result = ServoCommandCodec::decode_encoder_carry(data);

  assert(result.carry == -5);
  assert(result.value == 4096);

  std::cout << "  ✓ Negative carry decoded correctly" << std::endl;

  // Test insufficient data
  data = {0x00, 0x00, 0x00};
  result = ServoCommandCodec::decode_encoder_carry(data);

  assert(result.carry == 0);
  assert(result.value == 0);

  std::cout << "  ✓ Insufficient data returns default" << std::endl;
}

void test_decode_encoder_position()
{
  std::cout << "Testing decode_encoder_position..." << std::endl;

  MockServoXxd mock(3200.0f);

  // Test: 1 revolution = carry=1, value=0
  std::vector<uint8_t> data = {0x00, 0x00, 0x00, 0x01, 0x00, 0x00};

  Position pos = ServoCommandCodec::decode_encoder_position(data, &mock);

  assert(pos.revolutions() == 1);
  assert(pos.angle_ticks() == 0);
  assert(pos.get_ticks() == 16384);

  std::cout << "  ✓ 1 revolution decoded correctly" << std::endl;

  // Test: 1.5 revolutions = carry=1, value=8192
  data = {0x00, 0x00, 0x00, 0x01, 0x20, 0x00};

  pos = ServoCommandCodec::decode_encoder_position(data, &mock);

  assert(pos.revolutions() == 1);
  assert(pos.angle_ticks() == 8192);
  assert(pos.get_ticks() == 24576); // 16384 + 8192

  std::cout << "  ✓ 1.5 revolutions decoded correctly" << std::endl;
}

void test_decode_current_speed()
{
  std::cout << "Testing decode_current_speed..." << std::endl;

  // Test: 100 RPM
  std::vector<uint8_t> data = {0x00, 0x64};

  int16_t rpm = ServoCommandCodec::decode_current_speed(data);
  assert(rpm == 100);

  std::cout << "  ✓ 100 RPM decoded correctly" << std::endl;

  // Test: -100 RPM (0xFF9C)
  data = {0xFF, 0x9C};
  rpm = ServoCommandCodec::decode_current_speed(data);
  assert(rpm == -100);

  std::cout << "  ✓ -100 RPM decoded correctly" << std::endl;

  // Test insufficient data
  data = {0x00};
  rpm = ServoCommandCodec::decode_current_speed(data);
  assert(rpm == 0);

  std::cout << "  ✓ Insufficient data returns 0" << std::endl;
}

void test_decode_motor_speed()
{
  std::cout << "Testing decode_motor_speed..." << std::endl;

  MockServoXxd mock(3200.0f);

  std::vector<uint8_t> data = {0x00, 0x64}; // 100 RPM

  Speed speed = ServoCommandCodec::decode_motor_speed(data, &mock);

  assert(speed.get_rpm() == 100);

  std::cout << "  ✓ Speed object created correctly" << std::endl;
}

void test_decode_pulse_count()
{
  std::cout << "Testing decode_pulse_count..." << std::endl;

  // Test: 100000 pulses = 0x000186A0
  std::vector<uint8_t> data = {0x00, 0x01, 0x86, 0xA0};

  int32_t count = ServoCommandCodec::decode_pulse_count(data);
  assert(count == 100000);

  std::cout << "  ✓ 100000 pulses decoded correctly" << std::endl;

  // Test negative count
  data = {0xFF, 0xFE, 0x79, 0x60}; // -100000

  count = ServoCommandCodec::decode_pulse_count(data);
  assert(count == -100000);

  std::cout << "  ✓ -100000 pulses decoded correctly" << std::endl;
}

void test_decode_motor_status()
{
  std::cout << "Testing decode_motor_status..." << std::endl;

  // Test: State=MOVING(3), Enabled=1, Protected=0, Stall=0
  // Byte: 0b00001011 = 0x0B
  std::vector<uint8_t> data = {0x0B};

  auto status = ServoCommandCodec::decode_motor_status(data);

  assert(status.state == MotorStatus::State::FULL_SPEED);
  assert(status.enabled == true);
  assert(status.shaft_protected == false);
  assert(status.stall_flag == false);

  std::cout << "  ✓ Motor status decoded correctly" << std::endl;

  // Test with protection and stall flags
  // Byte: 0b00111001 = 0x39 (state=1, enabled=1, protected=1, stall=1)
  data = {0x39};
  status = ServoCommandCodec::decode_motor_status(data);

  assert(status.state == MotorStatus::State::ACCEL);
  assert(status.enabled == true);
  assert(status.shaft_protected == true);
  assert(status.stall_flag == true);

  std::cout << "  ✓ Flags decoded correctly" << std::endl;

  // Test empty data
  data = {};
  status = ServoCommandCodec::decode_motor_status(data);

  assert(status.state == MotorStatus::State::STOP);
  assert(status.enabled == false);

  std::cout << "  ✓ Empty data returns default" << std::endl;
}

void test_decode_protection_status()
{
  std::cout << "Testing decode_protection_status..." << std::endl;

  // Test: Protected=1, Code=5 (0b00001011 = 0x0B)
  std::vector<uint8_t> data = {0x0B};

  auto status = ServoCommandCodec::decode_protection_status(data);

  assert(status.protected_state == true);
  assert(status.protection_code == 5);

  std::cout << "  ✓ Protection status decoded correctly" << std::endl;

  // Test not protected
  data = {0x00};
  status = ServoCommandCodec::decode_protection_status(data);

  assert(status.protected_state == false);
  assert(status.protection_code == 0);

  std::cout << "  ✓ Not protected decoded correctly" << std::endl;
}

void test_decode_homing_status()
{
  std::cout << "Testing decode_homing_status..." << std::endl;

  std::vector<uint8_t> data = {0x02}; // Completed

  uint8_t status = ServoCommandCodec::decode_homing_status(data);
  assert(status == 2);

  std::cout << "  ✓ Homing status decoded correctly" << std::endl;
}

void test_decode_io_status()
{
  std::cout << "Testing decode_io_status..." << std::endl;

  std::vector<uint8_t> data = {0x55}; // Alternating bits

  uint8_t status = ServoCommandCodec::decode_io_status(data);
  assert(status == 0x55);

  std::cout << "  ✓ IO status decoded correctly" << std::endl;
}

// ============================================================================
// Roundtrip Tests
// ============================================================================

void test_roundtrip_encoder_position()
{
  std::cout << "Testing roundtrip encoder position..." << std::endl;

  MockServoXxd mock(3200.0f);

  // Create a position
  Position original(2.5, PositionUnit::REVOLUTIONS, &mock);

  // "Encode" as encoder carry data (simulate what hardware would return)
  std::vector<uint8_t> data(6);

  int32_t revs = original.revolutions();
  uint16_t angle = original.angle_ticks();

  // Encode carry (revolutions)
  data[0] = static_cast<uint8_t>((revs >> 24) & 0xFF);
  data[1] = static_cast<uint8_t>((revs >> 16) & 0xFF);
  data[2] = static_cast<uint8_t>((revs >> 8) & 0xFF);
  data[3] = static_cast<uint8_t>(revs & 0xFF);

  // Encode value (angle_ticks)
  data[4] = static_cast<uint8_t>((angle >> 8) & 0xFF);
  data[5] = static_cast<uint8_t>(angle & 0xFF);

  // Decode
  Position decoded = ServoCommandCodec::decode_encoder_position(data, &mock);

  // Compare
  assert(decoded.revolutions() == original.revolutions());
  assert(decoded.angle_ticks() == original.angle_ticks());

  std::cout << "  ✓ Position roundtrip successful" << std::endl;
  std::cout << "    Original: " << original.revolutions() << " rev + " << original.angle_ticks() << " ticks" << std::endl;
  std::cout << "    Decoded:  " << decoded.revolutions() << " rev + " << decoded.angle_ticks() << " ticks" << std::endl;
}

// ============================================================================
// Main
// ============================================================================

int main()
{
  std::cout << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "ServoCommandCodec Unit Tests" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << std::endl;

  // Encoder tests
  std::cout << "--- Encoder Tests ---" << std::endl;
  test_encode_move_position_mode_2();
  test_encode_move_position_mode_2_validation();
  test_encode_move_position_mode_3();
  test_encode_move_speed_mode();
  test_encode_stop_position_mode_2();
  test_encode_emergency_stop();
  test_encode_enable_motor();
  test_encode_set_working_current();
  test_encode_set_subdivision();
  test_encode_start_homing();
  std::cout << std::endl;

  // Decoder tests
  std::cout << "--- Decoder Tests ---" << std::endl;
  test_decode_encoder_carry();
  test_decode_encoder_position();
  test_decode_current_speed();
  test_decode_motor_speed();
  test_decode_pulse_count();
  test_decode_motor_status();
  test_decode_protection_status();
  test_decode_homing_status();
  test_decode_io_status();
  std::cout << std::endl;

  // Roundtrip tests
  std::cout << "--- Roundtrip Tests ---" << std::endl;
  test_roundtrip_encoder_position();
  std::cout << std::endl;

  std::cout << "========================================" << std::endl;
  std::cout << "✅ All Codec Tests Passed!" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << std::endl;

  return 0;
}
