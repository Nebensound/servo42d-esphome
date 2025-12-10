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
  // Use from_ticks() which is hardware-native (encoder counts)
  Position pos1 = Position::from_ticks(1000); // 1000 encoder ticks
  Speed spd1 = Speed::from_rpm(100, nullptr);
  Acceleration acc1 = Acceleration::from_internal(50); // Direct hardware value
  auto data = ServoCommandCodec::encode_move_position_mode_2(pos1, spd1, acc1);
  ASSERT_EQUAL(data.size(), 8u, "Payload size is 8 bytes (2+2+4)");
  ASSERT_EQUAL(data[0], 0x00, "Accel MSB");
  ASSERT_EQUAL(data[1], 0x32, "Accel LSB = 50");
  ASSERT_EQUAL(data[2], 0x00, "Speed MSB");
  ASSERT_EQUAL(data[3], 0x64, "Speed LSB = 100");
  ASSERT_EQUAL(data[4], 0x00, "Position byte 0");
  ASSERT_EQUAL(data[5], 0x00, "Position byte 1");
  ASSERT_EQUAL(data[6], 0x03, "Position byte 2");
  ASSERT_EQUAL(data[7], 0xE8, "Position byte 3 = 1000");

  // Test negative position
  Position pos2 = Position::from_ticks(-1000);
  Speed spd2 = Speed::from_rpm(200, nullptr);
  Acceleration acc2 = Acceleration::from_internal(100);
  data = ServoCommandCodec::encode_move_position_mode_2(pos2, spd2, acc2);
  ASSERT_EQUAL(data.size(), 8u, "Negative position payload size");
  ASSERT_EQUAL(data[0], 0x00, "Accel MSB");
  ASSERT_EQUAL(data[1], 100, "Accel LSB = 100");
  ASSERT_EQUAL(data[2], 0x00, "Speed MSB");
  ASSERT_EQUAL(data[3], 0xC8, "Speed LSB = 200");
  ASSERT_EQUAL(data[4], 0xFF, "Negative position byte 0");
  ASSERT_EQUAL(data[5], 0xFF, "Negative position byte 1");
  ASSERT_EQUAL(data[6], 0xFC, "Negative position byte 2");
  ASSERT_EQUAL(data[7], 0x18, "Negative position byte 3");

  // Test zero values
  Position pos3 = Position::from_ticks(0);
  Speed spd3 = Speed::from_rpm(0, nullptr);
  Acceleration acc3 = Acceleration::from_internal(0);
  data = ServoCommandCodec::encode_move_position_mode_2(pos3, spd3, acc3);
  ASSERT_EQUAL(data.size(), 8u, "Zero values payload size");
  ASSERT_EQUAL(data[0], 0x00, "Accel MSB zero");
  ASSERT_EQUAL(data[1], 0x00, "Accel LSB zero");
  ASSERT_EQUAL(data[2], 0x00, "Speed MSB zero");
  ASSERT_EQUAL(data[3], 0x00, "Speed LSB zero");
  ASSERT_EQUAL(data[4], 0x00, "Position byte 0 zero");
  ASSERT_EQUAL(data[7], 0x00, "Position byte 3 zero");

  // Test maximum values
  Position pos4 = Position::from_ticks(INT32_MAX);
  Speed spd4 = Speed::from_rpm(1000, nullptr);
  Acceleration acc4 = Acceleration::from_internal(255);
  data = ServoCommandCodec::encode_move_position_mode_2(pos4, spd4, acc4);
  ASSERT_EQUAL(data.size(), 8u, "Max values payload size");
  ASSERT_EQUAL(data[0], 0x00, "Max accel MSB (255 as uint16)");
  ASSERT_EQUAL(data[1], 0xFF, "Max accel LSB");
  ASSERT_EQUAL(data[2], 0x03, "Max speed MSB");
  ASSERT_EQUAL(data[3], 0xE8, "Max speed LSB");
  ASSERT_EQUAL(data[4], 0x7F, "Max position byte 0");
  ASSERT_EQUAL(data[5], 0xFF, "Max position byte 1");
  ASSERT_EQUAL(data[6], 0xFF, "Max position byte 2");
  ASSERT_EQUAL(data[7], 0xFF, "Max position byte 3");
}

void test_encode_stop_position_mode_2()
{
  std::cout << "\n=== encode_stop_position_mode_2 Tests ===" << std::endl;

  Acceleration decel1 = Acceleration::from_internal(128);
  auto data = ServoCommandCodec::encode_stop_position_mode_2(decel1);
  ASSERT_EQUAL(data.size(), 1u, "Stop payload size is 1 byte");
  ASSERT_EQUAL(data[0], 128, "Decel value = 128");

  Acceleration decel2 = Acceleration::from_internal(0);
  data = ServoCommandCodec::encode_stop_position_mode_2(decel2);
  ASSERT_EQUAL(data[0], 0, "Zero decel (instant stop)");

  Acceleration decel3 = Acceleration::from_internal(255);
  data = ServoCommandCodec::encode_stop_position_mode_2(decel3);
  ASSERT_EQUAL(data[0], 255, "Max decel value");
}

void test_encode_move_speed_mode()
{
  std::cout << "\n=== encode_move_speed_mode Tests ===" << std::endl;

  Speed spd1 = Speed::from_rpm(1000, nullptr); // Direction embedded in Speed (positive = CW)
  Acceleration acc1 = Acceleration::from_internal(50);
  auto data = ServoCommandCodec::encode_move_speed_mode(spd1, acc1);
  ASSERT_EQUAL(data.size(), 4u, "Speed mode payload size is 4 bytes");
  ASSERT_EQUAL(data[0], 1, "Direction = 1 (CW/forward)");
  ASSERT_EQUAL(data[1], 0x03, "Speed MSB");
  ASSERT_EQUAL(data[2], 0xE8, "Speed LSB = 1000");
  ASSERT_EQUAL(data[3], 50, "Accel = 50");

  Speed spd2 = Speed::from_rpm(-500, nullptr); // Negative speed = CCW
  Acceleration acc2 = Acceleration::from_internal(100);
  data = ServoCommandCodec::encode_move_speed_mode(spd2, acc2);
  ASSERT_EQUAL(data[0], 0, "Direction = 0 (CCW/reverse)");
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

  // Test control mode
  data = ServoCommandCodec::encode_set_control_mode(ControlMode::SR_OPEN);
  ASSERT_EQUAL(data.size(), 1u, "Control mode payload size");
  ASSERT_EQUAL(data[0], 3, "SR_OPEN = 3");

  data = ServoCommandCodec::encode_set_control_mode(ControlMode::SR_CLOSE);
  ASSERT_EQUAL(data[0], 4, "SR_CLOSE = 4");

  data = ServoCommandCodec::encode_set_control_mode(ControlMode::SR_VFOC);
  ASSERT_EQUAL(data[0], 5, "SR_VFOC = 5");
}

void test_decode_current_speed()
{
  std::cout << "\n=== decode_current_speed Tests ===" << std::endl;

  // Positive speed
  std::vector<uint8_t> data = {0x00, 0x64}; // 100 RPM
  Speed speed = ServoCommandCodec::decode_current_speed(data);
  ASSERT_EQUAL(speed.rpm_internal(), 100, "Decode 100 RPM");

  // Negative speed (two's complement)
  data = {0xFF, 0x9C}; // -100
  speed = ServoCommandCodec::decode_current_speed(data);
  ASSERT_EQUAL(speed.rpm_internal(), -100, "Decode -100 RPM");

  // Zero speed
  data = {0x00, 0x00};
  speed = ServoCommandCodec::decode_current_speed(data);
  ASSERT_EQUAL(speed.rpm_internal(), 0, "Decode 0 RPM");

  // Maximum hardware speed (3000 RPM is hardware limit)
  data = {0x0B, 0xB8}; // 3000
  speed = ServoCommandCodec::decode_current_speed(data);
  ASSERT_EQUAL(speed.rpm_internal(), 3000, "Decode max hardware speed (3000 RPM)");

  // Minimum hardware speed (-3000 RPM)
  data = {0xF4, 0x48}; // -3000
  speed = ServoCommandCodec::decode_current_speed(data);
  ASSERT_EQUAL(speed.rpm_internal(), -3000, "Decode min hardware speed (-3000 RPM)");

  // Insufficient data
  data = {0x00};
  speed = ServoCommandCodec::decode_current_speed(data);
  ASSERT_EQUAL(speed.rpm_internal(), 0, "Insufficient data returns 0");

  // Empty data
  data.clear();
  speed = ServoCommandCodec::decode_current_speed(data);
  ASSERT_EQUAL(speed.rpm_internal(), 0, "Empty data returns 0");
}

void test_decode_pulse_count()
{
  std::cout << "\n=== decode_pulse_count Tests ===" << std::endl;

  // Positive count
  std::vector<uint8_t> data = {0x00, 0x00, 0x03, 0xE8}; // 1000
  Position count = ServoCommandCodec::decode_pulse_count(data);
  ASSERT_EQUAL(count.get_ticks(), 1000, "Decode 1000 ticks");

  // Negative count
  data = {0xFF, 0xFF, 0xFC, 0x18}; // -1000
  count = ServoCommandCodec::decode_pulse_count(data);
  ASSERT_EQUAL(count.get_ticks(), -1000, "Decode -1000 ticks");

  // Zero
  data = {0x00, 0x00, 0x00, 0x00};
  count = ServoCommandCodec::decode_pulse_count(data);
  ASSERT_EQUAL(count.get_ticks(), 0, "Decode 0 ticks");

  // Large positive
  data = {0x00, 0x0F, 0x42, 0x40}; // 1,000,000
  count = ServoCommandCodec::decode_pulse_count(data);
  ASSERT_EQUAL(count.get_ticks(), 1000000, "Decode 1,000,000 ticks");

  // Maximum positive
  data = {0x7F, 0xFF, 0xFF, 0xFF}; // INT32_MAX
  count = ServoCommandCodec::decode_pulse_count(data);
  ASSERT_EQUAL(count.get_ticks(), INT32_MAX, "Decode INT32_MAX");

  // Insufficient data
  data = {0x00, 0x00, 0x03};
  count = ServoCommandCodec::decode_pulse_count(data);
  ASSERT_EQUAL(count.get_ticks(), 0, "Insufficient data returns 0");
}

void test_decode_encoder_carry()
{
  std::cout << "\n=== decode_encoder_carry Tests ===" << std::endl;

  // Normal encoder value: carry=5, value=0x1234 → position = 5×0x4000 + 0x1234 = 0x15234 = 86580
  std::vector<uint8_t> data = {0x00, 0x00, 0x00, 0x05, 0x12, 0x34};
  Position pos = ServoCommandCodec::decode_encoder_carry(data);
  ASSERT_EQUAL(pos.get_ticks(), 86580, "Position from carry=5, value=0x1234");

  // Zero values
  data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  pos = ServoCommandCodec::decode_encoder_carry(data);
  ASSERT_EQUAL(pos.get_ticks(), 0, "Zero position");

  // One full rotation: carry=1, value=0 → position = 0x4000 = 16384
  data = {0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
  pos = ServoCommandCodec::decode_encoder_carry(data);
  ASSERT_EQUAL(pos.get_ticks(), 16384, "One rotation (carry=1, value=0)");

  // Negative carry: carry=-1, value=0x3FF0 → position = -0x4000 + 0x3FF0 = -16
  data = {0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0xF0};
  pos = ServoCommandCodec::decode_encoder_carry(data);
  ASSERT_EQUAL(pos.get_ticks(), -16, "Negative carry position");

  // Large carry: carry=10000, value=0x3FFF → position = 10000×0x4000 + 0x3FFF = 163856383
  data = {0x00, 0x00, 0x27, 0x10, 0x3F, 0xFF};
  pos = ServoCommandCodec::decode_encoder_carry(data);
  int64_t expected = (10000LL * 0x4000LL) + 0x3FFF;
  ASSERT_EQUAL(pos.get_ticks(), expected, "Large carry position");

  // Insufficient data
  data = {0x00, 0x00, 0x00, 0x05};
  pos = ServoCommandCodec::decode_encoder_carry(data);
  ASSERT_EQUAL(pos.get_ticks(), 0, "Insufficient data returns zero position");
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

void test_decode_homing_status()
{
  std::cout << "\n=== decode_homing_status Tests ===" << std::endl;

  std::vector<uint8_t> data;
  ServoCommandCodec::HomingStatus hs;

  // Test IDLE state
  data = {0x00};
  hs = ServoCommandCodec::decode_homing_status(data);
  ASSERT_EQUAL(static_cast<int>(hs.state), static_cast<int>(ServoCommandCodec::HomingStatus::IDLE), "Status = IDLE");
  ASSERT_EQUAL(hs.error_code, 0, "IDLE error_code = 0");

  // Test IN_PROGRESS state
  data = {0x01};
  hs = ServoCommandCodec::decode_homing_status(data);
  ASSERT_EQUAL(static_cast<int>(hs.state), static_cast<int>(ServoCommandCodec::HomingStatus::IN_PROGRESS), "Status = IN_PROGRESS");
  ASSERT_EQUAL(hs.error_code, 0, "IN_PROGRESS error_code = 0");

  // Test COMPLETED state
  data = {0x02};
  hs = ServoCommandCodec::decode_homing_status(data);
  ASSERT_EQUAL(static_cast<int>(hs.state), static_cast<int>(ServoCommandCodec::HomingStatus::COMPLETED), "Status = COMPLETED");
  ASSERT_EQUAL(hs.error_code, 0, "COMPLETED error_code = 0");

  // Test ERROR state (value 3)
  data = {0x03};
  hs = ServoCommandCodec::decode_homing_status(data);
  ASSERT_EQUAL(static_cast<int>(hs.state), static_cast<int>(ServoCommandCodec::HomingStatus::ERROR), "Status = ERROR (3)");
  ASSERT_EQUAL(hs.error_code, 3, "ERROR error_code = 3");

  // Test ERROR state (value 4 - no endstop)
  data = {0x04};
  hs = ServoCommandCodec::decode_homing_status(data);
  ASSERT_EQUAL(static_cast<int>(hs.state), static_cast<int>(ServoCommandCodec::HomingStatus::ERROR), "Status = ERROR (4)");
  ASSERT_EQUAL(hs.error_code, 4, "ERROR error_code = 4");

  // Test ERROR state (value 5 - position error)
  data = {0x05};
  hs = ServoCommandCodec::decode_homing_status(data);
  ASSERT_EQUAL(static_cast<int>(hs.state), static_cast<int>(ServoCommandCodec::HomingStatus::ERROR), "Status = ERROR (5)");
  ASSERT_EQUAL(hs.error_code, 5, "ERROR error_code = 5");

  // Test empty data (communication error)
  data.clear();
  hs = ServoCommandCodec::decode_homing_status(data);
  ASSERT_EQUAL(static_cast<int>(hs.state), static_cast<int>(ServoCommandCodec::HomingStatus::ERROR), "Empty data → ERROR");
  ASSERT_EQUAL(hs.error_code, 0xFF, "Empty data error_code = 0xFF (comm error)");
}

void test_encode_homing_commands()
{
  std::cout << "\n=== Homing Encoder Tests ===" << std::endl;

  using namespace esphome::servoxxd;

  // Test encode_set_home_parameters (Command 0x90 - "Set the parameter of home")
  // Format: [hmTrig][hmDir][HmSpeed_hi][HmSpeed_lo][EndLimit]
  auto data = ServoCommandCodec::encode_set_home_parameters(
      EndstopTrigger::TRIGGER_HIGH,
      Direction::CW,
      Speed::from_rpm(500, nullptr), // 500 RPM
      true);
  ASSERT_EQUAL(data.size(), 5u, "Home params payload size = 5");
  ASSERT_EQUAL(data[0], 0x01, "Trigger = TRIGGER_HIGH");
  ASSERT_EQUAL(data[1], 0x00, "Direction = CW");
  ASSERT_EQUAL(data[2], 0x01, "Speed MSB (500)");
  ASSERT_EQUAL(data[3], 0xF4, "Speed LSB (500)");
  ASSERT_EQUAL(data[4], 0x01, "Endlimit enable = true");

  // Test with TRIGGER_LOW and CCW
  data = ServoCommandCodec::encode_set_home_parameters(
      EndstopTrigger::TRIGGER_LOW,
      Direction::CCW,
      Speed::from_rpm(300, nullptr), // 300 RPM
      false);
  ASSERT_EQUAL(data[0], 0x00, "Trigger = TRIGGER_LOW");
  ASSERT_EQUAL(data[1], 0x01, "Direction = CCW");
  ASSERT_EQUAL(data[2], 0x01, "Speed MSB (300)");
  ASSERT_EQUAL(data[3], 0x2C, "Speed LSB (300)");
  ASSERT_EQUAL(data[4], 0x00, "Endlimit enable = false");

  // Test encode_set_nolimit_home_parameters (SENSORLESS mode enabled)
  // Format: [retValue_b3][b2][b1][b0][mode_hi][mode_lo][ma_hi][ma_lo]
  Position reverse_pos = Position::from_ticks(500);
  data = ServoCommandCodec::encode_set_nolimit_home_parameters(
      reverse_pos,
      true,  // sensorless_enabled=true: no limit switch for go home
      1500); // current threshold (ma)
  ASSERT_EQUAL(data.size(), 8u, "Nolimit params payload size = 8");
  ASSERT_EQUAL(data[0], 0x00, "retValue byte 0");
  ASSERT_EQUAL(data[1], 0x00, "retValue byte 1");
  ASSERT_EQUAL(data[2], 0x01, "retValue byte 2");
  ASSERT_EQUAL(data[3], 0xF4, "retValue byte 3 (500 ticks)");
  ASSERT_EQUAL(data[4], 0x00, "mode MSB");
  ASSERT_EQUAL(data[5], 0x01, "mode LSB = 1 (sensorless enabled)");
  ASSERT_EQUAL(data[6], 0x05, "ma MSB (1500 mA)");
  ASSERT_EQUAL(data[7], 0xDC, "ma LSB (1500 mA)");

  // Test encode_set_nolimit_home_parameters (SENSORLESS disabled - ENDSTOP mode)
  data = ServoCommandCodec::encode_set_nolimit_home_parameters(
      Position::from_ticks(0x2000), // 180° (default)
      false,                        // sensorless_enabled=false: use limit switch
      0);                           // current=0 (disabled)
  ASSERT_EQUAL(data.size(), 8u, "Nolimit disabled payload size = 8");
  ASSERT_EQUAL(data[0], 0x00, "retValue byte 0");
  ASSERT_EQUAL(data[1], 0x00, "retValue byte 1");
  ASSERT_EQUAL(data[2], 0x20, "retValue byte 2 (0x2000 = 180°)");
  ASSERT_EQUAL(data[3], 0x00, "retValue byte 3");
  ASSERT_EQUAL(data[4], 0x00, "mode MSB");
  ASSERT_EQUAL(data[5], 0x00, "mode LSB = 0 (sensorless disabled)");
  ASSERT_EQUAL(data[6], 0x00, "ma MSB (0 mA)");
  ASSERT_EQUAL(data[7], 0x00, "ma LSB (0 mA)");

  // Test encode_set_nolimit_home_parameters with 360° reverse angle
  data = ServoCommandCodec::encode_set_nolimit_home_parameters(
      Position::from_ticks(0x4000), // 360°
      true,                         // sensorless_enabled=true
      2000);                        // 2000 mA
  ASSERT_EQUAL(data[0], 0x00, "retValue byte 0");
  ASSERT_EQUAL(data[1], 0x00, "retValue byte 1");
  ASSERT_EQUAL(data[2], 0x40, "retValue byte 2 (0x4000 = 360°)");
  ASSERT_EQUAL(data[3], 0x00, "retValue byte 3");
  ASSERT_EQUAL(data[4], 0x00, "mode MSB");
  ASSERT_EQUAL(data[5], 0x01, "mode LSB = 1 (sensorless enabled)");
  ASSERT_EQUAL(data[6], 0x07, "ma MSB (2000 mA)");
  ASSERT_EQUAL(data[7], 0xD0, "ma LSB (2000 mA)");

  // Test encode_set_zero_mode (VIRTUAL mode with 0_Mode)
  // Format: [mode][enable][speed][direction]
  data = ServoCommandCodec::encode_set_zero_mode(
      ServoCommandCodec::ZeroMode::NEAR_MODE, // mode=2: NearMode
      ServoCommandCodec::ZeroModeTask::SET,   // clean_set=SET (set zero)
      ZeroingSpeed::MEDIUM,
      Direction::CW);
  ASSERT_EQUAL(data.size(), 4u, "Zero mode payload size = 4");
  ASSERT_EQUAL(data[0], 0x02, "Mode = 2 (NearMode)");
  ASSERT_EQUAL(data[1], 0x01, "Enable = true");
  ASSERT_EQUAL(data[2], 0x02, "Speed = MEDIUM");
  ASSERT_EQUAL(data[3], 0x00, "Direction = CW (0)");

  // Test with DirMode CW
  data = ServoCommandCodec::encode_set_zero_mode(
      ServoCommandCodec::ZeroMode::DIR_MODE,  // mode=1: DirMode
      ServoCommandCodec::ZeroModeTask::CLEAN, // clean_set=CLEAN (clean zero)
      ZeroingSpeed::VERY_SLOW,
      Direction::CCW);
  ASSERT_EQUAL(data[0], 0x01, "Mode = 1 (DirMode)");
  ASSERT_EQUAL(data[1], 0x00, "Enable = false");
  ASSERT_EQUAL(data[2], 0x00, "Speed = VERY_SLOW");
  ASSERT_EQUAL(data[3], 0x01, "Direction = CCW (1)");

  // Test with VERY_FAST speed
  data = ServoCommandCodec::encode_set_zero_mode(
      ServoCommandCodec::ZeroMode::DIR_MODE, // mode=1: DirMode
      ServoCommandCodec::ZeroModeTask::SET,  // clean_set=SET
      ZeroingSpeed::VERY_FAST,
      Direction::CW);
  ASSERT_EQUAL(data[0], 0x01, "Mode = 1 (DirMode)");
  ASSERT_EQUAL(data[1], 0x01, "Enable = true");
  ASSERT_EQUAL(data[2], 0x04, "Speed = VERY_FAST");
  ASSERT_EQUAL(data[3], 0x00, "Direction = CW (0)");

  // Test Disable mode (clean_set=CLEAN means clean zero)
  data = ServoCommandCodec::encode_set_zero_mode(
      ServoCommandCodec::ZeroMode::DIR_MODE,  // mode
      ServoCommandCodec::ZeroModeTask::CLEAN, // clean_set=CLEAN (clean zero)
      ZeroingSpeed::SLOW,
      Direction::CW);
  ASSERT_EQUAL(data[0], 0x01, "Mode = 1 (value when disabled)");
  ASSERT_EQUAL(data[1], 0x00, "Enable = false");
  ASSERT_EQUAL(data[2], 0x01, "Speed = SLOW");
  ASSERT_EQUAL(data[3], 0x00, "Direction = CW (0)");

  // Test encode_go_home (empty payload)
  data = ServoCommandCodec::encode_go_home();
  ASSERT_EQUAL(data.size(), 0u, "Go home has no payload");

  // Test encode_set_current_axis_zero (empty payload)
  data = ServoCommandCodec::encode_set_current_axis_zero();
  ASSERT_EQUAL(data.size(), 0u, "Set current axis zero has no payload");

  // Test encode_set_limit_port_remap
  data = ServoCommandCodec::encode_set_limit_port_remap(true);
  ASSERT_EQUAL(data.size(), 1u, "Limit remap payload size = 1");
  ASSERT_EQUAL(data[0], 0x01, "Remap enable = true");

  data = ServoCommandCodec::encode_set_limit_port_remap(false);
  ASSERT_EQUAL(data[0], 0x00, "Remap enable = false");
}

void test_roundtrip_encoding()
{
  std::cout << "\n=== Round-Trip Encoding Tests ===" << std::endl;

  // Test move command encode → manual decode
  // Format: [acc_hi] [acc_lo] [speed_hi] [speed_lo] [pos_b3] [pos_b2] [pos_b1] [pos_b0]
  Position pos_obj = Position::from_ticks(12345);
  Speed spd_obj = Speed::from_rpm(999, nullptr);
  Acceleration acc_obj = Acceleration::from_internal(128);
  auto encoded = ServoCommandCodec::encode_move_position_mode_2(pos_obj, spd_obj, acc_obj);
  uint16_t acc = (static_cast<uint16_t>(encoded[0]) << 8) | encoded[1];
  uint16_t spd = (static_cast<uint16_t>(encoded[2]) << 8) | encoded[3];
  int32_t pos = (static_cast<int32_t>(encoded[4]) << 24) |
                (static_cast<int32_t>(encoded[5]) << 16) |
                (static_cast<int32_t>(encoded[6]) << 8) |
                static_cast<int32_t>(encoded[7]);

  ASSERT_EQUAL(acc, 128, "Round-trip accel");
  ASSERT_EQUAL(spd, 999, "Round-trip speed");
  ASSERT_EQUAL(pos, 12345, "Round-trip position");

  // Test current reading encode → decode
  std::vector<uint8_t> speed_data = {0x01, 0xF4}; // 500 RPM
  Speed decoded_speed = ServoCommandCodec::decode_current_speed(speed_data);
  ASSERT_EQUAL(decoded_speed.rpm_internal(), 500, "Round-trip current speed");

  // Test pulse count encode → decode
  std::vector<uint8_t> pulse_data = {0x00, 0x01, 0x86, 0xA0}; // 100,000 ticks
  Position decoded_pulses = ServoCommandCodec::decode_pulse_count(pulse_data);
  ASSERT_EQUAL(decoded_pulses.get_ticks(), 100000, "Round-trip pulse count");
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
  test_decode_homing_status();
  test_encode_homing_commands();
  test_roundtrip_encoding();

  std::cout << "\n========================================" << std::endl;
  std::cout << "✅ All CommandCodec Tests Passed (" << tests_passed << " assertions)!" << std::endl;
  std::cout << "========================================" << std::endl;

  return 0;
}
