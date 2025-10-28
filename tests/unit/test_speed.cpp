/**
 * @file test_speed.cpp
 * @brief Unit tests for Speed class with all 7 unit conversions
 *
 * Tests verify:
 * - Conversion accuracy from all 7 units to RPM
 * - Microstepping compensation (rpm_for_hardware)
 * - Edge cases (zero, negative, max values)
 * - Steps/s conversion for ESPHome base class
 */

#include "components/servoxxd_modbus/stepper/servoxxd_speed.h"
#include "components/servoxxd_modbus/stepper/servoxxd_modbus.h"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace esphome::servoxxd_modbus;

// Tolerance for float comparisons
constexpr float EPSILON = 0.01f;

bool float_eq(float a, float b)
{
  return std::abs(a - b) < EPSILON;
}

// Mock ServoXxdModbus for testing
class MockServoXxdModbus : public ServoXxdModbus
{
public:
  MockServoXxdModbus(float steps_per_rev, uint16_t microsteps = 16)
      : steps_per_rev_(steps_per_rev), microsteps_(microsteps) {}

  float get_steps_per_revolution() const override { return steps_per_rev_; }
  uint16_t get_microstepping() const override { return microsteps_; }

private:
  float steps_per_rev_;
  uint16_t microsteps_;
};

void test_speed_steps_per_sec()
{
  std::cout << "Testing STEPS_PER_SEC conversion..." << std::endl;

  float steps_per_rev = 3200.0f; // 200 * 16 microsteps
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 1000 steps/s should be (1000 * 60) / 3200 = 18.75 RPM → rounds to 19
  Speed speed(1000.0f, SpeedUnit::STEPS_PER_SEC, &mock);
  assert(speed.rpm() == 19.0f); // int16_t storage rounds to 19

  std::cout << "  ✓ 1000 steps/s = " << speed.rpm() << " RPM (18.75 → 19 after rounding)" << std::endl;
}

void test_speed_rpm()
{
  std::cout << "Testing RPM conversion (direct)..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 100 RPM should remain 100 RPM (motor native unit)
  Speed speed(100.0f, SpeedUnit::RPM, &mock);
  assert(float_eq(speed.rpm(), 100.0f));

  std::cout << "  ✓ 100 RPM = " << speed.rpm() << " RPM (expected 100)" << std::endl;
}

void test_speed_rev_per_sec()
{
  std::cout << "Testing REV_PER_SEC conversion..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 1.5 rev/s should be 1.5 * 60 = 90 RPM
  Speed speed(1.5f, SpeedUnit::REV_PER_SEC, &mock);
  assert(float_eq(speed.rpm(), 90.0f));

  std::cout << "  ✓ 1.5 rev/s = " << speed.rpm() << " RPM (expected 90)" << std::endl;
}

void test_speed_degrees_per_sec()
{
  std::cout << "Testing DEGREES_PER_SEC conversion..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 360 deg/s should be (360 * 60) / 360 = 60 RPM
  Speed speed(360.0f, SpeedUnit::DEGREES_PER_SEC, &mock);
  assert(float_eq(speed.rpm(), 60.0f));

  std::cout << "  ✓ 360 deg/s = " << speed.rpm() << " RPM (expected 60)" << std::endl;
}

void test_speed_radians_per_sec()
{
  std::cout << "Testing RADIANS_PER_SEC conversion..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);
  constexpr float PI = 3.14159265358979323846f;
  constexpr float two_pi = 2.0f * PI;

  // Test: 2π rad/s (1 rev/s) should be 60 RPM
  Speed speed(two_pi, SpeedUnit::RADIANS_PER_SEC, &mock);
  assert(float_eq(speed.rpm(), 60.0f));

  std::cout << "  ✓ 2π rad/s = " << speed.rpm() << " RPM (expected 60)" << std::endl;
}

void test_speed_degrees_per_min()
{
  std::cout << "Testing DEGREES_PER_MIN conversion..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 360 deg/min should be 360 / 6 = 60 RPM
  Speed speed(360.0f, SpeedUnit::DEGREES_PER_MIN, &mock);
  assert(float_eq(speed.rpm(), 60.0f));

  std::cout << "  ✓ 360 deg/min = " << speed.rpm() << " RPM (expected 60)" << std::endl;
}

void test_speed_degrees_per_hour()
{
  std::cout << "Testing DEGREES_PER_HOUR conversion..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 21600 deg/h should be 21600 / 360 = 60 RPM
  Speed speed(21600.0f, SpeedUnit::DEGREES_PER_HOUR, &mock);
  assert(float_eq(speed.rpm(), 60.0f));

  std::cout << "  ✓ 21600 deg/h = " << speed.rpm() << " RPM (expected 60)" << std::endl;
}

void test_speed_microstepping_compensation()
{
  std::cout << "Testing microstepping compensation..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock_base(steps_per_rev);
  Speed speed(100.0f, SpeedUnit::RPM, &mock_base);

  // Test different microstepping values
  MockServoXxdModbus mock_8(steps_per_rev, 8);
  MockServoXxdModbus mock_16(steps_per_rev, 16);
  MockServoXxdModbus mock_64(steps_per_rev, 64);
  MockServoXxdModbus mock_128(steps_per_rev, 128);
  MockServoXxdModbus mock_256(steps_per_rev, 256);

  // Reference: 16/32/64 should return unchanged
  int16_t rpm_16 = speed.rpm_for_hardware(&mock_16);
  assert(rpm_16 == 100);
  std::cout << "  ✓ microsteps=16: " << rpm_16 << " RPM (no compensation)" << std::endl;

  int16_t rpm_64 = speed.rpm_for_hardware(&mock_64);
  assert(rpm_64 == 100);
  std::cout << "  ✓ microsteps=64: " << rpm_64 << " RPM (no compensation)" << std::endl;

  // microsteps=8: divide by 2
  int16_t rpm_8 = speed.rpm_for_hardware(&mock_8);
  assert(rpm_8 == 50);
  std::cout << "  ✓ microsteps=8: " << rpm_8 << " RPM (÷2)" << std::endl;

  // microsteps=128: multiply by 8
  int16_t rpm_128 = speed.rpm_for_hardware(&mock_128);
  assert(rpm_128 == 800);
  std::cout << "  ✓ microsteps=128: " << rpm_128 << " RPM (×8)" << std::endl;

  // microsteps=256: multiply by 16
  int16_t rpm_256 = speed.rpm_for_hardware(&mock_256);
  assert(rpm_256 == 1600);
  std::cout << "  ✓ microsteps=256: " << rpm_256 << " RPM (×16)" << std::endl;
}

void test_speed_steps_per_sec_conversion()
{
  std::cout << "Testing steps_per_sec() for ESPHome..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev, 16);
  Speed speed(60.0f, SpeedUnit::RPM, &mock);

  // 60 RPM = 1 rev/s = 3200 steps/s
  float steps_per_s = speed.steps_per_sec(&mock);
  assert(float_eq(steps_per_s, 3200.0f));

  std::cout << "  ✓ 60 RPM = " << steps_per_s << " steps/s (expected 3200)" << std::endl;
}

void test_speed_negative_values()
{
  std::cout << "Testing negative speed values..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: -100 RPM should work (reverse direction)
  Speed speed(-100.0f, SpeedUnit::RPM, &mock);
  assert(float_eq(speed.rpm(), -100.0f));
  assert(speed.rpm_as_i16() == -100);

  std::cout << "  ✓ -100 RPM = " << speed.rpm() << " RPM (reverse direction)" << std::endl;
}

void test_speed_range_clamping()
{
  std::cout << "Testing range clamping..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: Very high speed should clamp to int16_t max (32767)
  Speed speed_high(40000.0f, SpeedUnit::RPM, &mock);
  assert(speed_high.rpm_as_i16() == 32767);
  std::cout << "  ✓ 40000 RPM clamped to " << speed_high.rpm_as_i16() << " (int16_t max)" << std::endl;

  // Test: Very low speed should clamp to int16_t min (-32768)
  Speed speed_low(-40000.0f, SpeedUnit::RPM, &mock);
  assert(speed_low.rpm_as_i16() == -32768);
  std::cout << "  ✓ -40000 RPM clamped to " << speed_low.rpm_as_i16() << " (int16_t min)" << std::endl;
}

void test_speed_null_parent()
{
  std::cout << "Testing null parent pointer handling..." << std::endl;

  // Test: STEPS_PER_SEC with null parent should default to 0
  Speed speed_null(1000.0f, SpeedUnit::STEPS_PER_SEC, nullptr);
  assert(speed_null.rpm() == 0.0f);
  std::cout << "  ✓ STEPS_PER_SEC with null parent → 0 RPM (error handling)" << std::endl;

  // Test: Other units with null parent should work fine
  Speed speed_rpm_null(100.0f, SpeedUnit::RPM, nullptr);
  assert(speed_rpm_null.rpm() == 100.0f);
  std::cout << "  ✓ RPM with null parent → 100 RPM (parent not needed)" << std::endl;
}

void test_speed_zero_value()
{
  std::cout << "Testing zero speed value..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  Speed speed_zero(0.0f, SpeedUnit::RPM, &mock);
  assert(speed_zero.rpm() == 0.0f);
  assert(speed_zero.rpm_as_i16() == 0);
  std::cout << "  ✓ 0 RPM = 0 RPM" << std::endl;
}

void test_speed_invalid_steps_per_revolution()
{
  std::cout << "Testing invalid steps_per_revolution..." << std::endl;

  // Mock with invalid (negative) steps_per_rev
  MockServoXxdModbus mock_invalid(-100.0f);

  // Should handle invalid steps_per_rev gracefully
  Speed speed(1000.0f, SpeedUnit::STEPS_PER_SEC, &mock_invalid);
  assert(speed.rpm() == 0.0f); // Should default to 0
  std::cout << "  ✓ STEPS_PER_SEC with negative steps_per_rev → 0 RPM (error handling)" << std::endl;

  // Mock with zero steps_per_rev
  MockServoXxdModbus mock_zero(0.0f);
  Speed speed_zero(1000.0f, SpeedUnit::STEPS_PER_SEC, &mock_zero);
  assert(speed_zero.rpm() == 0.0f);
  std::cout << "  ✓ STEPS_PER_SEC with zero steps_per_rev → 0 RPM (error handling)" << std::endl;
}

int main()
{
  std::cout << "\n=== Speed Class Unit Tests ===" << std::endl;
  std::cout << std::endl;

  test_speed_steps_per_sec();
  test_speed_rpm();
  test_speed_rev_per_sec();
  test_speed_degrees_per_sec();
  test_speed_radians_per_sec();
  test_speed_degrees_per_min();
  test_speed_degrees_per_hour();
  test_speed_microstepping_compensation();
  test_speed_steps_per_sec_conversion();
  test_speed_negative_values();
  test_speed_range_clamping();
  test_speed_null_parent();
  test_speed_zero_value();
  test_speed_invalid_steps_per_revolution();

  std::cout << std::endl;
  std::cout << "✅ All Speed tests passed!" << std::endl;
  std::cout << std::endl;

  return 0;
}
