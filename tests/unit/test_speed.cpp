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
  const float pi_value = static_cast<float>(PI);
  const float two_pi = 2.0f * pi_value;

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

  // Test different microstepping values - need separate Speed objects for each
  MockServoXxdModbus mock_8(steps_per_rev, 8);
  MockServoXxdModbus mock_16(steps_per_rev, 16);
  MockServoXxdModbus mock_64(steps_per_rev, 64);
  MockServoXxdModbus mock_128(steps_per_rev, 128);
  MockServoXxdModbus mock_256(steps_per_rev, 256);

  // Reference: 16/32/64 should return unchanged
  Speed speed_16(100.0f, SpeedUnit::RPM, &mock_16);
  int16_t rpm_16 = speed_16.rpm_for_hardware();
  assert(rpm_16 == 100);
  std::cout << "  ✓ microsteps=16: " << rpm_16 << " RPM (no compensation)" << std::endl;

  Speed speed_64(100.0f, SpeedUnit::RPM, &mock_64);
  int16_t rpm_64 = speed_64.rpm_for_hardware();
  assert(rpm_64 == 100);
  std::cout << "  ✓ microsteps=64: " << rpm_64 << " RPM (no compensation)" << std::endl;

  // microsteps=8: multiply by 2 (hardware divides by 2, so we compensate)
  Speed speed_8(100.0f, SpeedUnit::RPM, &mock_8);
  int16_t rpm_8 = speed_8.rpm_for_hardware();
  assert(rpm_8 == 200);
  std::cout << "  ✓ microsteps=8: " << rpm_8 << " RPM (×2)" << std::endl;

  // microsteps=128: divide by 8 (hardware multiplies by 8, so we compensate)
  Speed speed_128(100.0f, SpeedUnit::RPM, &mock_128);
  int16_t rpm_128 = speed_128.rpm_for_hardware();
  assert(rpm_128 == 12); // 100 / 8 = 12 (integer division)
  std::cout << "  ✓ microsteps=128: " << rpm_128 << " RPM (÷8)" << std::endl;

  // microsteps=256: divide by 16 (hardware multiplies by 16, so we compensate)
  Speed speed_256(100.0f, SpeedUnit::RPM, &mock_256);
  int16_t rpm_256 = speed_256.rpm_for_hardware();
  assert(rpm_256 == 6); // 100 / 16 = 6 (integer division)
  std::cout << "  ✓ microsteps=256: " << rpm_256 << " RPM (÷16)" << std::endl;
}

void test_speed_steps_per_sec_conversion()
{
  std::cout << "Testing steps_per_sec() for ESPHome..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev, 16);
  Speed speed(60.0f, SpeedUnit::RPM, &mock);

  // 60 RPM = 1 rev/s = 3200 steps/s
  float steps_per_s = speed.steps_per_sec();
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

  // Test: Very high speed should clamp to hardware max (3000 RPM)
  Speed speed_high(40000.0f, SpeedUnit::RPM, &mock);
  assert(speed_high.rpm_as_i16() == 3000);
  std::cout << "  ✓ 40000 RPM clamped to " << speed_high.rpm_as_i16() << " (hardware max)" << std::endl;

  // Test: Very low speed should clamp to hardware min (-3000 RPM)
  Speed speed_low(-40000.0f, SpeedUnit::RPM, &mock);
  assert(speed_low.rpm_as_i16() == -3000);
  std::cout << "  ✓ -40000 RPM clamped to " << speed_low.rpm_as_i16() << " (hardware min)" << std::endl;
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

void test_speed_factory_methods()
{
  std::cout << "Testing factory methods..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test from_rpm
  Speed speed1 = Speed::from_rpm(100.0f, &mock);
  assert(speed1.get_rpm() == 100);
  std::cout << "  ✓ from_rpm(100) → " << speed1.get_rpm() << " RPM" << std::endl;

  // Test from_steps_per_sec
  Speed speed2 = Speed::from_steps_per_sec(3200.0f, &mock);
  assert(speed2.get_rpm() == 60);
  std::cout << "  ✓ from_steps_per_sec(3200) → " << speed2.get_rpm() << " RPM" << std::endl;

  // Test from_rev_per_sec
  Speed speed3 = Speed::from_rev_per_sec(1.5f, &mock);
  assert(speed3.get_rpm() == 90);
  std::cout << "  ✓ from_rev_per_sec(1.5) → " << speed3.get_rpm() << " RPM" << std::endl;

  // Test from_degrees_per_sec
  Speed speed4 = Speed::from_degrees_per_sec(360.0f, &mock);
  assert(speed4.get_rpm() == 60);
  std::cout << "  ✓ from_degrees_per_sec(360) → " << speed4.get_rpm() << " RPM" << std::endl;

  // Test from_radians_per_sec
  const float pi_value = static_cast<float>(PI);
  const float two_pi = 2.0f * pi_value;
  Speed speed5 = Speed::from_radians_per_sec(two_pi, &mock);
  assert(speed5.get_rpm() == 60);
  std::cout << "  ✓ from_radians_per_sec(2π) → " << speed5.get_rpm() << " RPM" << std::endl;

  // Test from_degrees_per_min
  Speed speed6 = Speed::from_degrees_per_min(360.0f, &mock);
  assert(speed6.get_rpm() == 60);
  std::cout << "  ✓ from_degrees_per_min(360) → " << speed6.get_rpm() << " RPM" << std::endl;

  // Test from_degrees_per_hour
  Speed speed7 = Speed::from_degrees_per_hour(21600.0f, &mock);
  assert(speed7.get_rpm() == 60);
  std::cout << "  ✓ from_degrees_per_hour(21600) → " << speed7.get_rpm() << " RPM" << std::endl;
}

void test_speed_unit_conversions()
{
  std::cout << "Testing all unit accessor methods..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Create 60 RPM speed (1 rev/s)
  Speed speed(60.0f, SpeedUnit::RPM, &mock);

  // Test int return types
  assert(speed.get_rpm() == 60);
  assert(speed.get_steps_per_sec() == 3200);
  assert(speed.get_degrees_per_sec() == 360);
  assert(speed.get_degrees_per_min() == 360);
  assert(speed.get_degrees_per_hour() == 21600);
  
  // Test float return types
  assert(float_eq(speed.get_rev_per_sec(), 1.0f));
  assert(float_eq(speed.get_radians_per_sec(), 2.0f * static_cast<float>(PI)));

  std::cout << "  ✓ 60 RPM = " << speed.get_rpm() << " RPM (int16)" << std::endl;
  std::cout << "  ✓ 60 RPM = " << speed.get_steps_per_sec() << " steps/s (int32)" << std::endl;
  std::cout << "  ✓ 60 RPM = " << speed.get_rev_per_sec() << " rev/s (float)" << std::endl;
  std::cout << "  ✓ 60 RPM = " << speed.get_degrees_per_sec() << " deg/s (int32)" << std::endl;
  std::cout << "  ✓ 60 RPM = " << speed.get_radians_per_sec() << " rad/s (float)" << std::endl;
  std::cout << "  ✓ 60 RPM = " << speed.get_degrees_per_min() << " deg/min (int32)" << std::endl;
  std::cout << "  ✓ 60 RPM = " << speed.get_degrees_per_hour() << " deg/h (int32)" << std::endl;
}

void test_speed_setters()
{
  std::cout << "Testing setter methods..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  Speed speed(&mock);
  assert(speed.rpm_internal() == 0);

  // Test set_rpm
  speed.set_rpm(100.0f);
  assert(speed.get_rpm() == 100);
  std::cout << "  ✓ set_rpm(100) → " << speed.get_rpm() << " RPM" << std::endl;

  // Test set_steps_per_sec
  speed.set_steps_per_sec(3200.0f);
  assert(speed.get_rpm() == 60);
  std::cout << "  ✓ set_steps_per_sec(3200) → " << speed.get_rpm() << " RPM" << std::endl;

  // Test set_rev_per_sec
  speed.set_rev_per_sec(2.0f);
  assert(speed.get_rpm() == 120);
  std::cout << "  ✓ set_rev_per_sec(2) → " << speed.get_rpm() << " RPM" << std::endl;

  // Test set_degrees_per_sec
  speed.set_degrees_per_sec(360.0f);
  assert(speed.get_rpm() == 60);
  std::cout << "  ✓ set_degrees_per_sec(360) → " << speed.get_rpm() << " RPM" << std::endl;

  // Test set_radians_per_sec
  const float two_pi = 2.0f * static_cast<float>(PI);
  speed.set_radians_per_sec(two_pi);
  assert(speed.get_rpm() == 60);
  std::cout << "  ✓ set_radians_per_sec(2π) → " << speed.get_rpm() << " RPM" << std::endl;

  // Test set_degrees_per_min
  speed.set_degrees_per_min(360.0f);
  assert(speed.get_rpm() == 60);
  std::cout << "  ✓ set_degrees_per_min(360) → " << speed.get_rpm() << " RPM" << std::endl;

  // Test set_degrees_per_hour
  speed.set_degrees_per_hour(21600.0f);
  assert(speed.get_rpm() == 60);
  std::cout << "  ✓ set_degrees_per_hour(21600) → " << speed.get_rpm() << " RPM" << std::endl;
}

void test_speed_int_overloads()
{
  std::cout << "Testing int constructor/setter overloads..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test int64_t constructor
  Speed speed1(static_cast<int64_t>(100), SpeedUnit::RPM, &mock);
  assert(speed1.get_rpm() == 100);
  std::cout << "  ✓ Speed(int64_t 100, RPM) → " << speed1.get_rpm() << " RPM" << std::endl;

  // Test int32_t constructor
  Speed speed2(static_cast<int32_t>(50), SpeedUnit::RPM, &mock);
  assert(speed2.get_rpm() == 50);
  std::cout << "  ✓ Speed(int32_t 50, RPM) → " << speed2.get_rpm() << " RPM" << std::endl;

  // Test int64_t setter
  Speed speed3(&mock);
  speed3.set_rpm(static_cast<int64_t>(100));
  assert(speed3.get_rpm() == 100);
  std::cout << "  ✓ set_rpm(int64_t 100) → " << speed3.get_rpm() << " RPM" << std::endl;

  // Test int32_t setter
  speed3.set_rpm(static_cast<int32_t>(50));
  assert(speed3.get_rpm() == 50);
  std::cout << "  ✓ set_rpm(int32_t 50) → " << speed3.get_rpm() << " RPM" << std::endl;
}

void test_speed_comparison_operators()
{
  std::cout << "Testing comparison operators..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  Speed speed1(100.0f, SpeedUnit::RPM, &mock);
  Speed speed2(100.0f, SpeedUnit::RPM, &mock);
  Speed speed3(50.0f, SpeedUnit::RPM, &mock);

  // Test equality
  assert(speed1 == speed2);
  assert(!(speed1 == speed3));
  std::cout << "  ✓ Equality: speed1 == speed2, speed1 != speed3" << std::endl;

  // Test inequality
  assert(speed1 != speed3);
  assert(!(speed1 != speed2));
  std::cout << "  ✓ Inequality: speed1 != speed3, !(speed1 != speed2)" << std::endl;
}

int main()
{
  std::cout << "\n=== Speed Class Unit Tests ===" << std::endl;
  std::cout << std::endl;

  // Basic unit conversions
  test_speed_steps_per_sec();
  test_speed_rpm();
  test_speed_rev_per_sec();
  test_speed_degrees_per_sec();
  test_speed_radians_per_sec();
  test_speed_degrees_per_min();
  test_speed_degrees_per_hour();

  // Hardware-specific
  test_speed_microstepping_compensation();
  test_speed_steps_per_sec_conversion();

  // Edge cases
  test_speed_negative_values();
  test_speed_range_clamping();
  test_speed_zero_value();

  // API pattern tests (matching Position/Acceleration)
  test_speed_factory_methods();
  test_speed_unit_conversions();
  test_speed_setters();
  test_speed_int_overloads();
  test_speed_comparison_operators();

  // Error handling
  test_speed_null_parent();
  test_speed_invalid_steps_per_revolution();

  std::cout << std::endl;
  std::cout << "✅ All Speed tests passed!" << std::endl;
  std::cout << std::endl;

  return 0;
}
