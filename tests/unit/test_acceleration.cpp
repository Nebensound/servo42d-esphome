/**
 * @file test_acceleration.cpp
 * @brief Unit tests for Acceleration class with all 5 unit conversions
 *
 * Tests verify:
 * - Conversion accuracy from all 5 units to hardware value (0-255)
 * - Inverse time mapping (non-linear encoding)
 * - Edge cases (zero, instant, max values)
 * - RPM/s approximation accuracy
 * - Steps/s² conversion for ESPHome base class
 */

#include "components/servoxxd_modbus/stepper/servoxxd_acceleration.h"
#include "components/servoxxd_modbus/stepper/servoxxd_modbus.h"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace esphome::servoxxd_modbus;

// Tolerance for float comparisons
constexpr float EPSILON = 1.0f; // Larger tolerance for non-linear mapping

bool float_eq(float a, float b, float epsilon = EPSILON)
{
  return std::abs(a - b) < epsilon;
}

// Mock ServoXxdModbus for testing
class MockServoXxdModbus : public ServoXxdModbus
{
public:
  explicit MockServoXxdModbus(float steps_per_rev) : steps_per_rev_(steps_per_rev) {}

  float get_steps_per_revolution() const override { return steps_per_rev_; }

private:
  float steps_per_rev_;
};

void test_acceleration_steps_per_sec_sq()
{
  std::cout << "Testing STEPS_PER_SEC_SQ conversion..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 1000 steps/s² should be (1000 * 60) / 3200 = 18.75 RPM/s
  // acc = 256 - (20000 / 18.75) = 256 - 1066.67 = -810.67 → clamped to 1
  Acceleration acc(1000.0f, AccelerationUnit::STEPS_PER_SEC_SQ, &mock);
  assert(acc.acc_internal() == 1); // Slowest acceleration

  std::cout << "  ✓ 1000 steps/s² → acc=" << (int)acc.acc_internal() << " (slowest)" << std::endl;
}

void test_acceleration_rpm_per_sec()
{
  std::cout << "Testing RPM_PER_SEC conversion (motor native)..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 1000 RPM/s should be acc = 256 - (20000 / 1000) = 256 - 20 = 236
  Acceleration acc(1000.0f, AccelerationUnit::RPM_PER_SEC, &mock);
  assert(acc.acc_internal() == 236);

  std::cout << "  ✓ 1000 RPM/s → acc=" << (int)acc.acc_internal() << std::endl;

  // Verify reverse conversion is approximate
  float rpm_per_s = acc.rpm_per_sec();
  assert(float_eq(rpm_per_s, 1000.0f));
  std::cout << "  ✓ Reverse: acc=236 → " << rpm_per_s << " RPM/s (expected ~1000)" << std::endl;
}

void test_acceleration_rev_per_sec_sq()
{
  std::cout << "Testing REV_PER_SEC_SQ conversion..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 10 rev/s² should be 10 * 60 = 600 RPM/s
  // acc = 256 - (20000 / 600) = 256 - 33.33 = 222.67 → 223
  Acceleration acc(10.0f, AccelerationUnit::REV_PER_SEC_SQ, &mock);
  assert(acc.acc_internal() >= 222 && acc.acc_internal() <= 223);

  std::cout << "  ✓ 10 rev/s² → acc=" << (int)acc.acc_internal() << " (expected ~223)" << std::endl;
}

void test_acceleration_degrees_per_sec_sq()
{
  std::cout << "Testing DEGREES_PER_SEC_SQ conversion..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 360 deg/s² should be (360 * 60) / 360 = 60 RPM/s
  // acc = 256 - (20000 / 60) = 256 - 333.33 = -77.33 → clamped to 1
  Acceleration acc(360.0f, AccelerationUnit::DEGREES_PER_SEC_SQ, &mock);
  assert(acc.acc_internal() == 1);

  std::cout << "  ✓ 360 deg/s² → acc=" << (int)acc.acc_internal() << " (slowest)" << std::endl;
}

void test_acceleration_radians_per_sec_sq()
{
  std::cout << "Testing RADIANS_PER_SEC_SQ conversion..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);
  constexpr float PI = 3.14159265358979323846f;
  constexpr float two_pi = 2.0f * PI;

  // Test: 2π rad/s² (1 rev/s²) should be 60 RPM/s
  Acceleration acc(two_pi, AccelerationUnit::RADIANS_PER_SEC_SQ, &mock);
  assert(acc.acc_internal() == 1); // Low acceleration → slowest

  std::cout << "  ✓ 2π rad/s² → acc=" << (int)acc.acc_internal() << std::endl;
}

void test_acceleration_instant()
{
  std::cout << "Testing instant acceleration (acc=0)..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 0 RPM/s should result in acc=0 (instant, no ramping)
  Acceleration acc_zero(0.0f, AccelerationUnit::RPM_PER_SEC, &mock);
  assert(acc_zero.acc_internal() == 0);
  std::cout << "  ✓ 0 RPM/s → acc=" << (int)acc_zero.acc_internal() << " (instant)" << std::endl;

  // Test: Very high acceleration should also result in acc=0
  Acceleration acc_high(25000.0f, AccelerationUnit::RPM_PER_SEC, &mock);
  assert(acc_high.acc_internal() == 0);
  std::cout << "  ✓ 25000 RPM/s → acc=" << (int)acc_high.acc_internal() << " (instant, > max)" << std::endl;

  // Verify reverse conversion returns -1.0f for instant
  float rpm_per_s = acc_zero.rpm_per_sec();
  assert(rpm_per_s == -1.0f); // Sentinel value for instant
  std::cout << "  ✓ acc=0 → rpm_per_sec=-1.0 (sentinel for instant)" << std::endl;
}

void test_acceleration_max_value()
{
  std::cout << "Testing maximum acceleration (acc=255)..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: acc=255 is the FASTEST ramping (not instant)
  // To get acc=255: 256 - (20000 / rpm_per_s) = 255 → rpm_per_s = 20000
  // But 20000 RPM/s is the hardware MAX, which maps to acc=0 (instant)!
  // So we need slightly less: rpm_per_s = 20000 / (256-255) = 20000 RPM/s
  // Actually, acc=255 means: 256 - 255 = 1 → Δt = 1×50μs → a = 20000/1 = 20000 RPM/s
  // But that's ≥ MAX, so it becomes instant (acc=0)

  // To get acc=255, we need: rpm_per_s = 20000 / (256-255) = 20000 / 1 = 20000
  // But 20000 ≥ MAX → instant. So acc=255 is actually unreachable via constructor!
  // The closest we can get is slightly below MAX: e.g. 19999 RPM/s

  // Test with 19999 RPM/s: acc = 256 - (20000/19999) ≈ 256 - 1.00005 ≈ 255
  Acceleration acc(19999.0f, AccelerationUnit::RPM_PER_SEC, &mock);
  assert(acc.acc_internal() == 255);

  std::cout << "  ✓ 19999 RPM/s → acc=" << (int)acc.acc_internal() << " (fastest non-instant)" << std::endl;

  // Test that exactly MAX_RPM_PER_SEC → instant (acc=0)
  Acceleration acc_max(20000.0f, AccelerationUnit::RPM_PER_SEC, &mock);
  assert(acc_max.acc_internal() == 0);
  std::cout << "  ✓ 20000 RPM/s (MAX) → acc=0 (instant)" << std::endl;
}

void test_acceleration_non_linear_mapping()
{
  std::cout << "Testing non-linear inverse time mapping..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test several points to verify non-linear relationship
  struct TestCase
  {
    float rpm_per_s;
    uint8_t expected_acc_min;
    uint8_t expected_acc_max;
  };

  TestCase cases[] = {
      {100.0f, 56, 56},    // 256 - (20000/100) = 56
      {500.0f, 216, 216},  // 256 - (20000/500) = 216
      {1000.0f, 236, 236}, // 256 - (20000/1000) = 236
      {5000.0f, 252, 252}, // 256 - (20000/5000) = 252
      {10000.0f, 254, 254} // 256 - (20000/10000) = 254
  };

  for (const auto &test : cases)
  {
    Acceleration acc(test.rpm_per_s, AccelerationUnit::RPM_PER_SEC, &mock);
    uint8_t acc_value = acc.acc_internal();
    assert(acc_value >= test.expected_acc_min && acc_value <= test.expected_acc_max);
    std::cout << "  ✓ " << test.rpm_per_s << " RPM/s → acc=" << (int)acc_value
              << " (expected " << (int)test.expected_acc_min << ")" << std::endl;
  }
}

void test_acceleration_steps_per_sec2_conversion()
{
  std::cout << "Testing steps_per_sec2() for ESPHome..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 1000 RPM/s = (1000 / 60) * 3200 = 53333.33 steps/s²
  Acceleration acc(1000.0f, AccelerationUnit::RPM_PER_SEC, &mock);
  float steps_per_s2 = acc.steps_per_sec2();
  assert(float_eq(steps_per_s2, 53333.33f, 10.0f));

  std::cout << "  ✓ 1000 RPM/s = " << steps_per_s2 << " steps/s² (expected ~53333)" << std::endl;

  // Test instant (acc=0) returns -1.0f
  Acceleration acc_instant(0.0f, AccelerationUnit::RPM_PER_SEC, &mock);
  float steps_instant = acc_instant.steps_per_sec2();
  assert(steps_instant == -1.0f);
  std::cout << "  ✓ Instant (acc=0) → -1.0 steps/s² (sentinel)" << std::endl;
}

void test_acceleration_boundary_values()
{
  std::cout << "Testing boundary clamping..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: Very low acceleration should clamp to 1 (slowest)
  Acceleration acc_low(10.0f, AccelerationUnit::RPM_PER_SEC, &mock);
  assert(acc_low.acc_internal() == 1);
  std::cout << "  ✓ 10 RPM/s → acc=1 (minimum non-instant)" << std::endl;

  // Test: Negative acceleration should result in acc=0 (instant)
  Acceleration acc_neg(-100.0f, AccelerationUnit::RPM_PER_SEC, &mock);
  assert(acc_neg.acc_internal() == 0);
  std::cout << "  ✓ -100 RPM/s → acc=0 (instant, invalid input)" << std::endl;
}

void test_acceleration_null_parent()
{
  std::cout << "Testing null parent pointer handling..." << std::endl;

  // Test: STEPS_PER_SEC_SQ with null parent should default to 0 (instant)
  Acceleration acc_null(1000.0f, AccelerationUnit::STEPS_PER_SEC_SQ, nullptr);
  assert(acc_null.acc_internal() == 0);
  std::cout << "  ✓ STEPS_PER_SEC_SQ with null parent → acc=0 (error handling)" << std::endl;

  // Test: Other units with null parent should work fine
  Acceleration acc_rpm_null(1000.0f, AccelerationUnit::RPM_PER_SEC, nullptr);
  assert(acc_rpm_null.acc_internal() == 236); // Normal calculation
  std::cout << "  ✓ RPM_PER_SEC with null parent → acc=236 (parent not needed)" << std::endl;
}

void test_acceleration_invalid_steps_per_revolution()
{
  std::cout << "Testing invalid steps_per_revolution..." << std::endl;

  // Mock with invalid (negative) steps_per_rev
  MockServoXxdModbus mock_invalid(-100.0f);

  // Should handle invalid steps_per_rev gracefully
  Acceleration acc(1000.0f, AccelerationUnit::STEPS_PER_SEC_SQ, &mock_invalid);
  assert(acc.acc_internal() == 0); // Should default to 0 (instant)
  std::cout << "  ✓ STEPS_PER_SEC_SQ with negative steps_per_rev → acc=0 (error handling)" << std::endl;

  // Mock with zero steps_per_rev
  MockServoXxdModbus mock_zero(0.0f);
  Acceleration acc_zero(1000.0f, AccelerationUnit::STEPS_PER_SEC_SQ, &mock_zero);
  assert(acc_zero.acc_internal() == 0);
  std::cout << "  ✓ STEPS_PER_SEC_SQ with zero steps_per_rev → acc=0 (error handling)" << std::endl;
}

int main()
{
  std::cout << "\n=== Acceleration Class Unit Tests ===" << std::endl;
  std::cout << std::endl;

  test_acceleration_steps_per_sec_sq();
  test_acceleration_rpm_per_sec();
  test_acceleration_rev_per_sec_sq();
  test_acceleration_degrees_per_sec_sq();
  test_acceleration_radians_per_sec_sq();
  test_acceleration_instant();
  test_acceleration_max_value();
  test_acceleration_non_linear_mapping();
  test_acceleration_steps_per_sec2_conversion();
  test_acceleration_boundary_values();
  test_acceleration_null_parent();
  test_acceleration_invalid_steps_per_revolution();

  std::cout << std::endl;
  std::cout << "✅ All Acceleration tests passed!" << std::endl;
  std::cout << std::endl;

  return 0;
}
