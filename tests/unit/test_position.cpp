/**
 * @file test_position.cpp
 * @brief Unit tests for Position class with all 6 unit conversions
 *
 * Tests verify:
 * - Conversion accuracy from all 6 units to encoder ticks (split format)
 * - Split format (revolutions + angle_ticks) handling
 * - Arithmetic operators (+, -, ==)
 * - Edge cases (zero, negative, fractional revolutions)
 * - Steps conversion for ESPHome base class
 * - Carry/borrow behavior in split format
 */

#include "components/servoxxd_modbus/stepper/servoxxd_position.h"
#include "components/servoxxd_modbus/stepper/servoxxd_modbus.h"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace esphome::servoxxd_modbus;

// Tolerance for float comparisons
// Position has tick quantization: 1 tick = 360° / 16384 ≈ 0.022°
// So we need larger epsilon than Speed/Acceleration
constexpr float EPSILON = 0.025f;

bool float_eq(float a, float b)
{
  return std::abs(a - b) < EPSILON;
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

void test_position_steps()
{
  std::cout << "Testing STEPS conversion..." << std::endl;

  float steps_per_rev = 3200.0f;

  MockServoXxdModbus mock(steps_per_rev);

  // Test: 3200 steps = 1 revolution = 16384 ticks
  Position pos(3200.0f, PositionUnit::STEPS, &mock);
  assert(pos.revolutions() == 1);
  assert(pos.angle_ticks() == 0);
  assert(pos.ticks_total() == 16384);

  std::cout << "  ✓ 3200 steps = " << pos.revolutions() << " rev + " << pos.angle_ticks() << " ticks" << std::endl;

  // Test reverse conversion
  int32_t steps = pos.steps();
  assert(steps == 3200);
  std::cout << "  ✓ Reverse: 1 rev = " << steps << " steps" << std::endl;
}

void test_position_revolutions()
{
  std::cout << "Testing REVOLUTIONS conversion..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 2.5 revolutions = 2 rev + 0.5 rev = 2 rev + 8192 ticks
  Position pos(2.5f, PositionUnit::REVOLUTIONS, &mock);
  assert(pos.revolutions() == 2);
  assert(pos.angle_ticks() == 8192);

  std::cout << "  ✓ 2.5 rev = " << pos.revolutions() << " rev + " << pos.angle_ticks() << " ticks" << std::endl;

  // Test fractional conversion
  float degrees = pos.degrees();
  assert(float_eq(degrees, 900.0f)); // 2.5 * 360 = 900°
  std::cout << "  ✓ 2.5 rev = " << degrees << " degrees" << std::endl;
}

void test_position_degrees()
{
  std::cout << "Testing DEGREES conversion..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 720° = 2 revolutions = 32768 ticks
  Position pos(720.0f, PositionUnit::DEGREES, &mock);
  assert(pos.revolutions() == 2);
  assert(pos.angle_ticks() == 0);

  std::cout << "  ✓ 720° = " << pos.revolutions() << " rev + " << pos.angle_ticks() << " ticks" << std::endl;

  // Test fractional degree: 45° = 0.125 rev = 2048 ticks
  Position pos_45(45.0f, PositionUnit::DEGREES, &mock);
  assert(pos_45.revolutions() == 0);
  assert(pos_45.angle_ticks() == 2048);
  std::cout << "  ✓ 45° = " << pos_45.revolutions() << " rev + " << pos_45.angle_ticks() << " ticks" << std::endl;
}

void test_position_radians()
{
  std::cout << "Testing RADIANS conversion..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);
  constexpr float PI = 3.14159265358979323846f;
  constexpr float two_pi = 2.0f * PI;

  // Test: 2π radians = 1 revolution = 16384 ticks
  Position pos(two_pi, PositionUnit::RADIANS, &mock);
  assert(pos.revolutions() == 1);
  assert(pos.angle_ticks() == 0);

  std::cout << "  ✓ 2π rad = " << pos.revolutions() << " rev + " << pos.angle_ticks() << " ticks" << std::endl;

  // Test reverse conversion
  float radians = pos.radians();
  assert(float_eq(radians, two_pi));
  std::cout << "  ✓ Reverse: 1 rev = " << radians << " rad (expected " << two_pi << ")" << std::endl;
}

void test_position_arcminutes()
{
  std::cout << "Testing ARCMINUTES conversion..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 21600 arcmin = 360° = 1 revolution
  Position pos(21600.0f, PositionUnit::ARCMINUTES, &mock);
  assert(pos.revolutions() == 1);
  assert(pos.angle_ticks() == 0);

  std::cout << "  ✓ 21600 arcmin = " << pos.revolutions() << " rev + " << pos.angle_ticks() << " ticks" << std::endl;

  // Test: 60 arcmin = 1° = 1/360 rev
  Position pos_60(60.0f, PositionUnit::ARCMINUTES, &mock);
  float degrees = pos_60.degrees();
  assert(float_eq(degrees, 1.0f));
  std::cout << "  ✓ 60 arcmin = " << degrees << "° (expected 1)" << std::endl;
}

void test_position_arcseconds()
{
  std::cout << "Testing ARCSECONDS conversion..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: 1296000 arcsec = 360° = 1 revolution
  Position pos(1296000.0f, PositionUnit::ARCSECONDS, &mock);
  assert(pos.revolutions() == 1);
  assert(pos.angle_ticks() == 0);

  std::cout << "  ✓ 1296000 arcsec = " << pos.revolutions() << " rev + " << pos.angle_ticks() << " ticks" << std::endl;

  // Test: 3600 arcsec = 1° = 1/360 rev
  Position pos_3600(3600.0f, PositionUnit::ARCSECONDS, &mock);
  float degrees = pos_3600.degrees();
  assert(float_eq(degrees, 1.0f));
  std::cout << "  ✓ 3600 arcsec = " << degrees << "° (expected 1)" << std::endl;
}

void test_position_split_format()
{
  std::cout << "Testing split format construction..." << std::endl;

  // Test: Create from parts
  Position pos = Position::from_parts(3, 8192); // 3.5 revolutions
  assert(pos.revolutions() == 3);
  assert(pos.angle_ticks() == 8192);

  int64_t total = pos.ticks_total();
  assert(total == 3 * 16384 + 8192); // 57344
  std::cout << "  ✓ from_parts(3, 8192) = " << total << " total ticks" << std::endl;

  // Test: Create from total ticks
  Position pos2 = Position::from_ticks_total(57344);
  assert(pos2.revolutions() == 3);
  assert(pos2.angle_ticks() == 8192);
  std::cout << "  ✓ from_ticks_total(57344) = " << pos2.revolutions() << " rev + " << pos2.angle_ticks() << " ticks" << std::endl;
}

void test_position_carry_borrow()
{
  std::cout << "Testing carry/borrow behavior..." << std::endl;

  // Test: Overflow in angle_ticks (≥16384) should carry to revolutions
  Position pos = Position::from_parts(0, 16384 + 100); // Should normalize to 1 rev + 100 ticks
  assert(pos.revolutions() == 1);
  assert(pos.angle_ticks() == 100);
  std::cout << "  ✓ Overflow: (0, 16484) → " << pos.revolutions() << " rev + " << pos.angle_ticks() << " ticks" << std::endl;

  // Test: Large overflow
  Position pos2 = Position::from_parts(0, 32768 + 200); // Should normalize to 2 rev + 200 ticks
  assert(pos2.revolutions() == 2);
  assert(pos2.angle_ticks() == 200);
  std::cout << "  ✓ Large overflow: (0, 32968) → " << pos2.revolutions() << " rev + " << pos2.angle_ticks() << " ticks" << std::endl;
}

void test_position_negative_values()
{
  std::cout << "Testing negative position values..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Test: -1 revolution = -16384 ticks
  Position pos(-1.0f, PositionUnit::REVOLUTIONS, &mock);
  assert(pos.revolutions() == -1);
  assert(pos.angle_ticks() == 0);

  std::cout << "  ✓ -1 rev = " << pos.revolutions() << " rev + " << pos.angle_ticks() << " ticks" << std::endl;

  // Test: -0.5 revolution should be -1 rev + 8192 ticks
  Position pos_half(-0.5f, PositionUnit::REVOLUTIONS, &mock);
  assert(pos_half.revolutions() == -1);
  assert(pos_half.angle_ticks() == 8192);
  std::cout << "  ✓ -0.5 rev = " << pos_half.revolutions() << " rev + " << pos_half.angle_ticks() << " ticks" << std::endl;
}

void test_position_arithmetic()
{
  std::cout << "Testing arithmetic operators..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  Position pos1(1.0f, PositionUnit::REVOLUTIONS, &mock); // 1 rev
  Position pos2(0.5f, PositionUnit::REVOLUTIONS, &mock); // 0.5 rev

  // Test addition: 1 + 0.5 = 1.5 rev
  Position sum = pos1 + pos2;
  assert(sum.revolutions() == 1);
  assert(sum.angle_ticks() == 8192);
  std::cout << "  ✓ 1 rev + 0.5 rev = " << sum.revolutions() << " rev + " << sum.angle_ticks() << " ticks" << std::endl;

  // Test subtraction: 1 - 0.5 = 0.5 rev
  Position diff = pos1 - pos2;
  assert(diff.revolutions() == 0);
  assert(diff.angle_ticks() == 8192);
  std::cout << "  ✓ 1 rev - 0.5 rev = " << diff.revolutions() << " rev + " << diff.angle_ticks() << " ticks" << std::endl;

  // Test equality
  Position pos3(1.0f, PositionUnit::REVOLUTIONS, &mock);
  assert(pos1 == pos3);
  assert(!(pos1 == pos2));
  std::cout << "  ✓ Equality: pos1 == pos3, pos1 != pos2" << std::endl;
}

void test_position_zero()
{
  std::cout << "Testing zero position..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  Position pos(0.0f, PositionUnit::REVOLUTIONS, &mock);
  assert(pos.revolutions() == 0);
  assert(pos.angle_ticks() == 0);
  assert(pos.ticks_total() == 0);

  std::cout << "  ✓ 0 rev = " << pos.revolutions() << " rev + " << pos.angle_ticks() << " ticks" << std::endl;
}

void test_position_unit_conversions()
{
  std::cout << "Testing all unit accessor methods..." << std::endl;

  float steps_per_rev = 3200.0f;
  MockServoXxdModbus mock(steps_per_rev);

  // Create 1 revolution position
  Position pos(1.0f, PositionUnit::REVOLUTIONS, &mock);

  constexpr float PI = 3.14159265358979323846f;
  assert(pos.steps() == 3200);
  assert(float_eq(pos.degrees(), 360.0f));
  assert(float_eq(pos.radians(), 2.0f * PI));
  assert(float_eq(pos.arcminutes(), 21600.0f));
  assert(float_eq(pos.arcseconds(), 1296000.0f));

  std::cout << "  ✓ 1 rev = " << pos.steps() << " steps" << std::endl;
  std::cout << "  ✓ 1 rev = " << pos.degrees() << " degrees" << std::endl;
  std::cout << "  ✓ 1 rev = " << pos.radians() << " radians" << std::endl;
  std::cout << "  ✓ 1 rev = " << pos.arcminutes() << " arcmin" << std::endl;
  std::cout << "  ✓ 1 rev = " << pos.arcseconds() << " arcsec" << std::endl;
}

void test_position_null_parent()
{
  std::cout << "Testing null parent pointer handling..." << std::endl;

  // Test: STEPS with null parent should default to 0
  Position pos_null(3200.0f, PositionUnit::STEPS, nullptr);
  assert(pos_null.revolutions() == 0);
  assert(pos_null.angle_ticks() == 0);
  std::cout << "  ✓ STEPS with null parent → 0 position (error handling)" << std::endl;

  // Test: Other units with null parent should work fine
  Position pos_rev_null(1.0f, PositionUnit::REVOLUTIONS, nullptr);
  assert(pos_rev_null.revolutions() == 1);
  assert(pos_rev_null.angle_ticks() == 0);
  std::cout << "  ✓ REVOLUTIONS with null parent → 1 rev (parent not needed)" << std::endl;
}

void test_position_invalid_steps_per_revolution()
{
  std::cout << "Testing invalid steps_per_revolution..." << std::endl;

  // Mock with invalid (negative) steps_per_rev
  MockServoXxdModbus mock_invalid(-100.0f);

  // Should handle invalid steps_per_rev gracefully
  Position pos(3200.0f, PositionUnit::STEPS, &mock_invalid);
  assert(pos.revolutions() == 0);
  assert(pos.angle_ticks() == 0);
  std::cout << "  ✓ STEPS with negative steps_per_rev → 0 position (error handling)" << std::endl;

  // Mock with zero steps_per_rev
  MockServoXxdModbus mock_zero(0.0f);
  Position pos_zero(3200.0f, PositionUnit::STEPS, &mock_zero);
  assert(pos_zero.revolutions() == 0);
  assert(pos_zero.angle_ticks() == 0);
  std::cout << "  ✓ STEPS with zero steps_per_rev → 0 position (error handling)" << std::endl;
}

void test_position_large_values()
{
  std::cout << "Testing large position values..." << std::endl;

  MockServoXxdModbus mock(3200.0f);

  // Test: Large positive revolutions
  Position pos_large(10000.0f, PositionUnit::REVOLUTIONS, &mock);
  assert(pos_large.revolutions() == 10000);
  assert(pos_large.angle_ticks() == 0);
  std::cout << "  ✓ 10000 revolutions = 10000 rev" << std::endl;

  // Test: Large negative revolutions
  Position pos_neg_large(-10000.0f, PositionUnit::REVOLUTIONS, &mock);
  assert(pos_neg_large.revolutions() == -10000);
  assert(pos_neg_large.angle_ticks() == 0);
  std::cout << "  ✓ -10000 revolutions = -10000 rev" << std::endl;
}

int main()
{
  std::cout << "\n=== Position Class Unit Tests ===" << std::endl;
  std::cout << std::endl;

  test_position_steps();
  test_position_revolutions();
  test_position_degrees();
  test_position_radians();
  test_position_arcminutes();
  test_position_arcseconds();
  test_position_split_format();
  test_position_carry_borrow();
  test_position_negative_values();
  test_position_arithmetic();
  test_position_zero();
  test_position_unit_conversions();
  test_position_null_parent();
  test_position_invalid_steps_per_revolution();
  test_position_large_values();

  std::cout << std::endl;
  std::cout << "✅ All Position tests passed!" << std::endl;
  std::cout << std::endl;

  return 0;
}
