#include "components/servoxxd_modbus/stepper/servoxxd_speed.h"
#include <iostream>

using namespace esphome::servoxxd_modbus;

int main() {
  float steps_per_rev = 3200.0f;
  Speed speed(1000.0f, SpeedUnit::STEPS_PER_SEC, steps_per_rev);
  std::cout << "Expected: 18.75 RPM" << std::endl;
  std::cout << "Actual: " << speed.rpm() << " RPM" << std::endl;
  std::cout << "Internal: " << speed.rpm_as_i16() << " RPM (int16)" << std::endl;
  return 0;
}
