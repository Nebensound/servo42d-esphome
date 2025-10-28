#include "components/servoxxd_modbus/stepper/servoxxd_acceleration.h"
#include <iostream>

using namespace esphome::servoxxd_modbus;

int main() {
  float steps_per_rev = 3200.0f;
  Acceleration acc(20000.0f, AccelerationUnit::RPM_PER_SEC, steps_per_rev);
  std::cout << "20000 RPM/s → acc=" << (int)acc.acc_internal() << " (expected 255)" << std::endl;
  std::cout << "Formula: 256 - (20000 / 20000) = " << (256 - (20000/20000)) << std::endl;
  return 0;
}
