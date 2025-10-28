#include "components/servoxxd_modbus/stepper/servoxxd_position.h"
#include <iostream>

using namespace esphome::servoxxd_modbus;

int main() {
  float steps_per_rev = 3200.0f;
  Position pos_60(60.0f, PositionUnit::ARCMINUTES, steps_per_rev);
  std::cout << "60 arcmin:" << std::endl;
  std::cout << "  revolutions=" << pos_60.revolutions() << std::endl;
  std::cout << "  angle_ticks=" << pos_60.angle_ticks() << std::endl;
  std::cout << "  degrees()=" << pos_60.degrees() << std::endl;
  std::cout << "  Expected: 1.0°" << std::endl;
  std::cout << "  Formula: 60 arcmin / 21600 arcmin/rev = " << (60.0/21600.0) << " rev" << std::endl;
  std::cout << "  In ticks: " << (60.0/21600.0) << " × 16384 = " << ((60.0/21600.0) * 16384) << " ticks" << std::endl;
  return 0;
}
