#include <iostream>
#include "../../components/servoxxd/stepper/servoxxd_position.h"

using namespace esphome::servoxxd;

int main() {
    Position pos = Position::from_steps(1000, nullptr);
    std::cout << "Input: 1000 steps" << std::endl;
    std::cout << "get_steps(): " << pos.get_steps() << std::endl;
    std::cout << "get_ticks(): " << pos.get_ticks() << std::endl;
    return 0;
}
