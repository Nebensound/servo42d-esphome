#include <iostream>
#include <iomanip>
#include "../../components/servoxxd/stepper/servoxxd_position.h"
#include "../../components/servoxxd/stepper/servoxxd_command_codec.h"

using namespace esphome::servoxxd;

int main() {
    Position pos = Position::from_ticks(1000);
    std::cout << "Position created from 1000 ticks" << std::endl;
    std::cout << "get_steps(): " << pos.get_steps() << std::endl;
    std::cout << "get_ticks(): " << pos.get_ticks() << std::endl;
    
    // Test encoding
    Speed spd = Speed::from_rpm(100, nullptr);
    Acceleration acc = Acceleration::from_internal(50);
    auto data = ServoCommandCodec::encode_move_position_mode_2(pos, spd, acc);
    
    std::cout << "\nEncoded bytes:" << std::endl;
    for(size_t i = 0; i < data.size(); i++) {
        std::cout << "  [" << i << "] = 0x" << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(data[i]) << std::dec << " (" << static_cast<int>(data[i]) << ")" << std::endl;
    }
    return 0;
}
