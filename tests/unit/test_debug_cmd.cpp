#include "../../components/servoxxd/stepper/servoxxd_commands.h"
#include <iostream>
#include <iomanip>

using namespace esphome::servoxxd;

int main() {
    Command cmd1(Commandtype::READ_PROTECTION_STATUS);
    Command cmd2(Commandtype::READ_MOTOR_STATUS);
    
    std::cout << "READ_PROTECTION_STATUS (0x" << std::hex << static_cast<int>(Commandtype::READ_PROTECTION_STATUS) 
              << ") → function_code: 0x" << static_cast<int>(cmd1.function_code()) << std::endl;
    
    std::cout << "READ_MOTOR_STATUS (0x" << std::hex << static_cast<int>(Commandtype::READ_MOTOR_STATUS)
              << ") → function_code: 0x" << static_cast<int>(cmd2.function_code()) << std::endl;
    
    std::cout << "\nREAD range check:" << std::endl;
    std::cout << "READ_ENCODER_CARRY = 0x" << std::hex << static_cast<int>(Commandtype::READ_ENCODER_CARRY) << std::endl;
    std::cout << "READ_ZERO_RETURN_STATUS = 0x" << std::hex << static_cast<int>(Commandtype::READ_ZERO_RETURN_STATUS) << std::endl;
    std::cout << "READ_PROTECTION_STATUS = 0x" << std::hex << static_cast<int>(Commandtype::READ_PROTECTION_STATUS) << std::endl;
    std::cout << "READ_MOTOR_STATUS = 0x" << std::hex << static_cast<int>(Commandtype::READ_MOTOR_STATUS) << std::endl;
    
    return 0;
}
