#pragma once

#include <cstdint>

namespace esphome {
namespace servo42d_rs485 {

// INT48 Support Structure
struct Int48 {
  int32_t carry;    // Upper 32 bits
  uint16_t value;   // Lower 16 bits
  
  // Constructor from 48-bit value
  Int48(int64_t val = 0) {
    carry = static_cast<int32_t>(val >> 16);
    value = static_cast<uint16_t>(val & 0xFFFF);
  }
  
  // Convert to 64-bit integer
  int64_t to_int64() const {
    return (static_cast<int64_t>(carry) << 16) | value;
  }
  
  // Convert from register array (3 registers: carry high, carry low, value)
  static Int48 from_registers(const uint16_t* regs) {
    int32_t full_carry = (static_cast<int32_t>(regs[0]) << 16) | regs[1];
    return Int48((static_cast<int64_t>(full_carry) << 16) | regs[2]);
  }
  
  // Convert to register array (3 registers)
  void to_registers(uint16_t* regs) const {
    regs[0] = static_cast<uint16_t>(carry >> 16);
    regs[1] = static_cast<uint16_t>(carry & 0xFFFF);
    regs[2] = value;
  }
};

// Position Mode Parameter Structures
struct PositionParams1 {
  uint16_t direction;     // 0=CW, 1=CCW
  uint16_t acceleration;  // Acceleration value
  uint16_t speed;         // Speed in RPM
  uint16_t pulses;        // Number of pulses
};

struct PositionParams2 {
  uint16_t acceleration;  // Acceleration value
  uint16_t speed;         // Speed in RPM
  uint32_t abs_pulses;    // Absolute pulse position (2 registers)
};

struct PositionParams3 {
  uint16_t acceleration;  // Acceleration value
  uint16_t speed;         // Speed in RPM
  int32_t rel_axis;       // Relative axis position (2 registers)
};

struct PositionParams4 {
  uint16_t acceleration;  // Acceleration value
  uint16_t speed;         // Speed in RPM
  int32_t abs_axis;       // Absolute axis position (2 registers)
};

struct ZeroModeParams {
  uint16_t mode;          // 0=single turn, 1=multiple turns
  uint16_t enable;        // 0=disable, 1=enable
  uint16_t speed;         // Speed for zeroing
  uint16_t direction;     // 0=CW, 1=CCW
};

} // namespace servo42d_rs485
} // namespace esphome