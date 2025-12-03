#pragma once

// Mock ESPHome helpers for unit testing

#include <cstdint>
#include <chrono>

namespace esphome
{

  // Mock millis() function for timing
  inline uint32_t millis()
  {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(duration).count());
  }

  // Mock micros() function for precision timing
  inline uint32_t micros()
  {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(duration).count());
  }

} // namespace esphome
