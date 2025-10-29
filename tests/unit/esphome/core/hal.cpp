/**
 * @file hal.cpp
 * @brief Mock HAL implementation for unit tests
 */

#include "hal.h"
#include <chrono>

// Simple millis() implementation for unit tests
// Returns milliseconds since epoch (not since boot, but sufficient for testing)
uint32_t millis() {
  auto now = std::chrono::steady_clock::now();
  auto duration = now.time_since_epoch();
  auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
  return static_cast<uint32_t>(millis.count());
}
