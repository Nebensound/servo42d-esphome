#include <cmath>
#include <iostream>

constexpr float EPSILON = 0.01f;

bool float_eq(float a, float b) {
  return std::abs(a - b) < EPSILON;
}

int main() {
  std::cout << "Testing 1.01074 vs 1.0 with EPSILON=" << EPSILON << std::endl;
  std::cout << "  abs(1.01074 - 1.0) = " << std::abs(1.01074f - 1.0f) << std::endl;
  std::cout << "  Is < EPSILON? " << (std::abs(1.01074f - 1.0f) < EPSILON ? "YES" : "NO") << std::endl;
  std::cout << "  float_eq() = " << (float_eq(1.01074f, 1.0f) ? "true" : "false") << std::endl;
  return 0;
}
