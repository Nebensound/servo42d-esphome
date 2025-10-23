#pragma once

#include <cstdint>
#include <functional>
#include <ostream>

/**
 * @brief A class to represent a 48-bit signed integer.
 *
 * This class provides functionality for 48-bit integers with operator overloads
 * for arithmetic, bitwise, and comparison operations, as well as casting and streaming.
 */
class int48_t final
{
public:
  // Default constructors
  int48_t() : _s(0) {}
  int48_t(const int48_t &obj) = default;
  int48_t(int48_t &&obj) noexcept = default;

  // Converter constructors
  explicit int48_t(const std::int64_t i) : _s(static_cast<std::int64_t>(i & 0xFFFFFFFFFFFF)) {}
  explicit int48_t(const std::uint64_t i) : _s(static_cast<std::int64_t>(i & 0xFFFFFFFFFFFF)) {}

  // Default assignments
  int48_t &operator=(const int48_t &obj) = default;
  int48_t &operator=(int48_t &&obj) noexcept = default;

  // Converter assignments
  int48_t &operator=(std::int64_t i)
  {
    _s = static_cast<std::int64_t>(i & 0xFFFFFFFFFFFF);
    return *this;
  }

  int48_t &operator=(int i)
  {
    _s = static_cast<std::int64_t>(i & 0xFFFFFFFFFFFF);
    return *this;
  }

  int48_t &operator=(std::uint64_t i)
  {
    _s = static_cast<std::int64_t>(i & 0xFFFFFFFFFFFF);
    return *this;
  }

  // Arithmetic operators
  int48_t operator+(const int48_t &other) const { return int48_t((_s + other._s) & 0xFFFFFFFFFFFF); }
  int48_t &operator+=(const int48_t &other)
  {
    _s = (_s + other._s) & 0xFFFFFFFFFFFF;
    return *this;
  }

  int48_t operator-(const int48_t &other) const { return int48_t((_s - other._s) & 0xFFFFFFFFFFFF); }
  int48_t &operator-=(const int48_t &other)
  {
    _s = (_s - other._s) & 0xFFFFFFFFFFFF;
    return *this;
  }

  int48_t operator*(const int48_t &other) const { return int48_t((_s * other._s) & 0xFFFFFFFFFFFF); }
  int48_t &operator*=(const int48_t &other)
  {
    _s = (_s * other._s) & 0xFFFFFFFFFFFF;
    return *this;
  }

  int48_t operator/(const int48_t &other) const { return int48_t((_s / other._s) & 0xFFFFFFFFFFFF); }
  int48_t &operator/=(const int48_t &other)
  {
    _s = (_s / other._s) & 0xFFFFFFFFFFFF;
    return *this;
  }

  int48_t operator%(const int48_t &other) const { return int48_t((_s % other._s) & 0xFFFFFFFFFFFF); }
  int48_t &operator%=(const int48_t &other)
  {
    _s = (_s % other._s) & 0xFFFFFFFFFFFF;
    return *this;
  }

  // Bitwise operators
  int48_t operator&(const int48_t &other) const { return int48_t(_s & other._s); }
  int48_t &operator&=(const int48_t &other)
  {
    _s &= other._s;
    return *this;
  }

  int48_t operator|(const int48_t &other) const { return int48_t(_s | other._s); }
  int48_t &operator|=(const int48_t &other)
  {
    _s |= other._s;
    return *this;
  }

  int48_t operator|(unsigned char other) const { return int48_t(_s | static_cast<std::int64_t>(other)); }
  int48_t &operator|=(unsigned char other)
  {
    _s |= static_cast<std::int64_t>(other);
    return *this;
  }

  int48_t operator^(const int48_t &other) const { return int48_t(_s ^ other._s); }
  int48_t &operator^=(const int48_t &other)
  {
    _s ^= other._s;
    return *this;
  }

  int48_t operator<<(int shift) const { return int48_t((_s << shift) & 0xFFFFFFFFFFFF); }
  int48_t &operator<<=(int shift)
  {
    _s = (_s << shift) & 0xFFFFFFFFFFFF;
    return *this;
  }

  int48_t operator>>(int shift) const { return int48_t(_s >> shift); }
  int48_t &operator>>=(int shift)
  {
    _s >>= shift;
    return *this;
  }

  int48_t operator~() const { return int48_t(~_s & 0xFFFFFFFFFFFF); }
  int48_t operator-() const { return int48_t((-_s) & 0xFFFFFFFFFFFF); }

  // Comparison operators
  bool operator==(const int48_t &obj) const { return _s == obj._s; }
  bool operator!=(const int48_t &obj) const { return _s != obj._s; }
  bool operator<(const int48_t &obj) const { return _s < obj._s; }
  bool operator<=(const int48_t &obj) const { return _s <= obj._s; }
  bool operator>(const int48_t &obj) const { return _s > obj._s; }
  bool operator>=(const int48_t &obj) const { return _s >= obj._s; }

  // Cast operators
  explicit operator std::int64_t() const { return _s; }
  explicit operator double() const { return static_cast<double>(_s); }
  explicit operator float() const { return static_cast<float>(_s); }
  explicit operator std::uint64_t() const { return static_cast<std::uint64_t>(_s); }

  // Friend functions for comparisons with int64_t
  friend bool operator==(const std::int64_t i, const int48_t &obj);
  friend bool operator==(const int48_t &obj, const std::int64_t i);
  friend bool operator!=(const std::int64_t i, const int48_t &obj);
  friend bool operator!=(const int48_t &obj, const std::int64_t i);

  // Printing
  friend std::ostream &operator<<(std::ostream &stream, const int48_t &obj);

private:
  std::int64_t _s : 48;
} __attribute__((packed));

// Compile-time checks
static_assert(sizeof(int48_t) == 6, "size of int48_t is not 48bit, check your compiler!");
static_assert(sizeof(int48_t[2]) == 12, "size of int48_t[2] is not 2*48bit, check your compiler!");
static_assert(sizeof(int48_t[3]) == 18, "size of int48_t[3] is not 3*48bit, check your compiler!");
static_assert(sizeof(int48_t[4]) == 24, "size of int48_t[4] is not 4*48bit, check your compiler!");
