#include "servo42d_int48_t.hpp"

std::ostream &operator<<(std::ostream &stream, const int48_t &obj)
{
  stream << obj._s;
  return stream;
}

bool operator==(const std::int64_t i, const int48_t &obj)
{
  return i == obj._s;
}

bool operator==(const int48_t &obj, const std::int64_t i)
{
  return obj._s == i;
}

bool operator!=(const std::int64_t i, const int48_t &obj)
{
  return i != obj._s;
}

bool operator!=(const int48_t &obj, const std::int64_t i)
{
  return obj._s != i;
}

namespace std
{
  template <>
  struct hash<int48_t>
  {
    size_t operator()(const int48_t &obj) const
    {
      std::hash<std::int64_t> _helper;
      return _helper(static_cast<std::int64_t>(obj));
    }
  };
}
