#include "servoxxd_position.h"
#include "servoxxd_modbus.h"
#include <cmath>

namespace esphome
{
  namespace servoxxd_modbus
  {

    static const char *const TAG = "servoxxd_modbus.position";

    // Mathematical constants (C++11 compatible)
    constexpr float PI = 3.14159265358979323846f;
    constexpr float TWO_PI = 2.0f * PI;

    /**
     * @brief Convert position unit to string for logging
     */
    static const char *unit_to_string(PositionUnit unit)
    {
      switch (unit)
      {
      case PositionUnit::STEPS:
        return "steps";
      case PositionUnit::REVOLUTIONS:
        return "rev";
      case PositionUnit::DEGREES:
        return "deg";
      case PositionUnit::RADIANS:
        return "rad";
      case PositionUnit::ARCMINUTES:
        return "arcmin";
      case PositionUnit::ARCSECONDS:
        return "arcsec";
      default:
        return "unknown";
      }
    }

    Position::Position(float value, PositionUnit unit, const ServoXxdModbus *parent)
    {
      ESP_LOGV(TAG, "Creating Position: value=%.3f %s, parent=%p", value, unit_to_string(unit),
               static_cast<const void *>(parent));

      // Validate parent pointer for STEPS unit
      if (unit == PositionUnit::STEPS)
      {
        if (parent == nullptr)
        {
          ESP_LOGE(TAG, "Parent pointer is null (required for STEPS unit conversion)");
          return; // Leave position at zero
        }

        float steps_per_revolution = parent->get_steps_per_revolution();
        if (steps_per_revolution <= 0.0f)
        {
          ESP_LOGE(TAG, "Invalid steps_per_revolution: %.1f (must be > 0)", steps_per_revolution);
          return; // Leave position at zero
        }
      }

      // Convert to total ticks based on unit
      int64_t total_ticks = 0;

      switch (unit)
      {
      case PositionUnit::STEPS:
      {
        // STEPS: value is motor steps, convert using steps_per_revolution from parent
        // total_ticks = (value × 16384) / steps_per_revolution
        float steps_per_revolution = parent->get_steps_per_revolution();
        total_ticks = static_cast<int64_t>((value * TICKS_PER_REV) / steps_per_revolution);
        break;
      }

      case PositionUnit::REVOLUTIONS:
      {
        // Convert revolutions to encoder ticks
        // total_ticks = revolutions × 16384
        total_ticks = static_cast<int64_t>(roundf(value * static_cast<float>(TICKS_PER_REV)));
        break;
      }

      case PositionUnit::DEGREES:
      {
        // Convert degrees to encoder ticks
        // total_ticks = (degrees / 360) × 16384
        float revolutions = value / 360.0f;
        total_ticks = static_cast<int64_t>(roundf(revolutions * static_cast<float>(TICKS_PER_REV)));
        break;
      }

      case PositionUnit::RADIANS:
      {
        // Convert radians to encoder ticks
        // total_ticks = (radians / 2π) × 16384
        float revolutions = value / TWO_PI;
        total_ticks = static_cast<int64_t>(roundf(revolutions * static_cast<float>(TICKS_PER_REV)));
        break;
      }

      case PositionUnit::ARCMINUTES:
      {
        // Convert arcminutes to encoder ticks
        // 1 revolution = 360° = 21600 arcmin
        // total_ticks = (arcmin / 21600) × 16384
        float revolutions = value / 21600.0f;
        total_ticks = static_cast<int64_t>(roundf(revolutions * static_cast<float>(TICKS_PER_REV)));
        break;
      }

      case PositionUnit::ARCSECONDS:
      {
        // Convert arcseconds to encoder ticks
        // 1 revolution = 360° = 1296000 arcsec
        // total_ticks = (arcsec / 1296000) × 16384
        float revolutions = value / 1296000.0f;
        total_ticks = static_cast<int64_t>(roundf(revolutions * static_cast<float>(TICKS_PER_REV)));
        break;
      }

      default:
        ESP_LOGE(TAG, "Unknown PositionUnit: %d", static_cast<int>(unit));
        total_ticks = 0;
        break;
      }

      ESP_LOGV(TAG, "Converted to total ticks: %lld", total_ticks);

      // Step 2: Split total_ticks into revolutions and angle_ticks
      // revs = total_ticks / 16384 (integer division with sign)
      // angle_ticks = total_ticks % 16384 (always positive 0-16383)

      revs_ = static_cast<int32_t>(total_ticks / static_cast<int64_t>(TICKS_PER_REV));
      int64_t remainder = total_ticks % static_cast<int64_t>(TICKS_PER_REV);

      // Handle negative remainder (C++ modulo can be negative)
      if (remainder < 0)
      {
        revs_--;
        remainder += static_cast<int64_t>(TICKS_PER_REV);
      }

      angle_ticks_ = static_cast<uint16_t>(remainder);

      ESP_LOGV(TAG, "Split format: revs=%d, angle_ticks=%u", revs_, angle_ticks_);
    }

    Position Position::from_ticks_total(int64_t total_ticks)
    {
      Position pos;

      // Split total_ticks into revolutions and angle_ticks
      pos.revs_ = static_cast<int32_t>(total_ticks / static_cast<int64_t>(TICKS_PER_REV));
      int64_t remainder = total_ticks % static_cast<int64_t>(TICKS_PER_REV);

      // Handle negative remainder
      if (remainder < 0)
      {
        pos.revs_--;
        remainder += static_cast<int64_t>(TICKS_PER_REV);
      }

      pos.angle_ticks_ = static_cast<uint16_t>(remainder);

      ESP_LOGV(TAG, "from_ticks_total(%lld) → revs=%d, angle_ticks=%u", total_ticks, pos.revs_, pos.angle_ticks_);

      return pos;
    }

    Position Position::from_parts(int32_t revolutions, uint16_t angle_ticks)
    {
      Position pos;
      pos.revs_ = revolutions;
      pos.angle_ticks_ = angle_ticks;

      // Normalize to ensure angle_ticks is in [0, 16383]
      pos.normalize();

      ESP_LOGV(TAG, "from_parts(%d, %u) → revs=%d, angle_ticks=%u (after normalize)", revolutions, angle_ticks, pos.revs_,
               pos.angle_ticks_);

      return pos;
    }

    int64_t Position::ticks_total() const
    {
      // total_ticks = (revs × 16384) + angle_ticks
      int64_t total = static_cast<int64_t>(revs_) * static_cast<int64_t>(TICKS_PER_REV) + static_cast<int64_t>(angle_ticks_);
      return total;
    }

    int32_t Position::steps(const ServoXxdModbus *parent) const
    {
      if (parent == nullptr)
      {
        ESP_LOGE(TAG, "steps() called with null parent pointer");
        return 0;
      }

      float steps_per_rev = parent->get_steps_per_revolution();
      if (steps_per_rev <= 0)
      {
        ESP_LOGE(TAG, "Invalid steps_per_revolution from parent: %.1f", steps_per_rev);
        return 0;
      }

      // Convert encoder ticks to steps
      // steps = (total_ticks × steps_per_rev) / 16384
      int64_t total = this->ticks_total();
      int32_t steps_value = static_cast<int32_t>((total * static_cast<int64_t>(steps_per_rev)) / static_cast<int64_t>(TICKS_PER_REV));

      ESP_LOGV(TAG, "Position(revs=%d, ticks=%u) → %d steps (steps_per_rev=%.1f)", revs_, angle_ticks_, steps_value,
               steps_per_rev);

      return steps_value;
    }

    float Position::degrees() const
    {
      // Convert encoder ticks to degrees
      // degrees = (total_ticks / 16384) × 360
      int64_t total = this->ticks_total();
      float degrees_value = (static_cast<float>(total) / static_cast<float>(TICKS_PER_REV)) * 360.0f;
      return degrees_value;
    }

    float Position::radians() const
    {
      // Convert encoder ticks to radians
      // radians = (total_ticks / 16384) × 2π
      int64_t total = this->ticks_total();
      float radians_value = (static_cast<float>(total) / static_cast<float>(TICKS_PER_REV)) * TWO_PI;
      return radians_value;
    }

    float Position::arcminutes() const
    {
      // Convert encoder ticks to arcminutes
      // arcmin = (total_ticks / 16384) × 21600
      int64_t total = this->ticks_total();
      float arcmin_value = (static_cast<float>(total) / static_cast<float>(TICKS_PER_REV)) * 21600.0f;
      return arcmin_value;
    }

    float Position::arcseconds() const
    {
      // Convert encoder ticks to arcseconds
      // arcsec = (total_ticks / 16384) × 1296000
      int64_t total = this->ticks_total();
      float arcsec_value = (static_cast<float>(total) / static_cast<float>(TICKS_PER_REV)) * 1296000.0f;
      return arcsec_value;
    }

    uint32_t Position::steps_as_u32(const ServoXxdModbus *parent) const
    {
      int32_t steps_value = this->steps(parent);
      // Convert signed to unsigned (wraps around if negative)
      return static_cast<uint32_t>(steps_value);
    }

    Position Position::operator+(const Position &rhs) const
    {
      // Add total ticks and convert back to split format
      int64_t total_lhs = this->ticks_total();
      int64_t total_rhs = rhs.ticks_total();
      int64_t total_sum = total_lhs + total_rhs;

      return Position::from_ticks_total(total_sum);
    }

    Position Position::operator-(const Position &rhs) const
    {
      // Subtract total ticks and convert back to split format
      int64_t total_lhs = this->ticks_total();
      int64_t total_rhs = rhs.ticks_total();
      int64_t total_diff = total_lhs - total_rhs;

      return Position::from_ticks_total(total_diff);
    }

    bool Position::operator==(const Position &rhs) const
    {
      // Compare split format directly
      return (revs_ == rhs.revs_) && (angle_ticks_ == rhs.angle_ticks_);
    }

    void Position::normalize()
    {
      // Ensure angle_ticks is in [0, 16383] range
      while (angle_ticks_ >= TICKS_PER_REV)
      {
        revs_++;
        angle_ticks_ -= TICKS_PER_REV;
      }

      // Handle underflow (should not happen with uint16_t, but for completeness)
      // Note: angle_ticks_ is uint16_t, so it can't be negative
      // This is handled in the constructor when converting from int64_t
    }

  } // namespace servoxxd_modbus
} // namespace esphome
