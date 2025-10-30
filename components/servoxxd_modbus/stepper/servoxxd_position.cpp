#include "servoxxd_position.h"
#include "servoxxd_modbus.h"
#include <cmath>

namespace esphome
{
  namespace servoxxd_modbus
  {

    static const char *const TAG = "servoxxd_modbus.position";

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
      case PositionUnit::TICKS:
        return "ticks";
      default:
        return "unknown";
      }
    }

    // ============================================================================
    // Constructors
    // ============================================================================

    Position::Position(double value, PositionUnit unit, const ServoXxdModbus *parent) : parent_(parent)
    {
      ESP_LOGV(TAG, "Creating Position: value=%.3f %s, parent=%p", value, unit_to_string(unit),
               static_cast<const void *>(parent_));

      // Use factory methods for conversion
      Position temp;

      switch (unit)
      {
      case PositionUnit::TICKS:
        temp = Position::from_ticks(static_cast<int64_t>(value));
        break;
      case PositionUnit::STEPS:
        temp = Position::from_steps(static_cast<int64_t>(value), parent);
        break;
      case PositionUnit::REVOLUTIONS:
        temp = Position::from_revolutions(value);
        break;
      case PositionUnit::DEGREES:
        temp = Position::from_degrees(value);
        break;
      case PositionUnit::RADIANS:
        temp = Position::from_radians(value);
        break;
      case PositionUnit::ARCMINUTES:
        temp = Position::from_arcminutes(static_cast<int64_t>(value));
        break;
      case PositionUnit::ARCSECONDS:
        temp = Position::from_arcseconds(static_cast<int64_t>(value));
        break;
      default:
        ESP_LOGE(TAG, "Unknown PositionUnit: %d", static_cast<int>(unit));
        revs_ = 0;
        angle_ticks_ = 0;
        return;
      }

      // Copy result
      revs_ = temp.revs_;
      angle_ticks_ = temp.angle_ticks_;

      ESP_LOGV(TAG, "Position created: value=%.3f %s → revs=%d, angle_ticks=%u",
               value, unit_to_string(unit), revs_, angle_ticks_);
    }

    Position::Position(float value, PositionUnit unit, const ServoXxdModbus *parent)
        : Position(static_cast<double>(value), unit, parent) {}

    Position::Position(int64_t value, PositionUnit unit, const ServoXxdModbus *parent)
        : Position(static_cast<double>(value), unit, parent) {}

    Position::Position(int32_t value, PositionUnit unit, const ServoXxdModbus *parent)
        : Position(static_cast<double>(value), unit, parent) {}

    // ============================================================================
    // Factory Methods - Unit Conversions
    // ============================================================================

    Position Position::from_ticks(int64_t total_ticks)
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

      ESP_LOGV(TAG, "from_ticks(%lld) → revs=%d, angle_ticks=%u", total_ticks, pos.revs_, pos.angle_ticks_);

      return pos;
    }

    Position Position::from_steps(int64_t steps, const ServoXxdModbus *parent)
    {
      if (parent == nullptr)
      {
        ESP_LOGE(TAG, "Parent pointer is null (required for STEPS unit conversion)");
        return Position(); // Return zero position
      }

      float steps_per_revolution = parent->get_steps_per_revolution();
      if (steps_per_revolution <= 0.0f)
      {
        ESP_LOGE(TAG, "Invalid steps_per_revolution: %.1f (must be > 0)", steps_per_revolution);
        return Position(); // Return zero position
      }

      // revolutions = steps / steps_per_rev
      double revolutions_total = static_cast<double>(steps) / static_cast<double>(steps_per_revolution);
      return from_revolutions(revolutions_total);
    }

    Position Position::from_revolutions(double revolutions)
    {
      Position pos;
      double revolutions_total = revolutions;

      // Split into integer and fractional parts
      pos.revs_ = static_cast<int32_t>(revolutions_total);
      double fractional_rev = revolutions_total - static_cast<double>(pos.revs_);

      // Handle negative fractional part
      if (fractional_rev < 0.0)
      {
        pos.revs_--;
        fractional_rev += 1.0;
      }

      pos.angle_ticks_ = static_cast<uint16_t>(fractional_rev * static_cast<double>(TICKS_PER_REV));
      return pos;
    }

    Position Position::from_degrees(double deg)
    {
      // 1 revolution = 360°
      double revolutions_total = deg / 360.0;
      return from_revolutions(revolutions_total);
    }

    Position Position::from_radians(double rad)
    {
      // 1 revolution = 2π radians
      double revolutions_total = rad / TWO_PI;
      return from_revolutions(revolutions_total);
    }

    Position Position::from_arcminutes(int64_t arcminutes)
    {
      // 1 revolution = 21600 arcmin
      // Convert using ticks for precision
      int64_t total_ticks = (arcminutes * static_cast<int64_t>(TICKS_PER_REV)) / 21600LL;
      return from_ticks(total_ticks);
    }

    Position Position::from_arcseconds(int64_t arcseconds)
    {
      // 1 revolution = 1296000 arcsec
      // Convert using ticks for precision
      int64_t total_ticks = (arcseconds * static_cast<int64_t>(TICKS_PER_REV)) / 1296000LL;
      return from_ticks(total_ticks);
    }

    // ============================================================================
    // Getters
    // ============================================================================

    double Position::get_double_unit(PositionUnit unit) const
    {
      // Compute total ticks directly to avoid recursion with get_ticks()
      int64_t total = static_cast<int64_t>(revs_) * static_cast<int64_t>(TICKS_PER_REV) +
                      static_cast<int64_t>(angle_ticks_);

      switch (unit)
      {
      case PositionUnit::TICKS:
        return static_cast<double>(total);

      case PositionUnit::STEPS:
      {
        if (!parent_)
        {
          ESP_LOGE(TAG, "get_steps: parent_ is nullptr - cannot get steps_per_rev");
          return 0.0;
        }
        float steps_per_rev = parent_->get_steps_per_revolution();
        // steps = (total_ticks × steps_per_rev) / 16384
        return (static_cast<double>(total) * steps_per_rev) / static_cast<double>(TICKS_PER_REV);
      }

      case PositionUnit::REVOLUTIONS:
        // revs = total_ticks / 16384
        return static_cast<double>(total) / static_cast<double>(TICKS_PER_REV);

      case PositionUnit::DEGREES:
        // degrees = (total_ticks × 360) / 16384
        return (static_cast<double>(total) * 360.0) / static_cast<double>(TICKS_PER_REV);

      case PositionUnit::RADIANS:
        // radians = (total_ticks × 2π) / 16384
        return (static_cast<double>(total) * 2.0 * M_PI) / static_cast<double>(TICKS_PER_REV);

      case PositionUnit::ARCMINUTES:
        return static_cast<double>(get_int64_unit(PositionUnit::ARCMINUTES));

      case PositionUnit::ARCSECONDS:
        return static_cast<double>(get_int64_unit(PositionUnit::ARCSECONDS));

      default:
        ESP_LOGE(TAG, "Invalid PositionUnit in get_double_unit()");
        return 0.0;
      }
    }

    int64_t Position::get_int64_unit(PositionUnit unit) const
    {
      // Compute total ticks directly to avoid recursion with get_ticks()
      int64_t total = static_cast<int64_t>(revs_) * static_cast<int64_t>(TICKS_PER_REV) +
                      static_cast<int64_t>(angle_ticks_);

      switch (unit)
      {
      case PositionUnit::TICKS:
        return total;

      case PositionUnit::STEPS:
      {
        if (!parent_)
        {
          ESP_LOGE(TAG, "get_steps: parent_ is nullptr - cannot get steps_per_rev");
          return 0;
        }
        float steps_per_rev = parent_->get_steps_per_revolution();
        // steps = (total_ticks × steps_per_rev) / 16384
        return (total * static_cast<int64_t>(steps_per_rev)) / static_cast<int64_t>(TICKS_PER_REV);
      }

      case PositionUnit::ARCMINUTES:
        // arcmin = (total_ticks × 21600) / 16384
        return (total * 21600LL) / static_cast<int64_t>(TICKS_PER_REV);

      case PositionUnit::ARCSECONDS:
        // arcsec = (total_ticks × 1296000) / 16384
        return (total * 1296000LL) / static_cast<int64_t>(TICKS_PER_REV);

      default:
        ESP_LOGE(TAG, "Invalid PositionUnit for int64_t conversion: %s", unit_to_string(unit));
        return 0;
      }
    }

    // ============================================================================
    // Setters
    // ============================================================================

    void Position::set(double value, PositionUnit unit)
    {
      // Create temporary Position with the new value using the internal parent pointer
      Position temp(value, unit, this->parent_);

      // Copy the calculated values back to this object
      this->revs_ = temp.revs_;
      this->angle_ticks_ = temp.angle_ticks_;
    }

    void Position::set(float value, PositionUnit unit)
    {
      set(static_cast<double>(value), unit);
    }

    void Position::set(int64_t value, PositionUnit unit)
    {
      set(static_cast<double>(value), unit);
    }

    void Position::set(int32_t value, PositionUnit unit)
    {
      set(static_cast<double>(value), unit);
    }

    // ============================================================================
    // Operators
    // ============================================================================

    Position Position::operator+(const Position &rhs) const
    {
      // Add total ticks and convert back to split format
      int64_t total_lhs = this->get_ticks();
      int64_t total_rhs = rhs.get_ticks();
      int64_t total_sum = total_lhs + total_rhs;

      return Position::from_ticks(total_sum);
    }

    Position Position::operator-(const Position &rhs) const
    {
      // Subtract total ticks and convert back to split format
      int64_t total_lhs = this->get_ticks();
      int64_t total_rhs = rhs.get_ticks();
      int64_t total_diff = total_lhs - total_rhs;

      return Position::from_ticks(total_diff);
    }

    bool Position::operator==(const Position &rhs) const
    {
      // Compare split format directly
      return (revs_ == rhs.revs_) && (angle_ticks_ == rhs.angle_ticks_);
    }

    // ============================================================================
    // Private Methods
    // ============================================================================

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
