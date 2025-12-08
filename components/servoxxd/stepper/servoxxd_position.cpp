#include "servoxxd_position.h"
#include "servoxxd.h"
#include <cmath>

namespace esphome
{
  namespace servoxxd
  {

    static const char *const TAG = "servoxxd.position";

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

    Position::Position(double value, PositionUnit unit, const ServoXxd *parent) : parent_(parent)
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

      // Copy result (parent_ is already set in constructor initializer list)
      revs_ = temp.revs_;
      angle_ticks_ = temp.angle_ticks_;
      // Note: parent_ pointer is preserved from constructor parameter, not from temp

      ESP_LOGV(TAG, "Position created: value=%.3f %s → revs=%d, angle_ticks=%u, parent=%p",
               value, unit_to_string(unit), revs_, angle_ticks_, static_cast<const void *>(parent_));
    }

    Position::Position(float value, PositionUnit unit, const ServoXxd *parent)
        : Position(static_cast<double>(value), unit, parent) {}

    // Copy constructor
    Position::Position(const Position &other)
        : revs_(other.revs_), angle_ticks_(other.angle_ticks_), parent_(other.parent_)
    {
      ESP_LOGD(TAG, "Copy constructor: revs=%d, angle_ticks=%u, parent=%p (from %p)",
               revs_, angle_ticks_, static_cast<const void *>(parent_), static_cast<const void *>(other.parent_));
    }

    Position::Position(int64_t value, PositionUnit unit, const ServoXxd *parent)
        : Position(static_cast<double>(value), unit, parent) {}

    Position::Position(int32_t value, PositionUnit unit, const ServoXxd *parent)
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

    Position Position::from_ticks(int64_t total_ticks, const ServoXxd *parent)
    {
      Position pos = from_ticks(total_ticks);
      pos.parent_ = parent;
      return pos;
    }

    Position Position::from_steps(int64_t steps, const ServoXxd *parent)
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
      Position pos = from_revolutions(revolutions_total);
      pos.parent_ = parent; // Propagate parent pointer
      return pos;
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
      switch (unit)
      {
      case PositionUnit::TICKS:
        // total_ticks = revs * 16384 + angle_ticks
        return static_cast<double>(revs_) * static_cast<double>(TICKS_PER_REV) +
               static_cast<double>(angle_ticks_);

      case PositionUnit::STEPS:
      {
        if (!parent_)
        {
          ESP_LOGE(TAG, "get_steps: parent_ is nullptr - cannot get steps_per_rev");
          return 0.0;
        }
        float steps_per_rev = parent_->get_steps_per_revolution();
        // steps = revs * steps_per_rev + (angle_ticks * steps_per_rev / 16384)
        return static_cast<double>(revs_) * steps_per_rev +
               (static_cast<double>(angle_ticks_) * steps_per_rev / static_cast<double>(TICKS_PER_REV));
      }

      case PositionUnit::REVOLUTIONS:
        // revs = revs + angle_ticks / 16384
        return static_cast<double>(revs_) +
               (static_cast<double>(angle_ticks_) / static_cast<double>(TICKS_PER_REV));

      case PositionUnit::DEGREES:
        // degrees = revs * 360 + (angle_ticks * 360 / 16384)
        return static_cast<double>(revs_) * 360.0 +
               (static_cast<double>(angle_ticks_) * 360.0 / static_cast<double>(TICKS_PER_REV));

      case PositionUnit::RADIANS:
        // radians = revs * 2π + (angle_ticks * 2π / 16384)
        return static_cast<double>(revs_) * TWO_PI +
               (static_cast<double>(angle_ticks_) * TWO_PI / static_cast<double>(TICKS_PER_REV));

      case PositionUnit::ARCMINUTES:
        // arcmin = revs * 21600 + (angle_ticks * 21600 / 16384)
        return static_cast<double>(revs_) * 21600.0 +
               (static_cast<double>(angle_ticks_) * 21600.0 / static_cast<double>(TICKS_PER_REV));

      case PositionUnit::ARCSECONDS:
        // arcsec = revs * 1296000 + (angle_ticks * 1296000 / 16384)
        return static_cast<double>(revs_) * 1296000.0 +
               (static_cast<double>(angle_ticks_) * 1296000.0 / static_cast<double>(TICKS_PER_REV));

      default:
        ESP_LOGE(TAG, "Invalid PositionUnit in get_double_unit()");
        return 0.0;
      }
    }

    int64_t Position::get_int64_unit(PositionUnit unit) const
    {
      switch (unit)
      {
      case PositionUnit::TICKS:
        // total_ticks = revs * 16384 + angle_ticks
        return static_cast<int64_t>(revs_) * static_cast<int64_t>(TICKS_PER_REV) +
               static_cast<int64_t>(angle_ticks_);

      case PositionUnit::STEPS:
      {
        if (!parent_)
        {
          ESP_LOGE(TAG, "get_steps: parent_ is nullptr (this=%p, revs=%d, angle_ticks=%u)",
                   static_cast<const void *>(this), revs_, angle_ticks_);
          return 0;
        }
        float steps_per_rev = parent_->get_steps_per_revolution();
        int64_t result = static_cast<int64_t>(revs_) * static_cast<int64_t>(steps_per_rev) +
                         (static_cast<int64_t>(angle_ticks_) * static_cast<int64_t>(steps_per_rev)) / static_cast<int64_t>(TICKS_PER_REV);
        ESP_LOGD(TAG, "get_steps: this=%p, parent=%p, revs=%d, angle_ticks=%u, steps_per_rev=%.0f → result=%lld",
                 static_cast<const void *>(this), static_cast<const void *>(parent_),
                 revs_, angle_ticks_, steps_per_rev, static_cast<long long>(result));
        return result;
      }

      case PositionUnit::ARCMINUTES:
        // arcmin = revs * 21600 + (angle_ticks * 21600 / 16384)
        return static_cast<int64_t>(revs_) * 21600LL +
               (static_cast<int64_t>(angle_ticks_) * 21600LL) / static_cast<int64_t>(TICKS_PER_REV);

      case PositionUnit::ARCSECONDS:
        // arcsec = revs * 1296000 + (angle_ticks * 1296000 / 16384)
        return static_cast<int64_t>(revs_) * 1296000LL +
               (static_cast<int64_t>(angle_ticks_) * 1296000LL) / static_cast<int64_t>(TICKS_PER_REV);

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
      // Check parent compatibility: both must have same parent or one must be nullptr
      if (this->parent_ != nullptr && rhs.parent_ != nullptr && this->parent_ != rhs.parent_)
      {
        ESP_LOGW(TAG, "Adding positions with different parents (lhs=%p, rhs=%p) - result may be invalid for get_steps()",
                 static_cast<const void *>(this->parent_), static_cast<const void *>(rhs.parent_));
      }

      // Add directly in split format (revs + angle_ticks)
      Position result;
      result.revs_ = this->revs_ + rhs.revs_;
      result.angle_ticks_ = this->angle_ticks_ + rhs.angle_ticks_;

      // Handle carry: if angle_ticks >= 16384, add to revolutions
      if (result.angle_ticks_ >= TICKS_PER_REV)
      {
        result.revs_++;
        result.angle_ticks_ -= TICKS_PER_REV;
      }

      // Propagate parent: prefer non-null parent, prefer lhs if both non-null
      result.parent_ = this->parent_ ? this->parent_ : rhs.parent_;
      return result;
    }

    Position Position::operator-(const Position &rhs) const
    {
      // Check parent compatibility: both must have same parent or one must be nullptr
      if (this->parent_ != nullptr && rhs.parent_ != nullptr && this->parent_ != rhs.parent_)
      {
        ESP_LOGW(TAG, "Subtracting positions with different parents (lhs=%p, rhs=%p) - result may be invalid for get_steps()",
                 static_cast<const void *>(this->parent_), static_cast<const void *>(rhs.parent_));
      }

      // Subtract directly in split format (revs + angle_ticks)
      Position result;
      result.revs_ = this->revs_ - rhs.revs_;

      // Handle borrow: if angle_ticks would be negative
      if (this->angle_ticks_ >= rhs.angle_ticks_)
      {
        result.angle_ticks_ = this->angle_ticks_ - rhs.angle_ticks_;
      }
      else
      {
        result.revs_--;
        result.angle_ticks_ = this->angle_ticks_ + TICKS_PER_REV - rhs.angle_ticks_;
      }

      // Propagate parent: prefer non-null parent, prefer lhs if both non-null
      result.parent_ = this->parent_ ? this->parent_ : rhs.parent_;
      return result;
    }

    Position Position::operator*(double scalar) const
    {
      // Multiply by converting to revolutions (double), scale, convert back
      double total_revs = static_cast<double>(this->revs_) +
                          (static_cast<double>(this->angle_ticks_) / static_cast<double>(TICKS_PER_REV));
      double scaled_revs = total_revs * scalar;

      Position result = from_revolutions(scaled_revs);
      result.parent_ = this->parent_; // Propagate parent pointer
      return result;
    }

    Position Position::operator/(double scalar) const
    {
      if (scalar == 0.0)
      {
        ESP_LOGE(TAG, "Division by zero in Position::operator/");
        Position result(this->parent_);
        return result;
      }

      // Divide by converting to revolutions (double), scale, convert back
      double total_revs = static_cast<double>(this->revs_) +
                          (static_cast<double>(this->angle_ticks_) / static_cast<double>(TICKS_PER_REV));
      double scaled_revs = total_revs / scalar;

      Position result = from_revolutions(scaled_revs);
      result.parent_ = this->parent_; // Propagate parent pointer
      return result;
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

  } // namespace servoxxd
} // namespace esphome
