#include "servoxxd_acceleration.h"
#include "servoxxd_modbus.h"
#include <cmath>

namespace esphome
{
  namespace servoxxd_modbus
  {

    static const char *const TAG = "servoxxd_modbus.acceleration";

    // Mathematical constants (C++11 compatible)
    constexpr float PI = 3.14159265358979323846f;
    constexpr float TWO_PI = 2.0f * PI;

    // Hardware acceleration constants
    static constexpr float MAX_RPM_PER_SEC = 20000.0f; // acc=255 → Δt=50μs → 20000 RPM/s

    /**
     * @brief Convert acceleration unit to string for logging
     */
    static const char *unit_to_string(AccelerationUnit unit)
    {
      switch (unit)
      {
      case AccelerationUnit::STEPS_PER_SEC_SQ:
        return "steps/s²";
      case AccelerationUnit::RPM_PER_SEC:
        return "RPM/s";
      case AccelerationUnit::REV_PER_SEC_SQ:
        return "rev/s²";
      case AccelerationUnit::DEGREES_PER_SEC_SQ:
        return "deg/s²";
      case AccelerationUnit::RADIANS_PER_SEC_SQ:
        return "rad/s²";
      default:
        return "unknown";
      }
    }

    Acceleration::Acceleration(float value, AccelerationUnit unit, const ServoXxdModbus *parent)
    {
      // Validate parent pointer for STEPS_PER_SEC_SQ unit
      if (unit == AccelerationUnit::STEPS_PER_SEC_SQ)
      {
        if (parent == nullptr)
        {
          ESP_LOGE(TAG, "Parent pointer is null (required for STEPS_PER_SEC_SQ unit conversion)");
          return; // Leave acceleration at zero
        }

        float steps_per_revolution = parent->get_steps_per_revolution();
        if (steps_per_revolution <= 0.0f)
        {
          ESP_LOGE(TAG, "Invalid steps_per_revolution: %.1f (must be > 0)", steps_per_revolution);
          return; // Leave acceleration at zero
        }
      }

      // Step 1: Convert input value to RPM/s
      float rpm_per_s = 0.0f;

      ESP_LOGV(TAG, "Creating Acceleration: value=%.2f %s, parent=%p", value, unit_to_string(unit),
               static_cast<const void *>(parent));

      switch (unit)
      {
      case AccelerationUnit::STEPS_PER_SEC_SQ:
      {
        // Convert steps/s² to RPM/s: rpm_per_s = (steps/s² * 60) / steps_per_revolution
        float steps_per_revolution = parent->get_steps_per_revolution();
        rpm_per_s = (value * 60.0f) / steps_per_revolution;
        break;
      }

      case AccelerationUnit::RPM_PER_SEC:
        // Direct conversion (motor native unit)
        rpm_per_s = value;
        break;

      case AccelerationUnit::REV_PER_SEC_SQ:
        // Convert rev/s² to RPM/s: rpm_per_s = rev/s² * 60
        rpm_per_s = value * 60.0f;
        break;

      case AccelerationUnit::DEGREES_PER_SEC_SQ:
        // Convert deg/s² to RPM/s: rpm_per_s = (deg/s² * 60) / 360
        rpm_per_s = (value * 60.0f) / 360.0f;
        break;

      case AccelerationUnit::RADIANS_PER_SEC_SQ:
        // Convert rad/s² to RPM/s: rpm_per_s = (rad/s² * 60) / (2π)
        rpm_per_s = (value * 60.0f) / TWO_PI;
        break;

      default:
        ESP_LOGE(TAG, "Unknown AccelerationUnit: %d", static_cast<int>(unit));
        rpm_per_s = 0.0f;
        break;
      }

      ESP_LOGV(TAG, "Converted to RPM/s: %.2f", rpm_per_s);

      // Step 2: Map RPM/s to hardware value (0-255) using inverse time formula
      //
      // Hardware encoding:
      // - acc = 0:     Special case - instant speed change (∞ RPM/s)
      // - acc = 1-255: Δt = (256 - acc) × 50 μs
      //                a_eff = 20000 / (256 - acc)  [RPM/s]
      //
      // Inverse formula (user → hardware):
      // - If rpm_per_s ≤ 0 or ≥ MAX_RPM_PER_SEC:  acc = 0 (instant)
      // - If rpm_per_s > 0:                       acc = 256 - (20000 / rpm_per_s)
      //                                           acc = clamp(acc, 1, 255)

      if (rpm_per_s <= 0.0f)
      {
        // Zero or negative acceleration → instant (no ramping)
        acc_ = 0;
        ESP_LOGV(TAG, "Acceleration ≤ 0 → acc=0 (instant, no ramping)");
      }
      else if (rpm_per_s >= MAX_RPM_PER_SEC)
      {
        // Very high acceleration → instant (hardware limit)
        acc_ = 0;
        ESP_LOGV(TAG, "Acceleration ≥ %.0f RPM/s → acc=0 (instant, hardware limit)", MAX_RPM_PER_SEC);
      }
      else
      {
        // Normal range: calculate hardware value
        // acc = 256 - (20000 / rpm_per_s)
        float acc_float = 256.0f - (MAX_RPM_PER_SEC / rpm_per_s);

        // Clamp to valid range [1, 255]
        // Note: acc=0 is reserved for instant, so minimum is 1
        if (acc_float < 1.0f)
        {
          acc_ = 1;
          ESP_LOGW(TAG, "Calculated acc=%.2f < 1, clamping to 1 (slowest, ~%.0f RPM/s)", acc_float,
                   MAX_RPM_PER_SEC / (256.0f - 1.0f));
        }
        else if (acc_float > 255.0f)
        {
          acc_ = 255;
          ESP_LOGW(TAG, "Calculated acc=%.2f > 255, clamping to 255 (fastest, %.0f RPM/s)", acc_float, MAX_RPM_PER_SEC);
        }
        else
        {
          acc_ = static_cast<uint8_t>(roundf(acc_float));
          ESP_LOGV(TAG, "Calculated acc=%d (%.2f RPM/s effective)", acc_, MAX_RPM_PER_SEC / (256.0f - acc_));
        }
      }

      ESP_LOGV(TAG, "Final hardware acceleration: acc_=%d", acc_);
    }

    float Acceleration::rpm_per_sec() const
    {
      if (acc_ == 0)
      {
        // Special case: instant speed change (∞ RPM/s)
        // Return -1.0f as sentinel value for "instant"
        return -1.0f;
      }

      // Calculate effective acceleration from hardware value
      // a_eff = 20000 / (256 - acc)
      float rpm_per_s = MAX_RPM_PER_SEC / static_cast<float>(256 - acc_);

      return rpm_per_s;
    }

    float Acceleration::steps_per_sec2(const ServoXxdModbus *parent) const
    {
      if (parent == nullptr)
      {
        ESP_LOGE(TAG, "steps_per_sec2() called with null parent pointer");
        return -1.0f;
      }

      if (acc_ == 0)
      {
        // Special case: instant speed change
        return -1.0f;
      }

      // Get effective acceleration in RPM/s
      float rpm_per_s = this->rpm_per_sec();

      // Get steps per revolution from parent
      float steps_per_rev = parent->get_steps_per_revolution();
      if (steps_per_rev <= 0)
      {
        ESP_LOGE(TAG, "Invalid steps_per_revolution from parent: %.1f", steps_per_rev);
        return -1.0f;
      }

      // Convert RPM/s to steps/s²
      // steps/s² = (rpm_per_s / 60) * steps_per_rev
      float steps_per_s2 = (rpm_per_s / 60.0f) * steps_per_rev;

      ESP_LOGV(TAG, "Converted acc=%d (%.2f RPM/s) to %.2f steps/s²", acc_, rpm_per_s, steps_per_s2);

      return steps_per_s2;
    }

  } // namespace servoxxd_modbus
} // namespace esphome
