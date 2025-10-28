#include "servoxxd_acceleration.h"
#include "servoxxd_modbus.h"
#include <cmath>
#include <algorithm>

namespace esphome
{
  namespace servoxxd_modbus
  {

    static const char *const TAG = "servoxxd_modbus.acceleration";

    // Hardware acceleration constants
    static constexpr float MAX_RPM_PER_SEC = 20000.0f; // acc=255 → Δt=50μs → 20000 RPM/s

    // Constructor: Convert value from any unit to hardware encoding (0-255)
    Acceleration::Acceleration(float value, AccelerationUnit unit, const ServoXxdModbus *parent) : parent_(parent)
    {
      // Step 1: Convert to RPM/s
      float rpm_per_sec = 0.0f;

      switch (unit)
      {
      case AccelerationUnit::STEPS_PER_SEC_SQ:
        // Need parent for steps_per_revolution
        if (parent_ == nullptr)
        {
          ESP_LOGE(TAG, "Acceleration: parent required for STEPS_PER_SEC_SQ conversion");
          acc_ = 0;
          return;
        }
        else
        {
          float steps_per_rev = parent_->get_steps_per_revolution();
          if (steps_per_rev <= 0)
          {
            ESP_LOGE(TAG, "Acceleration: Invalid steps_per_revolution: %.2f", steps_per_rev);
            acc_ = 0;
            return;
          }
          rpm_per_sec = (value / steps_per_rev) * 60.0f;
        }
        break;

      case AccelerationUnit::RPM_PER_SEC:
        rpm_per_sec = value;
        break;

      case AccelerationUnit::REV_PER_SEC_SQ:
        rpm_per_sec = value * 60.0f;
        break;

      case AccelerationUnit::DEGREES_PER_SEC_SQ:
        rpm_per_sec = (value / 360.0f) * 60.0f;
        break;

      case AccelerationUnit::RADIANS_PER_SEC_SQ:
        rpm_per_sec = (value / (2.0f * M_PI)) * 60.0f;
        break;

      default:
        ESP_LOGE(TAG, "Unknown acceleration unit: %d", static_cast<int>(unit));
        acc_ = 0;
        return;
      }

      // Step 2: Map RPM/s to hardware value (0-255, non-linear inverse time)
      if (rpm_per_sec <= 0.0f || std::isinf(rpm_per_sec))
      {
        // Zero, negative, or infinite acceleration → instant (no ramp)
        // Negative acceleration is physically invalid
        acc_ = 0;
      }
      else if (rpm_per_sec >= MAX_RPM_PER_SEC)
      {
        // Very high acceleration (>= 20000 RPM/s) → instant (no ramp)
        // This avoids acc values >= 255 which would clamp incorrectly
        acc_ = 0;
      }
      else
      {
        // Formula: acc = 256 - (20000 / rpm_per_sec)
        // We use 20000 instead of 20 because hardware Δt = (256 - acc) × 50 μs
        float acc_float = 256.0f - (MAX_RPM_PER_SEC / rpm_per_sec);
        // Clamp manually (C++11 compatible)
        if (acc_float < 1.0f)
          acc_ = 1;
        else if (acc_float > 255.0f)
          acc_ = 255;
        else
          acc_ = static_cast<uint8_t>(std::round(acc_float)); // Round instead of truncate
      }
    }

    // Convert hardware encoding back to effective RPM/s
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
      return MAX_RPM_PER_SEC / static_cast<float>(256 - acc_);
    }

    // Get acceleration as steps per second squared
    float Acceleration::steps_per_sec2() const
    {
      if (parent_ == nullptr)
      {
        ESP_LOGE(TAG, "steps_per_sec2: parent is null, returning -1.0f");
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
      float steps_per_rev = parent_->get_steps_per_revolution();
      if (steps_per_rev <= 0)
      {
        ESP_LOGE(TAG, "Invalid steps_per_revolution from parent: %.1f", steps_per_rev);
        return -1.0f;
      }

      // Convert RPM/s to steps/s²
      // steps/s² = (rpm_per_s / 60) * steps_per_rev
      return (rpm_per_s / 60.0f) * steps_per_rev;
    }

  } // namespace servoxxd_modbus
} // namespace esphome
