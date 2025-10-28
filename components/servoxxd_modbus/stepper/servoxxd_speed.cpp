#include "servoxxd_speed.h"
#include "servoxxd_modbus.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome
{
  namespace servoxxd_modbus
  {

    static const char *const TAG = "servoxxd_modbus.speed";

    // Constructor: Convert value from any unit to RPM
    Speed::Speed(float value, SpeedUnit unit, const ServoXxdModbus *parent) : parent_(parent)
    {
      float rpm_float = 0.0f; // Temporary float for conversion

      switch (unit)
      {
      case SpeedUnit::STEPS_PER_SEC:
        // Need parent for steps_per_revolution
        if (parent_ == nullptr)
        {
          ESP_LOGE(TAG, "Speed: parent required for STEPS_PER_SEC conversion");
          rpm_ = 0;
          return;
        }
        else
        {
          float steps_per_rev = parent_->get_steps_per_revolution();
          if (steps_per_rev <= 0)
          {
            ESP_LOGE(TAG, "Speed: Invalid steps_per_revolution: %.2f", steps_per_rev);
            rpm_ = 0;
            return;
          }
          rpm_float = (value / steps_per_rev) * 60.0f;
        }
        break;

      case SpeedUnit::RPM:
        rpm_float = value;
        break;

      case SpeedUnit::REV_PER_SEC:
        rpm_float = value * 60.0f;
        break;

      case SpeedUnit::DEGREES_PER_SEC:
        rpm_float = (value / 360.0f) * 60.0f;
        break;

      case SpeedUnit::RADIANS_PER_SEC:
        rpm_float = (value / (2.0f * M_PI)) * 60.0f;
        break;

      case SpeedUnit::DEGREES_PER_MIN:
        rpm_float = value / 6.0f; // 360° / 60min = 6
        break;

      case SpeedUnit::DEGREES_PER_HOUR:
        rpm_float = value / 360.0f; // 360° = 1 rev, 60min = 1h → /360
        break;

      default:
        ESP_LOGE(TAG, "Unknown speed unit: %d", static_cast<int>(unit));
        rpm_ = 0;
        return;
      }

      // Clamp to hardware limits (-3000 to +3000 RPM) BEFORE casting to int16_t
      // to avoid undefined behavior on overflow
      if (rpm_float > 3000.0f)
      {
        ESP_LOGW(TAG, "Speed %.0f RPM exceeds max (3000 RPM), clamping", rpm_float);
        rpm_float = 3000.0f;
      }
      else if (rpm_float < -3000.0f)
      {
        ESP_LOGW(TAG, "Speed %.0f RPM below min (-3000 RPM), clamping", rpm_float);
        rpm_float = -3000.0f;
      }

      // Now safe to cast to int16_t
      rpm_ = static_cast<int16_t>(std::round(rpm_float));
    }

    // Get speed for hardware with microstepping compensation
    int16_t Speed::rpm_for_hardware() const
    {
      if (parent_ == nullptr)
      {
        ESP_LOGW(TAG, "rpm_for_hardware: parent is null, returning raw RPM");
        return rpm_;
      }

      uint16_t microsteps = parent_->get_microstepping();
      int16_t scaled_rpm = rpm_;

      // Apply hardware scaling compensation
      if (microsteps == 8)
      {
        scaled_rpm = rpm_ * 2; // Compensate for 8 microsteps
      }
      else if (microsteps == 128)
      {
        scaled_rpm = rpm_ / 8; // Compensate for 128 microsteps
      }
      else if (microsteps == 256)
      {
        scaled_rpm = rpm_ / 16; // Compensate for 256 microsteps
      }
      // For 16, 32, 64: no scaling needed (reference values)

      // Re-clamp after scaling
      if (scaled_rpm > 3000)
        scaled_rpm = 3000;
      if (scaled_rpm < -3000)
        scaled_rpm = -3000;

      return scaled_rpm;
    }

    // Get speed as steps per second
    float Speed::steps_per_sec() const
    {
      if (parent_ == nullptr)
      {
        ESP_LOGW(TAG, "steps_per_sec: parent is null, returning 0");
        return 0.0f;
      }

      float steps_per_rev = parent_->get_steps_per_revolution();
      return (rpm_ / 60.0f) * steps_per_rev;
    }

  } // namespace servoxxd_modbus
} // namespace esphome
