#include "servoxxd_speed.h"
#include "servoxxd.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome
{
  namespace servoxxd_modbus
  {

    static const char *const TAG = "servoxxd_modbus.speed";

    // Mathematical constants (C++11 compatible)
    constexpr float PI = 3.14159265358979323846f;
    constexpr float TWO_PI = 2.0f * PI;

    Speed::Speed(float value, SpeedUnit unit, const ServoXxdModbus *parent)
    {
      // Validate parent pointer for STEPS_PER_SEC unit
      if (unit == SpeedUnit::STEPS_PER_SEC)
      {
        if (parent == nullptr)
        {
          ESP_LOGE(TAG, "Parent pointer is null (required for STEPS_PER_SEC unit conversion)");
          return; // Leave speed at zero
        }
        
        float steps_per_revolution = parent->get_steps_per_revolution();
        if (steps_per_revolution <= 0.0f)
        {
          ESP_LOGE(TAG, "Invalid steps_per_revolution: %.1f (must be > 0)", steps_per_revolution);
          return; // Leave speed at zero
        }
      }

      float rpm_float = 0.0f;

      switch (unit)
      {
      case SpeedUnit::STEPS_PER_SEC:
      {
        // Convert steps/s to RPM
        // rpm = (steps/s * 60) / steps_per_rev
        float steps_per_revolution = parent->get_steps_per_revolution();
        rpm_float = (value * 60.0f) / steps_per_revolution;
        break;
      }

      case SpeedUnit::RPM:
        // Direct assignment - motor native unit
        rpm_float = value;
        break;

      case SpeedUnit::REV_PER_SEC:
        // Convert rev/s to RPM
        // rpm = rev/s * 60
        rpm_float = value * 60.0f;
        break;

      case SpeedUnit::DEGREES_PER_SEC:
        // Convert deg/s to RPM
        // rpm = (deg/s * 60) / 360
        rpm_float = (value * 60.0f) / 360.0f;
        break;

      case SpeedUnit::RADIANS_PER_SEC:
        // Convert radians/s to RPM: rpm = (rad/s * 60) / (2π)
        rpm_float = (value * 60.0f) / TWO_PI;
        break;

      case SpeedUnit::DEGREES_PER_MIN:
        // Convert deg/min to RPM
        // rpm = deg/min / 6
        rpm_float = value / 6.0f;
        break;

      case SpeedUnit::DEGREES_PER_HOUR:
        // Convert deg/h to RPM
        // rpm = deg/h / 360
        rpm_float = value / 360.0f;
        break;

      default:
        ESP_LOGE(TAG, "Unknown SpeedUnit: %d", static_cast<int>(unit));
        rpm_float = 0.0f;
        break;
      }

      // Round and clamp to int16_t range (-32768 to +32767)
      rpm_float = std::round(rpm_float);

      if (rpm_float > 32767.0f)
      {
        ESP_LOGW(TAG, "Speed %.1f RPM exceeds max (32767 RPM), clamping", rpm_float);
        rpm_ = 32767;
      }
      else if (rpm_float < -32768.0f)
      {
        ESP_LOGW(TAG, "Speed %.1f RPM below min (-32768 RPM), clamping", rpm_float);
        rpm_ = -32768;
      }
      else
      {
        rpm_ = static_cast<int16_t>(rpm_float);
      }

      ESP_LOGV(TAG, "Speed created: %.2f %s -> %d RPM",
               value,
               unit == SpeedUnit::STEPS_PER_SEC ? "steps/s" : unit == SpeedUnit::RPM            ? "RPM"
                                                          : unit == SpeedUnit::REV_PER_SEC      ? "rev/s"
                                                          : unit == SpeedUnit::DEGREES_PER_SEC  ? "deg/s"
                                                          : unit == SpeedUnit::RADIANS_PER_SEC  ? "rad/s"
                                                          : unit == SpeedUnit::DEGREES_PER_MIN  ? "deg/min"
                                                          : unit == SpeedUnit::DEGREES_PER_HOUR ? "deg/h"
                                                                                                : "unknown",
               rpm_);
    }

    int16_t Speed::rpm_for_hardware(const ServoXxdModbus *parent) const
    {
      if (parent == nullptr)
      {
        ESP_LOGE(TAG, "Parent is null, cannot apply microstepping compensation");
        return rpm_;
      }

      uint16_t microsteps = parent->get_microstepping();
      float compensated_rpm = static_cast<float>(rpm_);

      // Hardware calibration reference: 16, 32, or 64 subdivisions
      // For other values, apply inverse scaling to compensate
      // Hardware formula: asked_speed = actual_speed × (16 / current_microsteps)
      // Our compensation: send_speed = desired_speed × (current_microsteps / 16)

      if (microsteps == 16 || microsteps == 32 || microsteps == 64)
      {
        // Reference values - no compensation needed
        return rpm_;
      }
      else if (microsteps == 8)
      {
        // Hardware would multiply by 2, so we divide by 2
        compensated_rpm = compensated_rpm / 2.0f;
      }
      else if (microsteps == 128)
      {
        // Hardware would divide by 8, so we multiply by 8
        compensated_rpm = compensated_rpm * 8.0f;
      }
      else if (microsteps == 256)
      {
        // Hardware would divide by 16, so we multiply by 16
        compensated_rpm = compensated_rpm * 16.0f;
      }
      else if (microsteps < 16)
      {
        // General case for microsteps < 16: hardware multiplies by (16 / microsteps)
        float factor = 16.0f / static_cast<float>(microsteps);
        compensated_rpm = compensated_rpm / factor;
      }
      else
      {
        // General case for microsteps > 64: hardware divides by (microsteps / 16)
        float factor = static_cast<float>(microsteps) / 16.0f;
        compensated_rpm = compensated_rpm * factor;
      }

      // Round and clamp to int16_t range
      compensated_rpm = std::round(compensated_rpm);

      if (compensated_rpm > 32767.0f)
      {
        ESP_LOGW(TAG, "Compensated speed %.1f RPM exceeds max (32767 RPM), clamping", compensated_rpm);
        return 32767;
      }
      else if (compensated_rpm < -32768.0f)
      {
        ESP_LOGW(TAG, "Compensated speed %.1f RPM below min (-32768 RPM), clamping", compensated_rpm);
        return -32768;
      }

      int16_t result = static_cast<int16_t>(compensated_rpm);

      if (microsteps != 16 && microsteps != 32 && microsteps != 64)
      {
        ESP_LOGV(TAG, "Microstepping compensation: %d RPM -> %d RPM (microsteps=%d)",
                 rpm_, result, microsteps);
      }

      return result;
    }

    float Speed::steps_per_sec(const ServoXxdModbus *parent) const
    {
      if (parent == nullptr)
      {
        ESP_LOGE(TAG, "Parent is null, cannot convert to steps/s");
        return 0.0f;
      }

      float steps_per_rev = parent->get_steps_per_revolution();
      if (steps_per_rev <= 0.0f)
      {
        ESP_LOGE(TAG, "Invalid steps_per_revolution: %.2f", steps_per_rev);
        return 0.0f;
      }

      // Convert RPM to steps/s
      // steps/s = (rpm / 60) * steps_per_rev
      float steps_per_s = (static_cast<float>(rpm_) / 60.0f) * steps_per_rev;

      return steps_per_s;
    }

  } // namespace servoxxd_modbus
} // namespace esphome
