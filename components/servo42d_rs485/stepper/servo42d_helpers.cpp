#include "servo42d_helpers.h"
#include <cmath>

namespace esphome
{
  namespace servo42d_rs485
  {

    // ========================================================================
    // Speed/Acceleration Conversions
    // ========================================================================

    uint16_t Servo42dHelpers::steps_per_second_to_rpm(float steps_per_second, float steps_per_revolution)
    {
      // Convert steps/s to RPM based on steps_per_revolution
      float rpm = (steps_per_second / steps_per_revolution) * 60.0f;

      // Clamp to motor limits (typically 0-3000 RPM)
      if (rpm < 0)
        rpm = 0;
      if (rpm > 3000)
        rpm = 3000;

      return static_cast<uint16_t>(rpm);
    }

    float Servo42dHelpers::rpm_to_steps_per_second(uint16_t rpm, float steps_per_revolution)
    {
      return (rpm / 60.0f) * steps_per_revolution;
    }

    uint16_t Servo42dHelpers::acceleration_to_internal(float steps_per_second_sq, float steps_per_revolution)
    {
      // Motor acceleration is 0-255, where higher value = faster acceleration
      // This is a simplified conversion - may need tuning

      float revolutions_per_second_sq = steps_per_second_sq / steps_per_revolution;

      // Scale to 0-255 range (this is approximate)
      uint16_t accel = static_cast<uint16_t>(revolutions_per_second_sq * 10.0f);

      if (accel > 255)
        accel = 255;
      if (accel < 1)
        accel = 1;

      return accel;
    }

    // ========================================================================
    // Ticks Conversions (Encoder: 16384 ticks/revolution - hardware constant)
    // ========================================================================

    float Servo42dHelpers::ticks_to_degrees(int64_t ticks)
    {
      // 16384 ticks = 360 degrees
      float degrees = (ticks * 360.0f) / 16384.0f;
      return normalize_degrees(degrees);
    }

    float Servo42dHelpers::ticks_to_radians(int64_t ticks)
    {
      // 16384 ticks = 2π radians
      float radians = (ticks * 2.0f * M_PI) / 16384.0f;
      return normalize_radians(radians);
    }

    int64_t Servo42dHelpers::degrees_to_ticks(float degrees)
    {
      // 360 degrees = 16384 ticks
      return static_cast<int64_t>(std::round((degrees * 16384.0f) / 360.0f));
    }

    int64_t Servo42dHelpers::radians_to_ticks(float radians)
    {
      // 2π radians = 16384 ticks
      return static_cast<int64_t>(std::round((radians * 16384.0f) / (2.0f * M_PI)));
    }

    // ========================================================================
    // Steps Conversions (Motor: steps_per_revolution from config)
    // ========================================================================

    float Servo42dHelpers::steps_to_degrees(int32_t steps, float steps_per_revolution)
    {
      if (steps_per_revolution <= 0.0f)
        return 0.0f;
      
      float degrees = (steps * 360.0f) / steps_per_revolution;
      return normalize_degrees(degrees);
    }

    float Servo42dHelpers::steps_to_radians(int32_t steps, float steps_per_revolution)
    {
      if (steps_per_revolution <= 0.0f)
        return 0.0f;
      
      float radians = (steps * 2.0f * M_PI) / steps_per_revolution;
      return normalize_radians(radians);
    }

    int32_t Servo42dHelpers::degrees_to_steps(float degrees, float steps_per_revolution)
    {
      if (steps_per_revolution <= 0.0f)
        return 0;
      
      return static_cast<int32_t>(std::round((degrees * steps_per_revolution) / 360.0f));
    }

    int32_t Servo42dHelpers::radians_to_steps(float radians, float steps_per_revolution)
    {
      if (steps_per_revolution <= 0.0f)
        return 0;
      
      return static_cast<int32_t>(std::round((radians * steps_per_revolution) / (2.0f * M_PI)));
    }

    // ========================================================================
    // Angle Normalization Utilities
    // ========================================================================

    float Servo42dHelpers::normalize_degrees(float degrees)
    {
      // Normalize to 0-359.99...
      degrees = std::fmod(degrees, 360.0f);
      if (degrees < 0.0f)
        degrees += 360.0f;
      return degrees;
    }

    float Servo42dHelpers::normalize_radians(float radians)
    {
      // Normalize to 0-2π
      radians = std::fmod(radians, 2.0f * M_PI);
      if (radians < 0.0f)
        radians += 2.0f * M_PI;
      return radians;
    }

  } // namespace servo42d_rs485
} // namespace esphome
