#pragma once

#include "esphome/core/component.h"
#include <cstdint>
#include <cmath>

namespace esphome
{
  namespace servoxxd_modbus
  {

    // Forward declaration
    class ServoXxdModbus;

    /// Speed unit enumeration - maps to YAML SpeedUnit
    enum class SpeedUnit : uint8_t
    {
      STEPS_PER_SEC = 0,   ///< Steps per second (default - ESPHome stepper compatibility)
      RPM = 1,             ///< Revolutions per minute (motor native)
      REV_PER_SEC = 2,     ///< Revolutions per second
      DEGREES_PER_SEC = 3, ///< Degrees per second
      RADIANS_PER_SEC = 4, ///< Radians per second
      DEGREES_PER_MIN = 5, ///< Degrees per minute (moderate rotation)
      DEGREES_PER_HOUR = 6 ///< Degrees per hour (astronomical tracking)
    };

    /// Speed value with unit conversion support
    ///
    /// This class stores speed internally as RPM (int16_t) and provides
    /// conversion from all supported units. Conversion happens in the constructor.
    ///
    /// Usage:
    ///   Speed speed(1000.0f, SpeedUnit::STEPS_PER_SEC);
    ///   int16_t rpm = speed.rpm_as_i16();
    ///   int16_t hw_rpm = speed.rpm_for_hardware(parent);  // With microstepping scaling
    class Speed
    {
      friend class ServoXxdModbus;

    public:
      /// Construct Speed from value and unit
      /// @param value Speed value in the specified unit
      /// @param unit Speed unit (default: STEPS_PER_SEC)
      /// @param parent Pointer to parent ServoXxdModbus (for steps_per_revolution, required for STEPS_PER_SEC conversion)
      Speed(float value, SpeedUnit unit, const ServoXxdModbus *parent);

      /// Default constructor - initializes to 0 RPM
      Speed() = default;

      /// Get speed as RPM (float)
      float rpm() const { return rpm_; }

      /// Get speed as RPM (int16_t) - rounded and clamped
      int16_t rpm_as_i16() const { return rpm_; }

      /// Get speed for hardware with microstepping compensation
      ///
      /// The motor controller's speed values are calibrated for 16/32/64 subdivisions.
      /// For other microstepping settings, the hardware applies automatic scaling:
      /// asked_speed = actual_speed × (16 / current_microsteps)
      ///
      /// This method applies inverse scaling to compensate:
      /// - Microsteps 8: multiply by 2
      /// - Microsteps 16/32/64: no change (reference)
      /// - Microsteps 128: divide by 8
      /// - Microsteps 256: divide by 16
      ///
      /// @param parent Component instance with microstepping configuration
      /// @return Speed value compensated for hardware scaling
      int16_t rpm_for_hardware(const ServoXxdModbus *parent) const;

      /// Get speed as steps per second (for ESPHome stepper base class)
      /// @param parent Component instance with steps_per_revolution
      /// @return Speed in steps per second
      float steps_per_sec(const ServoXxdModbus *parent) const;

    private:
      int16_t rpm_{0}; ///< Internal storage: RPM (signed, -32768 to +32767)
    };

  } // namespace servoxxd_modbus
} // namespace esphome
