#pragma once

#include "esphome/core/log.h"

namespace esphome
{
  namespace servoxxd_modbus
  {

    // Forward declaration
    class ServoXxdModbus;

    /**
     * @brief Unit options for acceleration values
     *
     * Defines the units that can be used to specify motor acceleration.
     * All units are converted to a hardware-native value (0-255) during construction.
     */
    enum class AccelerationUnit : uint8_t
    {
      STEPS_PER_SEC_SQ = 0,   ///< Steps per second squared (ESPHome default)
      RPM_PER_SEC = 1,        ///< Revolutions per minute per second (motor native)
      REV_PER_SEC_SQ = 2,     ///< Revolutions per second squared
      DEGREES_PER_SEC_SQ = 3, ///< Degrees per second squared
      RADIANS_PER_SEC_SQ = 4  ///< Radians per second squared
    };

    /**
     * @brief Acceleration value with unit conversion and hardware encoding
     *
     * This class handles acceleration values with automatic unit conversion to the
     * hardware-native format (inverse time mapping, 0-255).
     *
     * **Hardware Encoding (Non-linear Inverse Time Mapping):**
     *
     * The motor controller uses a special encoding where the acceleration value controls
     * the time interval between successive ±1 RPM speed changes:
     *
     * - `acc = 0`: Special case - no ramp, instant speed change (∞ RPM/s)
     * - `acc = 1-255`: Δt = (256 - acc) × 50 μs
     *   - `acc = 1`: Slowest (Δt = 12.75 ms → ~78 RPM/s)
     *   - `acc = 255`: Fastest (Δt = 50 μs → 20000 RPM/s)
     *
     * **Conversion Formula (User → Hardware):**
     *
     * Given desired acceleration `a_user` in RPM/s:
     * ```
     * If a_user = 0 or ∞:  acc = 0  (no ramp)
     * If a_user > 0:       acc = 256 - (20000 / a_user)
     *                      acc = clamp(acc, 1, 255)
     * ```
     *
     * **Conversion Formula (Hardware → Effective Rate):**
     * ```
     * If acc = 0:       a_eff = ∞  (instant)
     * If acc = 1-255:   a_eff = 20000 / (256 - acc)  [RPM/s]
     * ```
     *
     * **Important Notes:**
     * - acc=0 is special: user value taken directly, no ramping
     * - Non-linear relationship: small changes at high acc values = large effect differences
     * - Not microstepping-calibrated (unlike Speed class)
     * - Applies to already-scaled RPM values
     *
     * @see ServoXxdModbus for parent class that provides steps_per_revolution
     */
    class Acceleration
    {
      friend class ServoXxdModbus;

    public:
      /**
       * @brief Construct an Acceleration from a value and unit
       *
       * Converts the input value to RPM/s, then maps to hardware value (0-255)
       * using the inverse time formula.
       *
       * @param value Acceleration magnitude in the specified unit
       * @param unit The unit of the acceleration value
       * @param parent Pointer to parent ServoXxdModbus (for steps_per_revolution, required for STEPS_PER_SEC_SQ conversion)
       */
      Acceleration(float value, AccelerationUnit unit, const ServoXxdModbus *parent);

      /**
       * @brief Default constructor (acceleration = 0, no ramping)
       */
      Acceleration() = default;

      /**
       * @brief Get the hardware-native acceleration value
       * @return Hardware value (0-255) for direct transmission to motor
       */
      uint8_t acc_internal() const { return acc_; }

      /**
       * @brief Get approximate acceleration in RPM/s (for display/logging)
       *
       * Note: This is an approximation due to non-linear hardware encoding.
       * Returns ∞ for acc_=0 (represented as -1.0f).
       *
       * @return Effective acceleration in RPM/s, or -1.0f for instant (acc_=0)
       */
      float rpm_per_sec() const;

      /**
       * @brief Get acceleration in steps/s² for ESPHome base class
       *
       * Converts the hardware acceleration back to steps/s² for compatibility
       * with ESPHome's stepper base class.
       *
       * @param parent Pointer to parent ServoXxdModbus (for steps_per_revolution)
       * @return Acceleration in steps/s², or -1.0f for instant (acc_=0)
       */
      float steps_per_sec2(const ServoXxdModbus *parent) const;

    private:
      uint8_t acc_{0}; ///< Hardware value 0-255 (inverse time mapping)
    };

  } // namespace servoxxd_modbus
} // namespace esphome
