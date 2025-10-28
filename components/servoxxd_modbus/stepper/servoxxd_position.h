#pragma once

#include "esphome/core/log.h"
#include <cstdint>

namespace esphome
{
  namespace servoxxd_modbus
  {

    // Forward declaration
    class ServoXxdModbus;

    /**
     * @brief Unit options for position values
     *
     * Defines the units that can be used to specify motor position.
     * All units are converted to encoder ticks (split format) during construction.
     */
    enum class PositionUnit : uint8_t
    {
      STEPS = 0,       ///< Steps (ESPHome default)
      REVOLUTIONS = 1, ///< Full revolutions
      DEGREES = 2,     ///< Degrees (0-360 per revolution)
      RADIANS = 3,     ///< Radians (0-2π per revolution)
      ARCMINUTES = 4,  ///< Arcminutes (1° = 60 arcmin)
      ARCSECONDS = 5   ///< Arcseconds (1° = 3600 arcsec)
    };

    /**
     * @brief Position value with unit conversion and encoder format support
     *
     * This class handles position values with automatic unit conversion to the
     * hardware-native encoder format (split format: revolutions + angle_ticks).
     *
     * **Hardware Encoding (Split Format):**
     *
     * The motor controller uses a split format based on the integrated encoder:
     *
     * ```
     * Position = revolutions + (angle_ticks / 16384)
     * ```
     *
     * - `revolutions`: Full rotations (int32_t, -2³¹..2³¹-1)
     * - `angle_ticks`: Angle within one rotation (uint16_t, 0-16383)
     * - `TICKS_PER_REV = 16384`: Encoder constant (hardware-fixed, 2¹⁴ ticks per revolution)
     *
     * **Carry/Borrow Behavior:**
     * ```
     * If angle_ticks ≥ 16384:  revolutions++, angle_ticks -= 16384  (overflow → next revolution)
     * If angle_ticks < 0:      revolutions--, angle_ticks += 16384  (underflow → previous revolution)
     * ```
     *
     * Valid range: `angle_ticks` is always [0, 16383], over/underflow is carried to `revolutions`
     *
     * **Conversion (User → Hardware):**
     * ```
     * total_ticks = (value × steps_per_rev × 16384) / steps_per_rev
     * revs = total_ticks / 16384           (division with sign)
     * angle_ticks = total_ticks % 16384    (modulo always positive 0-16383)
     * ```
     *
     * **Conversion (Hardware → Total Ticks):**
     * ```
     * total_ticks = (revs × 16384) + angle_ticks
     * ```
     *
     * @see ServoXxdModbus for parent class that provides steps_per_revolution
     */
    class Position
    {
      friend class ServoXxdModbus;

    public:
      /**
       * @brief Construct a Position from a value and unit
       *
       * Converts the input value to total encoder ticks, then splits into
       * revolutions and angle_ticks.
       *
       * @param value Position magnitude in the specified unit
       * @param unit The unit of the position value
       * @param parent Pointer to parent ServoXxdModbus (for steps_per_revolution, required for STEPS unit)
       */
      Position(float value, PositionUnit unit, const ServoXxdModbus *parent);

      /**
       * @brief Default constructor (position = 0)
       */
      Position() = default;

      /**
       * @brief Construct a Position from total encoder ticks
       *
       * Splits total_ticks into revolutions and angle_ticks.
       *
       * @param total_ticks Total encoder ticks (signed)
       * @return Position object in split format
       */
      static Position from_ticks_total(int64_t total_ticks);

      /**
       * @brief Construct a Position from split encoder format
       *
       * @param revolutions Full rotations (int32_t)
       * @param angle_ticks Angle within rotation (uint16_t, 0-16383)
       * @return Position object
       */
      static Position from_parts(int32_t revolutions, uint16_t angle_ticks);

      /**
       * @brief Get full revolutions component
       * @return Number of complete revolutions (signed)
       */
      int32_t revolutions() const { return revs_; }

      /**
       * @brief Get angle ticks component
       * @return Angle within current revolution (0-16383)
       */
      uint16_t angle_ticks() const { return angle_ticks_; }

      /**
       * @brief Get total encoder ticks
       * @return Total ticks = (revs × 16384) + angle_ticks
       */
      int64_t ticks_total() const;

      /**
       * @brief Get position in steps
       *
       * Converts encoder ticks to steps using steps_per_revolution.
       *
       * @param parent Pointer to parent ServoXxdModbus (for steps_per_revolution)
       * @return Position in steps (signed)
       */
      int32_t steps(const ServoXxdModbus *parent) const;

      /**
       * @brief Get position in degrees
       * @return Position in degrees (0-360 per revolution)
       */
      float degrees() const;

      /**
       * @brief Get position in radians
       * @return Position in radians (0-2π per revolution)
       */
      float radians() const;

      /**
       * @brief Get position in arcminutes
       * @return Position in arcminutes (1° = 60 arcmin)
       */
      float arcminutes() const;

      /**
       * @brief Get position in arcseconds
       * @return Position in arcseconds (1° = 3600 arcsec)
       */
      float arcseconds() const;

      /**
       * @brief Get position in steps as unsigned (for ESPHome compatibility)
       *
       * Note: ESPHome base class uses uint32_t for position. This method
       * converts signed steps to unsigned (wraps around if negative).
       *
       * @param parent Pointer to parent ServoXxdModbus (for steps_per_revolution)
       * @return Position in steps (unsigned)
       */
      uint32_t steps_as_u32(const ServoXxdModbus *parent) const;

      // Operators for position arithmetic
      Position operator+(const Position &rhs) const;
      Position operator-(const Position &rhs) const;
      bool operator==(const Position &rhs) const;
      bool operator!=(const Position &rhs) const { return !(*this == rhs); }

    private:
      static constexpr uint32_t TICKS_PER_REV = 16384u; ///< Encoder ticks per revolution (2^14)
      int32_t revs_{0};                                 ///< Full revolutions (signed)
      uint16_t angle_ticks_{0};                         ///< Angle within revolution (0-16383)

      /**
       * @brief Normalize angle_ticks to [0, 16383] and carry/borrow to revs_
       *
       * Ensures angle_ticks is always in valid range, adjusting revs_ accordingly.
       */
      void normalize();
    };

  } // namespace servoxxd_modbus
} // namespace esphome
