#pragma once

#include <cstdint>
#include <vector>
#include "servo42d_modbus_registers.h"

namespace esphome
{
  namespace servo42d_rs485
  {

    /**
     * Helper functions for unit conversions
     * Converts between steps/s, RPM, and internal motor units
     */
    class Servo42dHelpers
    {
    public:
      // Speed/Acceleration conversion helpers
      static uint16_t steps_per_second_to_rpm(float steps_per_second, float steps_per_revolution);
      static float rpm_to_steps_per_second(uint16_t rpm, float steps_per_revolution);
      static uint16_t acceleration_to_internal(float steps_per_second_sq, float steps_per_revolution);

      // Position conversion helpers - Ticks (encoder hardware: 16384 ticks/rev)
      static float ticks_to_degrees(int64_t ticks);
      static float ticks_to_radians(int64_t ticks);
      static int64_t degrees_to_ticks(float degrees);
      static int64_t radians_to_ticks(float radians);

      // Position conversion helpers - Steps (requires steps_per_revolution)
      static float steps_to_degrees(int32_t steps, float steps_per_revolution);
      static float steps_to_radians(int32_t steps, float steps_per_revolution);
      static int32_t degrees_to_steps(float degrees, float steps_per_revolution);
      static int32_t radians_to_steps(float radians, float steps_per_revolution);

      // Angle normalization utilities
      static float normalize_degrees(float degrees); // 0-359.99...
      static float normalize_radians(float radians); // 0-2π

      // Payload conversion helpers - convert structs to register arrays
      static std::vector<uint16_t> to_vector(const ModbusRegisters::Payload::HomingParams &params);
      static std::vector<uint16_t> to_vector(const ModbusRegisters::Payload::ZeroModeParams &params);
      static std::vector<uint16_t> to_vector(const ModbusRegisters::Payload::SpeedMode &params);
      static std::vector<uint16_t> to_vector(const ModbusRegisters::Payload::PositionMode1 &params);
      static std::vector<uint16_t> to_vector(const ModbusRegisters::Payload::PositionMode2 &params);
      static std::vector<uint16_t> to_vector(const ModbusRegisters::Payload::PositionMode3 &params);
      static std::vector<uint16_t> to_vector(const ModbusRegisters::Payload::PositionMode4 &params);
    };

  } // namespace servo42d_rs485
} // namespace esphome
