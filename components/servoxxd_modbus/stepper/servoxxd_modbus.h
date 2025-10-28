#pragma once

#include "esphome/components/modbus/modbus.h"
#include "esphome/components/stepper/stepper.h"
#include "esphome/core/component.h"
#include "servoxxd_speed.h"
#include "servoxxd_acceleration.h"
#include "servoxxd_position.h"

namespace esphome
{
  namespace servoxxd_modbus
  {

    // Forward declarations
    class StepperEngine;

    /**
     * @brief Main component class for ServoXxd stepper motors
     *
     * This is the facade class that integrates with ESPHome. It inherits from:
     * - stepper::Stepper: Provides ESPHome stepper interface
     * - modbus::ModbusDevice: Enables Modbus communication
     * - Component: ESPHome lifecycle management
     *
     * **Architecture:**
     * - Facade pattern: Public API for YAML configuration and actions
     * - Delegates all movement logic to StepperEngine (state machine)
     * - Manages configuration parameters (steps_per_rev, microstepping, currents, etc.)
     * - Synchronizes with ESPHome base class (current_position, target_position, max_speed)
     *
     * **Responsibilities:**
     * - Component lifecycle (setup, loop, dump_config)
     * - YAML configuration validation
     * - Public API for actions (move_to, home, stop, run_continuous, etc.)
     * - Modbus communication setup
     * - Helper methods for unit conversions (steps ↔ ticks)
     *
     * TODO: Implementation required
     * - [ ] Constructor and configuration setters
     * - [ ] setup() - Initialize hardware, validate config
     * - [ ] loop() - Call StepperEngine::update()
     * - [ ] dump_config() - Log configuration
     * - [ ] Public API methods (move_to, home, stop, etc.)
     * - [ ] Helper methods (get_steps_per_revolution, get_microstepping, etc.)
     * - [ ] Modbus callback handlers (response, error, timeout)
     * - [ ] Synchronization with base class (update current_position, target_position)
     * - [ ] Operating mode management (Position Mode vs Speed Mode)
     */
    class ServoXxdModbus : public stepper::Stepper, public modbus::ModbusDevice, public Component
    {
    public:
      ServoXxdModbus() = default;

      // ============================================================================
      // Component Lifecycle
      // ============================================================================

      /**
       * @brief Initialize the component
       *
       * TODO:
       * - Validate configuration (steps_per_rev > 0, microstepping valid, etc.)
       * - Initialize StepperEngine
       * - Query initial motor state (enabled, position, speed, protection)
       * - Set default values to hardware if needed
       */
      void setup() override { /* TODO */ }

      /**
       * @brief Called repeatedly by ESPHome
       *
       * TODO:
       * - Call StepperEngine::update() to process state machine
       * - Handle polling intervals
       * - Update base class position if changed
       */
      void loop() override { /* TODO */ }

      /**
       * @brief Log configuration to console
       *
       * TODO:
       * - Log all configuration parameters
       * - Log current motor state
       * - Log operating mode (Position vs Speed)
       */
      void dump_config() override { /* TODO */ }

      // ============================================================================
      // Configuration (called from Python/YAML)
      // ============================================================================

      /**
       * @brief Set steps per revolution
       *
       * This is the fundamental configuration that affects all unit conversions.
       * Must be set before any movement commands.
       *
       * TODO: Store and validate (must be > 0)
       */
      void set_steps_per_revolution(float steps) { /* TODO */ }

      /**
       * @brief Get steps per revolution
       *
       * Used by Speed, Acceleration, Position classes for unit conversions.
       */
      virtual float get_steps_per_revolution() const { /* TODO: return steps_per_revolution_; */ return 200.0f; }

      /**
       * @brief Set microstepping mode
       *
       * Valid values: 8, 16, 32, 64, 128, 256
       * Affects Speed class hardware compensation (rpm_for_hardware).
       *
       * TODO: Store, validate, and send to hardware
       */
      void set_microstepping(uint16_t microsteps) { /* TODO */ }

      /**
       * @brief Get current microstepping mode
       *
       * Used by Speed class for hardware compensation.
       */
      virtual uint16_t get_microstepping() const { /* TODO: return microstepping_; */ return 16; }

      // TODO: Add more configuration setters:
      // - set_working_current(uint16_t ma)
      // - set_holding_current_percent(uint8_t percent)
      // - set_shaft_direction(bool reverse)
      // - set_en_pin_level(bool active_high)
      // - set_work_mode(WorkMode mode)
      // - set_home_direction(bool cw)
      // - set_home_speed(Speed speed)
      // - set_sleep_when_done(bool enable)
      // - etc.

      // ============================================================================
      // Public API (called from Actions)
      // ============================================================================

      /**
       * @brief Move to absolute position
       *
       * TODO:
       * - Delegate to StepperEngine::move_to()
       * - Validate: Position Mode only
       * - Validate: Not in Error state
       */
      void move_to(const Position &position) { /* TODO */ }

      /**
       * @brief Start homing sequence
       *
       * TODO:
       * - Delegate to StepperEngine::home()
       * - Validate: Position Mode only
       * - Validate: Not in Error state
       */
      void home() { /* TODO */ }

      /**
       * @brief Stop motor with deceleration
       *
       * TODO:
       * - Delegate to StepperEngine::stop()
       * - Works in both Position and Speed modes
       */
      void stop() { /* TODO */ }

      /**
       * @brief Run continuously at specified speed
       *
       * TODO:
       * - Delegate to StepperEngine::run_continuous()
       * - Validate: Speed Mode only
       * - Validate: Not in Error state
       */
      void run_continuous(const Speed &speed) { /* TODO */ }

      /**
       * @brief Emergency stop (immediate halt, no deceleration)
       *
       * TODO:
       * - Delegate to StepperEngine::emergency_stop()
       * - Disables motor immediately
       * - Sets error flag
       */
      void emergency_stop() { /* TODO */ }

      /**
       * @brief Enable motor
       *
       * TODO:
       * - Delegate to StepperEngine::enable()
       * - Sends enable command to hardware
       */
      void enable() { /* TODO */ }

      /**
       * @brief Disable motor
       *
       * TODO:
       * - Delegate to StepperEngine::disable()
       * - Sends disable command to hardware
       */
      void disable() { /* TODO */ }

      // TODO: Add more public API methods:
      // - set_zero() - Set current position as zero
      // - release_protection() - Clear error state
      // - restart() - Restart motor controller
      // - calibrate() - Run motor calibration
      // - key_lock() / key_unlock() - Physical button lock
      // - set_work_mode() - Switch between modes
      // - set_speed() - Update speed for next movement
      // - set_acceleration() - Update acceleration
      // - etc.

      // ============================================================================
      // Modbus Callbacks (called by ModbusDevice base class)
      // ============================================================================

      /**
       * @brief Handle Modbus response
       *
       * TODO:
       * - Parse response data
       * - Delegate to StepperEngine for processing
       */
      void on_modbus_data(const std::vector<uint8_t> &data) override { /* TODO */ }

      /**
       * @brief Handle Modbus error
       *
       * TODO:
       * - Log error
       * - Delegate to StepperEngine for error handling
       */
      void on_modbus_error(uint8_t function_code, uint8_t exception_code) override { /* TODO */ }

      // ============================================================================
      // Helper Methods (used by unit type classes and StepperEngine)
      // ============================================================================

      /**
       * @brief Convert steps to encoder ticks
       *
       * TODO: Implement conversion: ticks = (steps × 16384) / steps_per_rev
       */
      int64_t steps_to_ticks(int32_t steps) const { /* TODO */ return 0; }

      /**
       * @brief Convert encoder ticks to steps
       *
       * TODO: Implement conversion: steps = (ticks × steps_per_rev) / 16384
       */
      int32_t ticks_to_steps(int64_t ticks) const { /* TODO */ return 0; }

    private:
      // TODO: Add member variables:
      // - StepperEngine* engine_{nullptr};
      // - float steps_per_revolution_{200.0f};
      // - uint16_t microstepping_{16};
      // - uint16_t working_current_{500};
      // - uint8_t holding_current_percent_{50};
      // - bool shaft_reversed_{false};
      // - WorkMode work_mode_{WorkMode::CR_OPEN_LOOP};
      // - Speed home_speed_;
      // - Acceleration default_acceleration_;
      // - etc.

      friend class Speed;
      friend class Acceleration;
      friend class Position;
      friend class StepperEngine;
    };

  } // namespace servoxxd_modbus
} // namespace esphome
