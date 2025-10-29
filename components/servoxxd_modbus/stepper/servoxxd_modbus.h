#pragma once

// Undefine Arduino macros that conflict with our method names
#ifdef degrees
#undef degrees
#endif
#ifdef radians
#undef radians
#endif

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
      ~ServoXxdModbus(); // Implemented in .cpp to avoid incomplete type

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
      void setup() override;

      /**
       * @brief Called repeatedly by ESPHome
       *
       * TODO:
       * - Call StepperEngine::update() to process state machine
       * - Handle polling intervals
       * - Update base class position if changed
       */
      void loop() override;

      /**
       * @brief Log configuration to console
       *
       * TODO:
       * - Log all configuration parameters
       * - Log current motor state
       * - Log operating mode (Position vs Speed)
       */
      void dump_config() override;

      // ============================================================================
      // Configuration (called from Python/YAML)
      // ============================================================================

      /**
       * @brief Set steps per revolution
       *
       * This is the fundamental configuration that affects all unit conversions.
       * Must be set before any movement commands.
       */
      void set_steps_per_revolution(float steps)
      {
        if (steps <= 0.0f)
        {
          ESP_LOGE("servoxxd_modbus", "Invalid steps_per_revolution: %.1f (must be > 0)", steps);
          return;
        }
        steps_per_revolution_ = steps;
      }

      /**
       * @brief Get steps per revolution
       *
       * Used by Speed, Acceleration, Position classes for unit conversions.
       */
      virtual float get_steps_per_revolution() const { return steps_per_revolution_; }

      /**
       * @brief Set microstepping mode
       *
       * Valid values: 1-256 (per specification)
       * Affects Speed class hardware compensation (rpm_for_hardware).
       */
      void set_microstepping(uint16_t microsteps)
      {
        // Validate microstepping value (1-256 per spec)
        if (microsteps < 1 || microsteps > 256)
        {
          ESP_LOGE("servoxxd_modbus", "Invalid microstepping: %u (must be 1-256)", microsteps);
          return;
        }
        microstepping_ = microsteps;
        // TODO: Send to hardware via Modbus command
      }

      /**
       * @brief Get current microstepping mode
       *
       * Used by Speed class for hardware compensation.
       */
      virtual uint16_t get_microstepping() const { return microstepping_; }

      /**
       * @brief Set speed from value and unit (called from Python/YAML)
       * Creates a Speed object internally for configuration.
       */
      void set_speed(float value, SpeedUnit unit)
      {
        // Store as Speed object for later use
        // This will be used as default/max speed for movements
        default_speed_ = Speed(value, unit, this);
      }

      /**
       * @brief Set acceleration from value and unit (called from Python/YAML)
       * Creates an Acceleration object internally for configuration.
       */
      void set_acceleration(float value, AccelerationUnit unit)
      {
        default_acceleration_ = Acceleration(value, unit, this);
      }

      /**
       * @brief Set homing speed from value and unit (called from Python/YAML)
       */
      void set_homing_speed(float value, SpeedUnit unit)
      {
        home_speed_ = Speed(value, unit, this);
      }

      // Configuration setters for motor parameters
      void set_address(uint8_t addr) { this->address_ = addr; }
      void set_servo_type(uint8_t type) { /* Store servo type */ }
      void set_control_mode(uint8_t mode) { /* Store control mode */ }
      void set_working_current(uint16_t ma) { working_current_ = ma; }
      void set_holding_current_percent(float percent) { holding_current_percent_ = static_cast<uint8_t>(percent * 100.0f); }
      void set_en_pin_active(uint8_t value) { /* Store EN pin setting */ }
      void set_auto_screen_off(bool enable) { /* Store auto screen off */ }
      void set_lock_keys_at_startup(bool lock) { /* Store key lock setting */ }
      void set_mode(uint8_t mode) { /* Store operating mode (POSITION/SPEED) */ }
      void set_sleep_when_done(uint32_t ms) { /* Store sleep delay */ }

      // Homing configuration setters
      void set_homing_mode(uint8_t mode) { /* Store homing mode */ }
      void set_homing_direction(uint8_t direction) { home_direction_cw_ = (direction == 0); }
      void set_homing_zeroing_speed(uint8_t level) { /* Store zeroing speed level for VIRTUAL mode */ }
      void set_endstop_trigger(uint8_t trigger) { /* Store endstop trigger level */ }
      void set_homing_current(uint16_t ma) { /* Store homing current for SENSORLESS mode */ }
      void set_homing_at_startup(bool enable) { /* Store homing at startup flag */ }

      // ============================================================================
      // Public API (called from Actions)
      // ============================================================================

      /**
       * @brief Move to absolute position
       *
       * Minimal implementation: Simply delegates to StepperEngine.
       * TODO later: Add Position Mode validation, error state check
       */
      void move_to(const Position &position);

      /**
       * @brief Start homing sequence
       *
       * Minimal implementation: Simply delegates to StepperEngine.
       * TODO later: Add Position Mode validation, error state check
       */
      void home();

      /**
       * @brief Stop motor with deceleration
       *
       * Minimal implementation: Simply delegates to StepperEngine.
       * Works in both Position and Speed modes.
       */
      void stop();

      /**
       * @brief Run continuously at specified speed
       *
       * Minimal implementation: Simply delegates to StepperEngine.
       * TODO later: Add Speed Mode validation, error state check
       */
      void run_continuous(const Speed &speed);

      /**
       * @brief Emergency stop (immediate halt, no deceleration)
       *
       * Minimal implementation: Simply delegates to StepperEngine.
       */
      void emergency_stop();

      /**
       * @brief Enable motor
       *
       * Minimal implementation: Simply delegates to StepperEngine.
       */
      void enable();

      /**
       * @brief Disable motor
       *
       * Minimal implementation: Simply delegates to StepperEngine.
       */
      void disable();

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
      void on_modbus_data(const std::vector<uint8_t> &data) override;

      /**
       * @brief Handle Modbus error
       *
       * TODO:
       * - Log error
       * - Delegate to StepperEngine for error handling
       */
      void on_modbus_error(uint8_t function_code, uint8_t exception_code) override;

      // ============================================================================
      // Helper Methods (used by unit type classes and StepperEngine)
      // ============================================================================

      /**
       * @brief Step/Tick conversion is handled by the Position class
       *
       * The Position class provides tested and validated conversion between
       * steps and encoder ticks. Do not duplicate this logic here.
       *
       * Usage examples:
       *
       * Steps → Ticks:
       *   Position pos(steps_value, PositionUnit::STEPS, this);
       *   int64_t total_ticks = pos.ticks_total();
       *   int32_t revs = pos.revolutions();
       *   uint16_t angle = pos.angle_ticks();
       *
       * Ticks → Steps:
       *   Position pos = Position::from_ticks_total(total_ticks);
       *   int32_t steps = pos.steps();
       *
       * Split Format → Steps:
       *   Position pos = Position::from_parts(revolutions, angle_ticks);
       *   int32_t steps = pos.steps();
       *
       * Benefits of using Position class:
       * - Handles split format (revolutions + angle_ticks)
       * - Automatic carry/borrow normalization
       * - All unit conversions in one place
       * - Comprehensive unit tests
       *
       * @see Position for implementation details
       */

    private:
      // Core components
      StepperEngine *engine_{nullptr};

      // Motor configuration
      float steps_per_revolution_{200.0f}; ///< Steps per revolution (typically 200 for 1.8° motors)
      uint16_t microstepping_{16};         ///< Microstepping divisor (8, 16, 32, 64, 128, 256)

      // Current settings
      uint16_t working_current_{500};       ///< Working current in mA (0-2000mA typical)
      uint8_t holding_current_percent_{50}; ///< Holding current as % of working current (0-100)

      // Motor behavior
      bool shaft_reversed_{false};     ///< Reverse shaft direction
      bool en_pin_active_high_{false}; ///< EN pin polarity

      // Homing configuration
      bool home_direction_cw_{true};                   ///< Home in clockwise direction
      Speed home_speed_{100.0f, SpeedUnit::RPM, this}; ///< Speed for homing operations

      // Default motion parameters
      Speed default_speed_{100.0f, SpeedUnit::RPM, this};                               ///< Default/max speed for movements
      Acceleration default_acceleration_{1000.0f, AccelerationUnit::RPM_PER_SEC, this}; ///< Default acceleration

      // Operating mode (TODO: Define WorkMode enum)
      // WorkMode work_mode_{WorkMode::SR_CLOSE_LOOP}; // Serial interface, closed loop

      // Sleep configuration
      bool sleep_when_done_{false}; ///< Enter sleep mode after motion complete

      // TODO: Add more configuration as needed:
      // - Protection thresholds
      // - Calibration parameters
      // - Polling intervals
      // - Timeout values

      friend class Speed;
      friend class Acceleration;
      friend class Position;
      friend class StepperEngine;
    };

  } // namespace servoxxd_modbus
} // namespace esphome
