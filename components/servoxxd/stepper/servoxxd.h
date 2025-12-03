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
#include "servoxxd_modbus.h"
#include <optional>

namespace esphome
{
  namespace servoxxd
  {

    // Forward declarations
    class StepperEngine;

    // Enum definitions for YAML configuration
    enum class ServoType : uint8_t
    {
      SERVO28D = 0,
      SERVO35D = 1,
      SERVO42D = 2,
      SERVO57D = 3,
    };

    enum class ControlMode : uint8_t
    {
      SR_OPEN = 0,  // SR open loop mode
      SR_CLOSE = 1, // SR closed loop mode
      SR_VFOC = 2,  // SR vector FOC mode
    };

    enum class EnPinActive : uint8_t
    {
      EN_LOW = 0,    // EN pin active low (motor enabled when LOW)
      EN_HIGH = 1,   // EN pin active high (motor enabled when HIGH)
      EN_ALWAYS = 2, // Motor always enabled (ignore EN pin)
    };

    enum class OperatingMode : uint8_t
    {
      POSITION = 0, // Position control mode
      SPEED = 1,    // Speed control mode
    };

    enum class EndstopTrigger : uint8_t
    {
      TRIGGER_LOW = 0,  // Endstop triggers on LOW signal
      TRIGGER_HIGH = 1, // Endstop triggers on HIGH signal
    };

    enum class HomingMode : uint8_t
    {
      SENSORLESS = 0, // Sensorless homing using stall detection
      ENDSTOP = 1,    // Homing with physical endstop switch
      VIRTUAL = 2,    // Virtual homing (just set zero)
    };

    enum class HomingDirection : uint8_t
    {
      CW = 0,      // Clockwise
      CCW = 1,     // Counter-clockwise
      NEAREST = 2, // Nearest direction
    };

    enum class Direction : uint8_t
    {
      CW = 0,  // Clockwise
      CCW = 1, // Counter-clockwise
    };

    enum class ZeroingSpeed : uint8_t
    {
      VERY_SLOW = 0,
      SLOW = 1,
      MEDIUM = 2,
      FAST = 3,
      VERY_FAST = 4,
    };

    // Forward declaration for HomingConfig (defined after class for access to Speed)
    class ServoXxd;

    /**
     * @brief Homing configuration structure
     *
     * Stores all homing-related parameters.
     * Mode determines which speed field is active (union).
     */
    struct HomingConfig
    {
      HomingMode mode{HomingMode::ENDSTOP};           ///< Homing mode (determines which speed field is used)
      bool at_startup{false};                         ///< Perform homing at startup
      HomingDirection direction{HomingDirection::CW}; ///< Homing direction

      // Speed - union of two types (mode determines which is active):
      // - VIRTUAL: speed_level (0-4)
      // - ENDSTOP/SENSORLESS: speed (Speed object)
      union
      {
        Speed speed;   ///< For ENDSTOP/SENSORLESS modes
        uint8_t level; ///< For VIRTUAL mode (ZeroingSpeed 0-4)
      };

      EndstopTrigger endstop_trigger{EndstopTrigger::TRIGGER_LOW}; ///< For ENDSTOP mode
      uint16_t current_ma{0};                                      ///< For SENSORLESS mode (0 = use defaults)

      // Constructor - don't initialize union member yet (will be done in ServoXxd constructor)
      HomingConfig() : level(2) {} // Default to MEDIUM for VIRTUAL, will be overwritten for ENDSTOP/SENSORLESS

      // Destructor - clean up Speed if that's the active member
      ~HomingConfig()
      {
        if (mode != HomingMode::VIRTUAL)
          speed.~Speed();
      }

      // Copy constructor
      HomingConfig(const HomingConfig &other)
          : mode(other.mode), at_startup(other.at_startup), direction(other.direction),
            endstop_trigger(other.endstop_trigger), current_ma(other.current_ma)
      {
        if (mode == HomingMode::VIRTUAL)
          level = other.level;
        else
          new (&speed) Speed(other.speed);
      }

      // Copy assignment
      HomingConfig &operator=(const HomingConfig &other)
      {
        if (this != &other)
        {
          // Destroy old Speed if needed
          if (mode != HomingMode::VIRTUAL)
            speed.~Speed();

          mode = other.mode;
          at_startup = other.at_startup;
          direction = other.direction;
          endstop_trigger = other.endstop_trigger;
          current_ma = other.current_ma;

          // Copy union member based on new mode
          if (mode == HomingMode::VIRTUAL)
            level = other.level;
          else
            new (&speed) Speed(other.speed);
        }
        return *this;
      }
    };

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
    class ServoXxd : public stepper::Stepper, public modbus::ModbusDevice, public Component
    {
    public:
      // ==== Action-API Methoden (Stub, TODO: Implementierung) ====
      void set_work_mode(OperatingMode mode); // TODO: Implement
      // void set_microsteps(uint16_t microsteps); // bereits implementiert
      // void set_working_current(uint16_t current_ma); // bereits implementiert
      // void set_holding_current_percent(uint8_t percent); // bereits implementiert
      void set_speed(const Speed &speed);               // TODO: Implement
      void set_acceleration(const Acceleration &accel); // TODO: Implement
      void set_zero();                                  // TODO: Implement
      void report_position(const Position &pos);        // TODO: Implement
      void release_protection();                        // TODO: Implement
      void restart();                                   // TODO: Implement
      void calibrate();                                 // TODO: Implement
      void key_lock();                                  // TODO: Implement
      void key_unlock();                                // TODO: Implement
    public:
      ServoXxd();  // Implemented in .cpp to initialize homing_.speed with valid parent pointer
      ~ServoXxd(); // Implemented in .cpp to avoid incomplete type

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
       * @brief Set microstepping subdivision (per specification)
       *
       * Valid values: 1-256 (hardware supports any value in this range)
       * Affects Speed class hardware compensation (rpm_for_hardware).
       */
      void set_microsteps(uint16_t microsteps)
      {
        // Validate microstepping value (1-256 per spec)
        if (microsteps < 1 || microsteps > 256)
        {
          ESP_LOGE("servoxxd_modbus", "Invalid microsteps: %u (must be 1-256)", microsteps);
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
       * @brief Get current operating mode
       *
       * Used by StepperEngine for mode validation.
       */
      OperatingMode get_operating_mode() const { return operating_mode_; }

      /**
       * @brief Get homing configuration
       *
       * Used by StepperEngine for homing commands.
       */
      const HomingConfig &get_homing_config() const { return homing_; }

      /**
       * @brief Get default speed
       */
      const Speed &get_default_speed() const { return default_speed_; }

      /**
       * @brief Get default acceleration
       */
      const Acceleration &get_default_acceleration() const { return default_acceleration_; }

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

      // Configuration setters for motor parameters
      void set_address(uint8_t addr) { this->address_ = addr; }
      void set_servo_type(ServoType type) { /* Store servo type */ }
      void set_control_mode(ControlMode mode) { /* Store control mode */ }
      void set_working_current(uint16_t ma) { working_current_ = ma; }
      void set_holding_current_percent(uint8_t percent)
      {
        if (percent > 100)
        {
          ESP_LOGE("servoxxd_modbus", "Invalid holding current percent: %u (must be 0-100)", percent);
          return;
        }
        holding_current_percent_ = percent;
      }
      void set_en_pin_active(EnPinActive value) { /* Store EN pin setting */ }
      void set_auto_screen_off(bool enable) { /* Store auto screen off */ }
      void set_lock_keys_at_startup(bool lock) { /* Store key lock setting */ }
      void set_mode(OperatingMode mode) { operating_mode_ = mode; }
      void set_sleep_when_done(uint32_t ms) { /* Store sleep delay */ }

      // Homing configuration setters - grouped together
      void set_homing_mode(HomingMode mode) { homing_.mode = mode; }
      void set_homing_at_startup(bool enable) { homing_.at_startup = enable; }
      void set_homing_direction(HomingDirection direction) { homing_.direction = direction; }

      /**
       * @brief Set homing speed for ENDSTOP/SENSORLESS modes
       * Overload for regular Speed type with value and unit
       */
      void set_homing_speed(float value, SpeedUnit unit)
      {
        // Destroy old Speed if needed, construct new one
        if (homing_.mode != HomingMode::VIRTUAL)
          homing_.speed.~Speed();
        new (&homing_.speed) Speed(value, unit, this);
      }

      /**
       * @brief Set homing speed level for VIRTUAL mode
       * Overload for ZeroingSpeed enum (0-4)
       */
      void set_homing_speed(uint8_t level)
      {
        if (level > 4)
        {
          ESP_LOGE("servoxxd_modbus", "Invalid homing speed level: %u (must be 0-4)", level);
          return;
        }
        homing_.level = level;
      }

      void set_homing_endstop_trigger(EndstopTrigger trigger) { homing_.endstop_trigger = trigger; }
      void set_homing_current(uint16_t ma) { homing_.current_ma = ma; }

      // ============================================================================
      // Public API (called from Actions)
      // ============================================================================

      /**
       * @brief Move to absolute position
       *
       * Minimal implementation: Simply delegates to StepperEngine.
       * TODO later: Add Position Mode validation, error state check
       */
      void move_to(const Position &position, std::optional<Speed> speed = std::nullopt,
                   std::optional<Acceleration> accel = std::nullopt);

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
      void stop(std::optional<Acceleration> decel = std::nullopt);

      /**
       * @brief Run continuously at specified speed
       *
       * Minimal implementation: Simply delegates to StepperEngine.
       * TODO later: Add Speed Mode validation, error state check
       */
      void run_continuous(std::optional<Speed> speed = std::nullopt,
                          std::optional<Acceleration> accel = std::nullopt);

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
      // Core components (4-layer architecture)
      ModbusTransport *transport_{nullptr}; // Layer 4: Transport abstraction
      StepperEngine *engine_{nullptr};      // Layer 2: State machine & movement logic

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
      HomingConfig homing_;

      // Default motion parameters
      Speed default_speed_{100.0f, SpeedUnit::RPM, this};                               ///< Default/max speed for movements
      Acceleration default_acceleration_{1000.0f, AccelerationUnit::RPM_PER_SEC, this}; ///< Default acceleration

      // Operating mode
      OperatingMode operating_mode_{OperatingMode::POSITION}; ///< Current operating mode (POSITION or SPEED)

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

  } // namespace servoxxd
} // namespace esphome
