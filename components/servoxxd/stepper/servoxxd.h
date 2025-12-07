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
      SR_OPEN = 3,  // SR open loop mode (serial interface)
      SR_CLOSE = 4, // SR closed loop mode (serial interface)
      SR_VFOC = 5,  // SR vector FOC mode (serial interface)
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

      // Constructor - requires parent pointer for Speed initialization
      // Note: Will be properly initialized in ServoXxd constructor
      HomingConfig() : level(0) {} // Temporary - will be overwritten by ServoXxd constructor

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
      void set_control_mode(ControlMode mode);          // Change control mode at runtime (sends Command 0x82)
      void set_speed(const Speed &speed);               // TODO: Implement
      void set_acceleration(const Acceleration &accel); // TODO: Implement
      void set_zero();                                  // TODO: Implement
      void report_position(const Position &pos);        // TODO: Implement

      // Position synchronization helpers (keep internal Position objects in sync with base class int32_t members)
      void set_current_pos(const Position &pos);        ///< Update current_pos_ and sync base class current_position
      void set_target_pos(const Position &pos);         ///< Update target_pos_ and sync base class target_position

      // Pure delegation methods (inline)
      void release_protection() { this->engine_->release_protection(); }  ///< Clear protection state
      void restart() { this->engine_->restart(); }                        ///< Restart motor controller
      void calibrate() { this->engine_->calibrate(); }                    ///< Start encoder calibration
      void key_lock() { this->engine_->key_lock(); }                      ///< Lock physical buttons
      void key_unlock() { this->engine_->key_unlock(); }                  ///< Unlock physical buttons
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
       * Critical setting - triggers motor restart if changed after setup.
       */
      void set_microsteps(uint16_t microsteps);

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
       * @brief Get control mode (hardware loop type)
       *
       * Used by StepperEngine for setup commands.
       */
      ControlMode get_control_mode() const { return control_mode_; }

      /**
       * @brief Get working current in mA
       *
       * Used by StepperEngine for motor setup.
       */
      uint16_t get_working_current() const { return working_current_; }

      /**
       * @brief Get holding current percentage
       *
       * Used by StepperEngine for motor setup.
       */
      uint8_t get_holding_current_percent() const { return holding_current_percent_; }

      /**
       * @brief Get EN pin active mode
       *
       * Used by StepperEngine for motor setup.
       */
      EnPinActive get_en_pin_active() const { return en_pin_active_; }

      /**
       * @brief Get auto screen off setting
       *
       * Used by StepperEngine for motor setup.
       */
      bool get_auto_screen_off() const { return auto_screen_off_; }

      /**
       * @brief Get lock keys at startup setting
       *
       * Used by StepperEngine for motor setup.
       */
      bool get_lock_keys_at_startup() const { return lock_keys_at_startup_; }

      /**
       * @brief Get homing configuration
       *
       * Used by StepperEngine for homing commands.
       */
      const HomingConfig &get_homing_config() const { return homing_; }

      /**
       * @brief Set homing mode (ENDSTOP, SENSORLESS, VIRTUAL)
       *
       * Called from Python/YAML. Destroys old union member and constructs new one.
       */
      void set_homing_mode(HomingMode mode)
      {
        if (homing_.mode == mode)
          return; // No change

        // Destroy old union member
        if (homing_.mode != HomingMode::VIRTUAL)
          homing_.speed.~Speed();

        // Update mode
        homing_.mode = mode;

        // Construct new union member
        if (mode == HomingMode::VIRTUAL)
          homing_.level = 2; // Default to MEDIUM
        else
          new (&homing_.speed) Speed(100.0f, SpeedUnit::RPM, this); // Default speed
      }

      /**
       * @brief Set homing at startup flag
       */
      void set_homing_at_startup(bool enable) { homing_.at_startup = enable; }

      /**
       * @brief Set homing direction (CW, CCW, NEAREST)
       */
      void set_homing_direction(HomingDirection dir) { homing_.direction = dir; }

      /**
       * @brief Set homing speed for ENDSTOP/SENSORLESS modes
       *
       * Called from Python/YAML with value and unit.
       */
      void set_homing_speed(float value, SpeedUnit unit)
      {
        if (homing_.mode == HomingMode::VIRTUAL)
        {
          ESP_LOGW("servoxxd", "set_homing_speed: ignored for VIRTUAL mode (use set_homing_speed_level)");
          return;
        }
        // Reconstruct Speed object with new value
        homing_.speed.~Speed();
        new (&homing_.speed) Speed(value, unit, this);
      }

      /**
       * @brief Set homing speed level for VIRTUAL mode (0-4)
       *
       * 0=SLOWEST, 1=SLOW, 2=MEDIUM, 3=FAST, 4=FASTEST
       */
      void set_homing_speed_level(uint8_t level)
      {
        if (homing_.mode != HomingMode::VIRTUAL)
        {
          ESP_LOGW("servoxxd", "set_homing_speed_level: ignored for non-VIRTUAL mode (use set_homing_speed)");
          return;
        }
        if (level > 4)
        {
          ESP_LOGE("servoxxd", "Invalid homing speed level: %u (must be 0-4)", level);
          return;
        }
        homing_.level = level;
      }

      /**
       * @brief Set endstop trigger mode (HIGH, LOW) for ENDSTOP mode
       */
      void set_homing_endstop_trigger(EndstopTrigger trigger)
      {
        homing_.endstop_trigger = trigger;
      }

      /**
       * @brief Set homing current threshold for SENSORLESS mode
       *
       * @param current_milliamps Current in mA (0-5200 depending on servo_type)
       */
      void set_homing_current(uint16_t current_milliamps)
      {
        homing_.current_ma = current_milliamps;
      }

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
      void set_en_pin_active(EnPinActive value) { en_pin_active_ = value; }
      void set_auto_screen_off(bool enable) { auto_screen_off_ = enable; }
      void set_lock_keys_at_startup(bool lock) { lock_keys_at_startup_ = lock; }
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
      void home() { this->engine_->home(); }

      /**
       * @brief Stop motor with deceleration
       *
       * Minimal implementation: Simply delegates to StepperEngine.
       * Works in both Position and Speed modes.
       */
      void stop(std::optional<Acceleration> decel = std::nullopt)
      {
        Acceleration actual_decel = decel.has_value() ? decel.value() : this->default_acceleration_;
        this->engine_->stop(actual_decel);
      }

      /**
       * @brief Run continuously at specified speed
       *
       * Minimal implementation: Simply delegates to StepperEngine.
       * TODO later: Add Speed Mode validation, error state check
       */
      void run_continuous(std::optional<Speed> speed = std::nullopt,
                          std::optional<Acceleration> accel = std::nullopt)
      {
        Speed actual_speed = speed.has_value() ? speed.value() : this->default_speed_;
        Acceleration actual_accel = accel.has_value() ? accel.value() : this->default_acceleration_;
        this->engine_->run_continuous(actual_speed, actual_accel);
      }

      /**
       * @brief Emergency stop (immediate halt, no deceleration)
       *
       * Minimal implementation: Simply delegates to StepperEngine.
       */
      void emergency_stop() { this->engine_->emergency_stop(); }

      /**
       * @brief Enable motor
       *
       * Minimal implementation: Simply delegates to StepperEngine.
       */
      void enable() { this->engine_->enable(); }

      /**
       * @brief Disable motor
       *
       * Minimal implementation: Simply delegates to StepperEngine.
       */
      void disable() { this->engine_->disable(); }

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
      bool shaft_reversed_{false};                     ///< Reverse shaft direction
      EnPinActive en_pin_active_{EnPinActive::EN_LOW}; ///< EN pin active level (default: LOW)
      bool auto_screen_off_{true};                     ///< Auto screen off after 15s (default: true)
      bool lock_keys_at_startup_{false};               ///< Lock physical keys at startup (default: false)

      // Homing configuration
      HomingConfig homing_;

      // Default motion parameters
      Speed default_speed_{100.0f, SpeedUnit::RPM, this};                               ///< Default/max speed for movements
      Acceleration default_acceleration_{1000.0f, AccelerationUnit::RPM_PER_SEC, this}; ///< Default acceleration

      // Position tracking (internal Position objects - primary source of truth)
      Position current_pos_{0.0f, PositionUnit::STEPS, this};     ///< Current position (raw encoder + offset)
      Position target_pos_{0.0f, PositionUnit::STEPS, this};      ///< Target position for moves
      Position position_offset_{0.0f, PositionUnit::STEPS, this}; ///< Offset for report_position() zeroing

      // Operating mode
      OperatingMode operating_mode_{OperatingMode::POSITION}; ///< Current operating mode (POSITION or SPEED)

      // Control mode (hardware loop type)
      ControlMode control_mode_{ControlMode::SR_OPEN}; ///< Control mode: SR_OPEN, SR_CLOSE, or SR_VFOC

      // Setup state
      bool is_setup_{false}; ///< True after setup() completes, enables runtime hardware updates

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
