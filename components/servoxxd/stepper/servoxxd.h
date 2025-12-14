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

    // State enum (defined in servoxxd_stepper_engine.h)
    enum class State;

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
      NO_HOMING = 0,  // No homing configured
      ENDSTOP = 1,    // Homing with physical endstop switch (used limit switch)
      SENSORLESS = 2, // Sensorless homing using stall detection (no limit switch)
      VIRTUAL = 3,    // Virtual homing (software move to position 0)
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
      HomingMode mode{HomingMode::NO_HOMING};         ///< Homing mode (determines which speed field is used)
      bool at_startup{false};                         ///< Perform homing at startup
      HomingDirection direction{HomingDirection::CW}; ///< Homing direction

      // Speed - union of two types (mode determines which is active):
      // - VIRTUAL: speed_level (ZeroingSpeed enum)
      // - ENDSTOP/SENSORLESS: speed (Speed object)
      union
      {
        Speed speed;        ///< For ENDSTOP/SENSORLESS modes
        ZeroingSpeed level; ///< For VIRTUAL mode
      };

      EndstopTrigger endstop_trigger{EndstopTrigger::TRIGGER_LOW}; ///< For ENDSTOP mode
      uint16_t current_ma{0};                                      ///< For SENSORLESS mode (0 = use defaults)

      // Constructor - requires parent pointer for Speed initialization
      // Note: Will be properly initialized in ServoXxd constructor
      HomingConfig() : level(ZeroingSpeed::MEDIUM) {} // Temporary - will be overwritten by ServoXxd constructor

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
     */
    class ServoXxd : virtual public Component, public stepper::Stepper, public modbus::ModbusDevice
    {
    public:
      // ==== Stepper Compatibility Methods ====
      // These methods provide compatibility with ESPHome's stepper interface
      // and delegate to ServoXxd's Position/Speed/Acceleration objects
      void set_target(int32_t steps);     // Delegate to ServoXxd's Position tracking
      void set_max_speed(float speed);    // Delegate to ServoXxd's Speed objects  
      void set_deceleration(float decel); // Delegate to ServoXxd's Acceleration (decel = accel)
      void set_acceleration(float accel); // Delegate to ServoXxd's Acceleration

      // ==== Action-API Methods ====
      void set_control_mode(ControlMode mode);          // Change control mode at runtime (sends Commandtype 0x82)
      void set_speed(const Speed &speed);               // Update default speed for movements
      void set_acceleration(const Acceleration &accel); // Update default acceleration
      void set_zero();                                  // Store current position as zero (VIRTUAL homing)
      void report_position(const Position &pos);        // Set position offset for zeroing

      // Position synchronization helpers (keep internal Position objects in sync with base class int32_t members)
      void set_current_pos(const Position &pos); ///< Update current_pos_ and sync base class current_position
      void set_target_pos(const Position &pos);  ///< Update target_pos_ and sync base class target_position

      // Pure delegation methods (declared here, implemented in .cpp to avoid incomplete type errors)
      void release_protection(); ///< Clear protection state
      void restart();            ///< Restart motor controller
      void calibrate();          ///< Start encoder calibration
      void key_lock();           ///< Lock physical buttons
      void key_unlock();         ///< Unlock physical buttons
    public:
      ServoXxd();  // Implemented in .cpp to initialize homing_.speed with valid parent pointer
      ~ServoXxd(); // Implemented in .cpp to avoid incomplete type

      // ============================================================================
      // Component Lifecycle
      // ============================================================================

      /**
       * @brief Initialize the component
       *
       * - Validates configuration (steps_per_rev > 0)
       * - Creates ModbusTransport and StepperEngine
       * - Enqueues initial configuration commands
       * - Sets up periodic position synchronization
       */
      void setup() override;

      /**
       * @brief Called repeatedly by ESPHome
       *
       * - Calls StepperEngine::update() for state machine and hardware polling
       * - Checks for external target_position changes
       * - Position sync handled via set_interval (100ms)
       */
      void loop() override;

      /**
       * @brief Log configuration to console
       *
       * Logs:
       * - Operating mode (POSITION/SPEED) and control mode (SR_OPEN/SR_CLOSE/SR_VFOC)
       * - Motor configuration (steps/rev, microstepping, currents)
       * - Homing configuration (mode-specific settings)
       * - Motion parameters (speed, acceleration)
       * - Current state (position, speed, engine state)
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
       * @brief Get State
       *
       * @return State Current state of the motor state machine
       */
      State get_state();

      /**
       * @brief Get state as string
       *
       * @return std::string Current state as string
       */
      std::string get_state_as_string();

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
       * @brief Get current state as string
       *
       * Returns the current state of the motor state machine:
       * "Disabled", "Idle", "Moving", "Running", "Homing", "Calibrating", "Stopping", "Error"
       */
      std::string get_state_string() const;

      /**
       * @brief Set homing mode (ENDSTOP, SENSORLESS, VIRTUAL)
       *
       * Called from Python/YAML. Destroys old union member and constructs new one.
       */
      void set_homing_mode(HomingMode mode)
      {
        if (homing_.mode == mode)
          return; // No change

        // Destroy old union member (if previously not NO_HOMING or VIRTUAL)
        if (homing_.mode != HomingMode::NO_HOMING && homing_.mode != HomingMode::VIRTUAL)
          homing_.speed.~Speed();

        // Update mode
        homing_.mode = mode;

        // Construct new union member
        if (mode == HomingMode::VIRTUAL)
          homing_.level = ZeroingSpeed::MEDIUM; // Default to MEDIUM
        else if (mode != HomingMode::NO_HOMING)
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
       * Called from Python/YAML with Speed object.
       */
      void set_homing_speed(const Speed &speed)
      {
        if (homing_.mode == HomingMode::VIRTUAL)
        {
          ESP_LOGW("servoxxd", "set_homing_speed: ignored for VIRTUAL mode (use set_homing_speed_level)");
          return;
        }
        // Reconstruct Speed object with new value
        homing_.speed.~Speed();
        new (&homing_.speed) Speed(speed);
      }

      /**
       * @brief Set homing speed level for VIRTUAL mode
       *
       * @param level ZeroingSpeed enum (VERY_SLOW, SLOW, MEDIUM, FAST, VERY_FAST)
       */
      void set_homing_speed_level(ZeroingSpeed level)
      {
        if (homing_.mode != HomingMode::VIRTUAL)
        {
          ESP_LOGW("servoxxd", "set_homing_speed_level: ignored for non-VIRTUAL mode (use set_homing_speed)");
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
      void set_servo_type(ServoType type [[maybe_unused]]) { /* Store servo type */ }
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
      void set_sleep_when_done(uint32_t ms [[maybe_unused]]) { /* Store sleep delay */ }

      // ============================================================================
      // Public API (called from Actions)
      // ============================================================================

      /**
       * @brief Move to absolute position
       *
       * Delegates to StepperEngine with default values if parameters not provided.
       * Only valid in POSITION mode.
       */
      void move_to(const Position &position, std::optional<Speed> speed = std::nullopt,
                   std::optional<Acceleration> accel = std::nullopt);

      /**
       * @brief Start homing sequence
       *
       * Delegates to StepperEngine. Behavior depends on homing_.mode configuration.
       * Only valid in POSITION mode.
       */
      void home();

      /**
       * @brief Stop motor with deceleration
       *
       * Works in both Position and Speed modes.
       */
      void stop(std::optional<Acceleration> decel = std::nullopt);

      /**
       * @brief Run continuously at specified speed
       *
       * Delegates to StepperEngine with default values if parameters not provided.
       * Only valid in SPEED mode.
       */
      void run_continuous(std::optional<Speed> speed = std::nullopt,
                          std::optional<Acceleration> accel = std::nullopt);

      /**
       * @brief Emergency stop (immediate halt, no deceleration)
       */
      void emergency_stop();

      /**
       * @brief Enable motor
       */
      void enable();

      /**
       * @brief Disable motor
       */
      void disable();

      // ============================================================================
      // Modbus Callbacks (called by ModbusDevice base class)
      // ============================================================================

      /**
       * @brief Handle Modbus response
       *
       * Forwards response to ModbusTransport for command completion.
       */
      void on_modbus_data(const std::vector<uint8_t> &data) override;

      /**
       * @brief Handle Modbus error
       *
       * Logs error details.
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

      friend class Speed;
      friend class Acceleration;
      friend class Position;
      friend class StepperEngine;
    };

  } // namespace servoxxd
} // namespace esphome
