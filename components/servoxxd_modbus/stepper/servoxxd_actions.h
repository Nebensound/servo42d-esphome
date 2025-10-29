#pragma once

#include "esphome/core/automation.h"
#include "servoxxd_modbus.h"
#include "servoxxd_speed.h"
#include "servoxxd_acceleration.h"
#include "servoxxd_position.h"

namespace esphome
{
  namespace servoxxd_modbus
  {

    /**
     * @brief Action templates for ServoXxd stepper motor control
     *
     * This file defines all 18 action classes that can be used in YAML automations.
     * Each action corresponds to a public method on ServoXxdModbus.
     *
     * **Action Categories:**
     *
     * 1. **Movement Actions (5):**
     *    - SetTargetAction: Move to absolute position (Position Mode)
     *    - RunContinuousAction: Run at constant speed (Speed Mode)
     *    - StopAction: Stop with deceleration
     *    - EmergencyStopAction: Immediate halt
     *    - HomeAction: Run homing sequence
     *
     * 2. **Position Actions (2):**
     *    - ReportPositionAction: Set current position offset
     *    - SetZeroAction: Store current position as zero for virtual homing
     *
     * 3. **Control Actions (2):**
     *    - EnableAction: Enable motor
     *    - DisableAction: Disable motor
     *
     * 4. **Speed/Acceleration Actions (2):**
     *    - SetSpeedAction: Set speed for next movement
     *    - SetAccelerationAction: Set acceleration/deceleration
     *
     * 5. **Configuration Actions (4):**
     *    - SetWorkModeAction: Change work mode (CR_OPEN_LOOP, SR_VFOC, etc.)
     *    - SetWorkingCurrentAction: Set working current (mA)
     *    - SetHoldingCurrentPercentAction: Set holding current (0-100%)
     *    - SetMicrosteppingAction: Set microstepping (8, 16, 32, 64, 128, 256)
     *
     * 6. **System Actions (3):**
     *    - ReleaseProtectionAction: Clear error/protection state
     *    - RestartAction: Restart motor controller
     *    - CalibrateAction: Run motor calibration
     *
     * **Implementation Pattern:**
     * Each action class inherits from Action<> and implements:
     * - play() method that calls the corresponding ServoXxdModbus method
     * - Template setters for unit-based values (position, speed, acceleration)
     * - ESPHome automation system integration
     *
     * TODO: Implementation required for all 18 actions
     * - [ ] SetTargetAction with Position support
     * - [ ] RunContinuousAction with Speed support
     * - [ ] StopAction
     * - [ ] EmergencyStopAction
     * - [ ] HomeAction
     * - [ ] ReportPositionAction with Position support
     * - [ ] SetZeroAction
     * - [ ] EnableAction
     * - [ ] DisableAction
     * - [ ] SetSpeedAction with Speed support
     * - [ ] SetAccelerationAction with Acceleration support
     * - [ ] SetWorkModeAction
     * - [ ] SetWorkingCurrentAction
     * - [ ] SetHoldingCurrentPercentAction
     * - [ ] SetMicrosteppingAction
     * - [ ] ReleaseProtectionAction
     * - [ ] RestartAction
     * - [ ] CalibrateAction
     *
     * @see ServoXxdModbus for component API
     * @see README.md for user-facing documentation
     * @see 01-yaml-api.md for YAML configuration details
     */

    // ============================================================================
    // 1. Movement Actions
    // ============================================================================

    /**
     * @brief Action: Move to absolute position (Position Mode only)
     *
     * YAML: `stepper.set_target`
     *
     * TODO: Implement play() method
     * - Call parent_->move_to(position_)
     * - Handle templatable position values
     */
    template <typename... Ts>
    class SetTargetAction : public Action<Ts...>
    {
    public:
      explicit SetTargetAction(ServoXxdModbus *parent) : parent_(parent) {}

      TEMPLATABLE_VALUE(float, value)

      void set_unit(PositionUnit unit) { unit_ = unit; }

      void play(Ts... x) override
      {
        float value = this->value_.value(x...);
        Position pos(value, unit_, parent_);
        parent_->move_to(pos);
      }

    protected:
      ServoXxdModbus *parent_;
      PositionUnit unit_{PositionUnit::STEPS};
    };

    /**
     * @brief Action: Run continuously at constant speed (Speed Mode only)
     *
     * YAML: `stepper.run_continuous`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class RunContinuousAction : public Action<Ts...>
    {
    public:
      explicit RunContinuousAction(ServoXxdModbus *parent) : parent_(parent) {}

      TEMPLATABLE_VALUE(float, value)

      void set_unit(SpeedUnit unit) { unit_ = unit; }

      void play(Ts... x) override
      {
        float value = this->value_.value(x...);
        Speed speed(value, unit_, parent_);
        parent_->run_continuous(speed);
      }

    protected:
      ServoXxdModbus *parent_;
      SpeedUnit unit_{SpeedUnit::STEPS_PER_SEC};
    };

    /**
     * @brief Action: Stop motor with deceleration
     *
     * YAML: `stepper.stop`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class StopAction : public Action<Ts...>
    {
    public:
      explicit StopAction(ServoXxdModbus *parent) : parent_(parent) {}

      void play(Ts... x) override
      {
        parent_->stop();
      }

    protected:
      ServoXxdModbus *parent_;
    };

    /**
     * @brief Action: Emergency stop (immediate halt)
     *
     * YAML: `stepper.emergency_stop`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class EmergencyStopAction : public Action<Ts...>
    {
    public:
      explicit EmergencyStopAction(ServoXxdModbus *parent) : parent_(parent) {}

      void play(Ts... x) override
      {
        parent_->emergency_stop();
      }

    protected:
      ServoXxdModbus *parent_;
    };

    /**
     * @brief Action: Run homing sequence
     *
     * YAML: `stepper.home`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class HomeAction : public Action<Ts...>
    {
    public:
      explicit HomeAction(ServoXxdModbus *parent) : parent_(parent) {}

      void play(Ts... x) override
      {
        parent_->home();
      }

    protected:
      ServoXxdModbus *parent_;
    };

    // ============================================================================
    // 2. Position Actions
    // ============================================================================

    /**
     * @brief Action: Set current position offset
     *
     * YAML: `stepper.report_position`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class ReportPositionAction : public Action<Ts...>
    {
    public:
      explicit ReportPositionAction(ServoXxdModbus *parent) : parent_(parent) {}

      TEMPLATABLE_VALUE(float, value)

      void set_unit(PositionUnit unit) { unit_ = unit; }

      void play(Ts... x) override
      {
        // TODO: Implement report_position in ServoXxdModbus first
        // float value = this->value_.value(x...);
        // Position pos(value, unit_, parent_);
        // parent_->report_position(pos);
      }

    protected:
      ServoXxdModbus *parent_;
      PositionUnit unit_{PositionUnit::STEPS};
    };

    /**
     * @brief Action: Store current position as zero for virtual homing
     *
     * YAML: `stepper.set_zero`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class SetZeroAction : public Action<Ts...>
    {
    public:
      explicit SetZeroAction(ServoXxdModbus *parent) : parent_(parent) {}

      void play(Ts... x) override
      {
        // TODO: Implement set_zero in ServoXxdModbus first
        // parent_->set_zero();
      }

    protected:
      ServoXxdModbus *parent_;
    };

    // ============================================================================
    // 3. Control Actions
    // ============================================================================

    /**
     * @brief Action: Enable motor
     *
     * YAML: `stepper.enable`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class EnableAction : public Action<Ts...>
    {
    public:
      explicit EnableAction(ServoXxdModbus *parent) : parent_(parent) {}

      void play(Ts... x) override
      {
        parent_->enable();
      }

    protected:
      ServoXxdModbus *parent_;
    };

    /**
     * @brief Action: Disable motor
     *
     * YAML: `stepper.disable`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class DisableAction : public Action<Ts...>
    {
    public:
      explicit DisableAction(ServoXxdModbus *parent) : parent_(parent) {}

      void play(Ts... x) override
      {
        parent_->disable();
      }

    protected:
      ServoXxdModbus *parent_;
    };

    // ============================================================================
    // 4. Speed/Acceleration Actions
    // ============================================================================

    /**
     * @brief Action: Set speed for next movement
     *
     * YAML: `stepper.set_speed`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class SetSpeedAction : public Action<Ts...>
    {
    public:
      explicit SetSpeedAction(ServoXxdModbus *parent) : parent_(parent) {}

      TEMPLATABLE_VALUE(float, value)

      void set_unit(SpeedUnit unit) { unit_ = unit; }

      void play(Ts... x) override
      {
        // TODO: Implement set_speed in ServoXxdModbus first
        // float value = this->value_.value(x...);
        // Speed speed(value, unit_, parent_);
        // parent_->set_speed(speed);
      }

    protected:
      ServoXxdModbus *parent_;
      SpeedUnit unit_{SpeedUnit::STEPS_PER_SEC};
    };

    /**
     * @brief Action: Set acceleration/deceleration
     *
     * YAML: `stepper.set_acceleration`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class SetAccelerationAction : public Action<Ts...>
    {
    public:
      explicit SetAccelerationAction(ServoXxdModbus *parent) : parent_(parent) {}

      TEMPLATABLE_VALUE(float, value)

      void set_unit(AccelerationUnit unit) { unit_ = unit; }

      void play(Ts... x) override
      {
        // TODO: Implement set_acceleration in ServoXxdModbus first
        // float value = this->value_.value(x...);
        // Acceleration acc(value, unit_, parent_);
        // parent_->set_acceleration(acc);
      }

    protected:
      ServoXxdModbus *parent_;
      AccelerationUnit unit_{AccelerationUnit::STEPS_PER_SEC_SQ};
    };

    // ============================================================================
    // 5. Configuration Actions
    // ============================================================================

    /**
     * @brief Action: Change work mode
     *
     * YAML: `stepper.set_work_mode`
     *
     * TODO: Implement play() method
     * TODO: Define WorkMode enum
     */
    template <typename... Ts>
    class SetWorkModeAction : public Action<Ts...>
    {
    public:
      explicit SetWorkModeAction(ServoXxdModbus *parent) : parent_(parent) {}

      // TODO: Add WorkMode parameter
      // void set_work_mode(WorkMode mode) { mode_ = mode; }

      void play(Ts... x) override
      {
        // TODO: Implement set_work_mode in ServoXxdModbus first
        // parent_->set_work_mode(mode_);
      }

    protected:
      ServoXxdModbus *parent_;
      // TODO: WorkMode mode_;
    };

    /**
     * @brief Action: Set working current (mA)
     *
     * YAML: `stepper.set_working_current`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class SetWorkingCurrentAction : public Action<Ts...>
    {
    public:
      explicit SetWorkingCurrentAction(ServoXxdModbus *parent) : parent_(parent) {}

      TEMPLATABLE_VALUE(uint16_t, current)

      void play(Ts... x) override
      {
        // TODO: Implement set_working_current in ServoXxdModbus first
        // uint16_t current = this->current_.value(x...);
        // parent_->set_working_current(current);
      }

    protected:
      ServoXxdModbus *parent_;
    };

    /**
     * @brief Action: Set holding current percent (0-100%)
     *
     * YAML: `stepper.set_holding_current_percent`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class SetHoldingCurrentPercentAction : public Action<Ts...>
    {
    public:
      explicit SetHoldingCurrentPercentAction(ServoXxdModbus *parent) : parent_(parent) {}

      TEMPLATABLE_VALUE(uint8_t, percent)

      void play(Ts... x) override
      {
        // TODO: Implement set_holding_current_percent in ServoXxdModbus first
        // uint8_t percent = this->percent_.value(x...);
        // parent_->set_holding_current_percent(percent);
      }

    protected:
      ServoXxdModbus *parent_;
    };

    /**
     * @brief Action: Set microstepping
     *
     * YAML: `stepper.set_microstepping`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class SetMicrosteppingAction : public Action<Ts...>
    {
    public:
      explicit SetMicrosteppingAction(ServoXxdModbus *parent) : parent_(parent) {}

      void set_microstepping(uint16_t microsteps) { microsteps_ = microsteps; }

      void play(Ts... x) override
      {
        parent_->set_microstepping(microsteps_);
      }

    protected:
      ServoXxdModbus *parent_;
      uint16_t microsteps_{16};
    };

    // ============================================================================
    // 6. System Actions
    // ============================================================================

    /**
     * @brief Action: Clear error/protection state
     *
     * YAML: `stepper.release_protection`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class ReleaseProtectionAction : public Action<Ts...>
    {
    public:
      explicit ReleaseProtectionAction(ServoXxdModbus *parent) : parent_(parent) {}

      void play(Ts... x) override
      {
        // TODO: Implement release_protection in ServoXxdModbus first
        // parent_->release_protection();
      }

    protected:
      ServoXxdModbus *parent_;
    };

    /**
     * @brief Action: Restart motor controller
     *
     * YAML: `stepper.restart`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class RestartAction : public Action<Ts...>
    {
    public:
      explicit RestartAction(ServoXxdModbus *parent) : parent_(parent) {}

      void play(Ts... x) override
      {
        // TODO: Implement restart in ServoXxdModbus first
        // parent_->restart();
      }

    protected:
      ServoXxdModbus *parent_;
    };

    /**
     * @brief Action: Run motor calibration
     *
     * YAML: `stepper.calibrate`
     *
     * TODO: Implement play() method
     */
    template <typename... Ts>
    class CalibrateAction : public Action<Ts...>
    {
    public:
      explicit CalibrateAction(ServoXxdModbus *parent) : parent_(parent) {}

      void play(Ts... x) override
      {
        // TODO: Implement calibrate in ServoXxdModbus first
        // parent_->calibrate();
      }

    protected:
      ServoXxdModbus *parent_;
    };

  } // namespace servoxxd_modbus
} // namespace esphome
