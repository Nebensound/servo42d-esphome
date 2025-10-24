#pragma once

#include "esphome/core/automation.h"
#include "servo42d.h"

namespace esphome
{
  namespace servo42d_rs485
  {

    // stepper.enable
    template <typename... Ts>
    class EnableMotorAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      void play(Ts... x) override
      {
        this->parent_->enable_motor();
      }
    };

    // stepper.disable
    template <typename... Ts>
    class DisableMotorAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      void play(Ts... x) override
      {
        this->parent_->disable_motor();
      }
    };

    // stepper.emergency_stop
    template <typename... Ts>
    class EmergencyStopAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      void play(Ts... x) override
      {
        this->parent_->emergency_stop();
      }
    };

    // stepper.run_continuous
    template <typename... Ts>
    class RunContinuousAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      TEMPLATABLE_VALUE(float, speed)
      TEMPLATABLE_VALUE(uint8_t, direction)

      void play(Ts... x) override
      {
        this->parent_->run_continuous(
            this->speed_.value(x...),
            this->direction_.value(x...));
      }
    };

    // stepper.stop
    template <typename... Ts>
    class StopMotorAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      void play(Ts... x) override
      {
        this->parent_->stop_motor();
      }
    };

    // stepper.home
    template <typename... Ts>
    class HomeAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      void play(Ts... x) override
      {
        this->parent_->home();
      }
    };

    // stepper.reset_position
    template <typename... Ts>
    class ResetPositionAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      void play(Ts... x) override
      {
        this->parent_->reset_position();
      }
    };

    // stepper.calibrate
    template <typename... Ts>
    class CalibrateMotorAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      void play(Ts... x) override
      {
        this->parent_->calibrate_motor();
      }
    };

    // stepper.release_protection
    template <typename... Ts>
    class ReleaseProtectionAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      void play(Ts... x) override
      {
        this->parent_->release_protection();
      }
    };

    // stepper.restart
    template <typename... Ts>
    class RestartMotorAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      void play(Ts... x) override
      {
        this->parent_->restart_motor();
      }
    };

    // stepper.set_work_mode
    template <typename... Ts>
    class SetWorkModeAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      TEMPLATABLE_VALUE(uint16_t, mode)

      void play(Ts... x) override
      {
        this->parent_->set_work_mode(this->mode_.value(x...));
      }
    };

    // stepper.set_working_current
    template <typename... Ts>
    class SetWorkingCurrentAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      TEMPLATABLE_VALUE(uint16_t, current)

      void play(Ts... x) override
      {
        this->parent_->set_working_current_runtime(this->current_.value(x...));
      }
    };

    // stepper.set_holding_current_percent
    template <typename... Ts>
    class SetHoldingCurrentPercentAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      TEMPLATABLE_VALUE(uint8_t, percent)

      void play(Ts... x) override
      {
        this->parent_->set_holding_current_percent_runtime(this->percent_.value(x...));
      }
    };

    // stepper.set_microstepping
    template <typename... Ts>
    class SetMicrosteppingAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      TEMPLATABLE_VALUE(uint16_t, subdivision)

      void play(Ts... x) override
      {
        this->parent_->set_microstepping(this->subdivision_.value(x...));
      }
    };

    // stepper.key_lock
    template <typename... Ts>
    class KeyLockAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      void play(Ts... x) override
      {
        this->parent_->key_lock();
      }
    };

    // stepper.key_unlock
    template <typename... Ts>
    class KeyUnlockAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      void play(Ts... x) override
      {
        this->parent_->key_unlock();
      }
    };

    // stepper.set_target - Set target position
    template <typename... Ts>
    class SetTargetAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      TEMPLATABLE_VALUE(int32_t, target)

      void play(Ts... x) override
      {
        auto target_value = this->target_.value(x...);
        this->parent_->set_target(target_value);
      }
    };

    // stepper.report_position - Reset position offset
    template <typename... Ts>
    class ReportPositionAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      TEMPLATABLE_VALUE(int32_t, position)

      void play(Ts... x) override
      {
        auto position_value = this->position_.value(x...);
        this->parent_->report_position(position_value);
      }
    };

  } // namespace servo42d_rs485
} // namespace esphome
