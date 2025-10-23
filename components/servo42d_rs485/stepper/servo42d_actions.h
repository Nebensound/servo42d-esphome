#pragma once

#include "esphome/core/automation.h"
#include "servo42d.h"

namespace esphome
{
  namespace servo42d_rs485
  {

    template <typename... Ts>
    class EnableMotorAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      TEMPLATABLE_VALUE(bool, enable)

      void play(Ts... x) override
      {
        this->parent_->enable_motor(this->enable_.value(x...));
      }
    };

    template <typename... Ts>
    class EmergencyStopAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      void play(Ts... x) override
      {
        this->parent_->emergency_stop();
      }
    };

    template <typename... Ts>
    class ReleaseProtectionAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      void play(Ts... x) override
      {
        this->parent_->release_protection();
      }
    };

    template <typename... Ts>
    class CalibrateMotorAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      void play(Ts... x) override
      {
        this->parent_->calibrate_motor();
      }
    };

    template <typename... Ts>
    class GoToZeroAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      TEMPLATABLE_VALUE(bool, enable)
      TEMPLATABLE_VALUE(uint16_t, speed)
      TEMPLATABLE_VALUE(uint16_t, direction)

      void play(Ts... x) override
      {
        this->parent_->go_to_zero(
            this->enable_.value(x...),
            this->speed_.value(x...),
            this->direction_.value(x...));
      }
    };

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

    template <typename... Ts>
    class SetWorkingCurrentAction : public Action<Ts...>, public Parented<Servo42dRs485>
    {
    public:
      TEMPLATABLE_VALUE(uint16_t, current)

      void play(Ts... x) override
      {
        this->parent_->set_working_current(this->current_.value(x...));
      }
    };

  } // namespace servo42d_rs485
} // namespace esphome
