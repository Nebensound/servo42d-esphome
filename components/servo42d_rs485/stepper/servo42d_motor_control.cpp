#include "servo42d_motor_control.h"
#include "servo42d.h"
#include "servo42d_modbus_commands.h"
#include "esphome/core/log.h"

namespace esphome
{
  namespace servo42d_rs485
  {

    static const char *const TAG = "servo42d_rs485.motor_control";

    void Servo42dMotorControl::enable_motor()
    {
      ESP_LOGI(TAG, "Enabling motor...");
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::EN_CONTROL, 0x0001);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
        if (success) {
          ESP_LOGI(TAG, "Motor enabled successfully");
        } else {
          ESP_LOGW(TAG, "Failed to enable motor");
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dMotorControl::disable_motor()
    {
      ESP_LOGI(TAG, "Disabling motor...");
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::EN_CONTROL, 0x0000);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
        if (success) {
          ESP_LOGI(TAG, "Motor disabled successfully");
        } else {
          ESP_LOGW(TAG, "Failed to disable motor");
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dMotorControl::emergency_stop()
    {
      ESP_LOGW(TAG, "EMERGENCY STOP triggered!");
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::EMERGENCY_STOP, 0x0098);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
        if (success) {
          ESP_LOGI(TAG, "Emergency stop executed");
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dMotorControl::run_continuous(float speed_steps_per_sec, uint8_t direction)
    {
      // Use MODBUS multi-write speed mode command (per manual 8.3.3)
      float steps_per_revolution = this->parent_->get_steps_per_revolution();
      if (steps_per_revolution <= 0)
      {
        ESP_LOGE(TAG, "Cannot run_continuous - steps_per_revolution not configured");
        return;
      }

      // Convert steps/s to RPM and clamp to [0, 3000]
      uint16_t speed_rpm = Servo42dHelpers::steps_per_second_to_rpm(speed_steps_per_sec, steps_per_revolution);

      // Acceleration internal unit 0-255. We use a conservative default if not configured.
      // Note: Exposed configuration for acceleration is not directly accessible here; using default 10.
      uint16_t acc_internal = 10;

      // Build speed mode payload using typed struct
      ModbusRegisters::Payload::SpeedMode payload = {
          .dir = static_cast<uint16_t>(direction ? 1 : 0), // 0=CW, 1=CCW
          .acc = acc_internal,                             // 0-255
          .speed = speed_rpm                               // 0-3000 RPM
      };

      auto cmd = std::make_unique<MultiWriteCommand>(
          ModbusRegisters::MultiWrite::SPEED_MODE,
          Servo42dHelpers::to_vector(payload));
      cmd->set_completion_callback([speed_rpm, direction](BaseCommand *, bool success)
                                   {
        if (success) {
          ESP_LOGI(TAG, "Continuous rotation started: dir=%s, speed=%u RPM", direction ? "CCW" : "CW", speed_rpm);
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dMotorControl::stop_motor()
    {
      // Use speed mode stop with deceleration (acc > 0) to stop smoothly
      ModbusRegisters::Payload::SpeedMode payload = {
          .dir = 0,  // dir ignored when speed=0
          .acc = 4,  // acc small decel
          .speed = 0 // speed=0
      };
      auto cmd = std::make_unique<MultiWriteCommand>(
          ModbusRegisters::MultiWrite::SPEED_MODE,
          Servo42dHelpers::to_vector(payload));
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
        if (success) {
          ESP_LOGI(TAG, "Motor stop initiated (decelerating)");
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dMotorControl::home()
    {
      // Per manual: First set homing parameters (0x0090) then trigger GoHome (0x0091)
      // We currently expose only direction and speed in the parent config; use defaults for others.
      ModbusRegisters::Payload::HomingParams payload = {
          .hm_trig = 0,                                                         // Low active (default)
          .hm_dir = this->parent_->get_homing_direction(),                      // 0=CW, 1=CCW
          .hm_speed = static_cast<uint16_t>(this->parent_->get_homing_speed()), // RPM
          .end_limit = 0                                                        // EndLimit disabled by default
      };

      auto set_params = std::make_unique<MultiWriteCommand>(
          ModbusRegisters::MultiWrite::HOMING_PARAMS,
          Servo42dHelpers::to_vector(payload));
      set_params->set_completion_callback([this](BaseCommand *, bool success)
                                          {
        if (success) {
          ESP_LOGI(TAG, "Homing params set; triggering GoHome");
          auto go_home = std::make_unique<WriteCommand>(ModbusRegisters::Write::GO_HOME, 0x0001);
          go_home->set_completion_callback([](BaseCommand*, bool ok){
            if (ok) ESP_LOGI(TAG, "GoHome command sent");
          });
          this->parent_->get_command_queue()->enqueue(std::move(go_home));
        } else {
          ESP_LOGW(TAG, "Failed to set homing parameters");
        } });
      this->parent_->get_command_queue()->enqueue(std::move(set_params));
    }

    void Servo42dMotorControl::reset_position()
    {
      // Per manual: Set current axis to zero via 0x0092
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::SET_CURRENT_POSITION_ZERO, 0x0001);
      cmd->set_completion_callback([this](BaseCommand *, bool success)
                                   {
        if (success) {
          // Also update local baseline/current position
          int32_t encoder_value = this->parent_->get_encoder_ticks();
          this->parent_->set_encoder_base_value(encoder_value);
          this->parent_->set_current_position(0);
          ESP_LOGI(TAG, "Position reset to zero (0x0092). Baseline=%d ticks", encoder_value);
        } else {
          ESP_LOGW(TAG, "Failed to set current axis to zero (0x0092)");
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dMotorControl::calibrate_motor()
    {
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::CALIBRATION_COMMAND, 0x0001);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
        if (success) {
          ESP_LOGI(TAG, "Motor calibration started");
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dMotorControl::release_protection()
    {
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::RELEASE_PROTECTION_STATE, 0x0001);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
        if (success) {
          ESP_LOGI(TAG, "Protection released");
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dMotorControl::restart_motor()
    {
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::SYSTEM_RESET, 0x0001);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
        if (success) {
          ESP_LOGI(TAG, "Motor restart initiated");
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dMotorControl::set_work_mode(uint16_t mode)
    {
      // 1. Set work mode
      auto mode_cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::WORK_MODE, mode);
      mode_cmd->set_completion_callback([mode](BaseCommand *, bool success)
                                        {
        if (success) {
          const char* mode_names[] = {"CR_OPEN", "CR_CLOSE", "CR_vFOC", "SR_OPEN", "SR_CLOSE", "SR_vFOC"};
          const char* mode_name = (mode < 6) ? mode_names[mode] : "UNKNOWN";
          ESP_LOGI(TAG, "Work mode set to %s (mode %d)", mode_name, mode);
        } });
      this->parent_->get_command_queue()->enqueue(std::move(mode_cmd));

      // 2. Set holding current % for OPEN/CLOSE modes only
      // vFOC modes (2,5) use self-adaption and don't support this register
      if (mode == 0 || mode == 1 || mode == 3 || mode == 4)
      {
        auto holding_current = this->parent_->get_holding_current_percent();
        auto hold_cmd = std::make_unique<WriteCommand>(
            ModbusRegisters::Write::HOLDING_CURRENT_PERCENT, holding_current);
        hold_cmd->set_completion_callback([holding_current](BaseCommand *, bool success)
                                          {
          if (success) {
            ESP_LOGI(TAG, "Holding current set to %d%%", holding_current);
          } });
        this->parent_->get_command_queue()->enqueue(std::move(hold_cmd));
      }
    }

    void Servo42dMotorControl::set_working_current_runtime(uint16_t current_ma)
    {
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::WORKING_CURRENT, current_ma);
      cmd->set_completion_callback([current_ma](BaseCommand *, bool success)
                                   {
        if (success) {
          ESP_LOGI(TAG, "Working current set to %d mA", current_ma);
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dMotorControl::set_holding_current_percent_runtime(uint8_t percent)
    {
      // Only works in SR_OPEN and SR_CLOSE modes, not SR_vFOC
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::HOLDING_CURRENT_PERCENT, percent);
      cmd->set_completion_callback([percent](BaseCommand *, bool success)
                                   {
        if (success) {
          ESP_LOGI(TAG, "Holding current set to %d%%", percent);
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dMotorControl::set_microstepping(uint16_t subdivision)
    {
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::SUBDIVISION, subdivision);
      cmd->set_completion_callback([subdivision](BaseCommand *, bool success)
                                   {
        if (success) {
          ESP_LOGI(TAG, "Microstepping set to 1/%d", subdivision);
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dMotorControl::key_lock()
    {
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::KEY_LOCK, 0x0001);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
        if (success) {
          ESP_LOGI(TAG, "Keys locked");
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dMotorControl::key_unlock()
    {
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::KEY_LOCK, 0x0000);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
        if (success) {
          ESP_LOGI(TAG, "Keys unlocked");
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

  } // namespace servo42d_rs485
} // namespace esphome
