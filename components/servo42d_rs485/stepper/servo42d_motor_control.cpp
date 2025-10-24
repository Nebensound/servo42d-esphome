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
      float steps_per_revolution = this->parent_->get_steps_per_revolution();

      if (steps_per_revolution <= 0)
      {
        ESP_LOGE(TAG, "Cannot run_continuous - steps_per_revolution not configured");
        return;
      }

      // Convert steps/s to RPM: RPM = (steps/s / steps_per_revolution) * 60
      float rpm = (speed_steps_per_sec / steps_per_revolution) * 60.0f;

      // Apply direction: 0=CW (positive), 1=CCW (negative)
      int16_t rpm_signed = static_cast<int16_t>(direction == 0 ? rpm : -rpm);

      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::SPEED_MODE,
                                                static_cast<uint16_t>(rpm_signed));
      cmd->set_completion_callback([rpm_signed](BaseCommand *, bool success)
                                   {
        if (success) {
          ESP_LOGI(TAG, "Continuous rotation started at %d RPM", rpm_signed);
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dMotorControl::stop_motor()
    {
      // Stop motor by setting speed to 0 RPM
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::SPEED_MODE, 0x0000);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
        if (success) {
          ESP_LOGI(TAG, "Motor stopped");
        } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dMotorControl::home()
    {
      bool use_virtual_home = this->parent_->get_use_virtual_home();

      if (use_virtual_home)
      {
        // Virtual homing to specified angle
        std::vector<uint16_t> params = {
            static_cast<uint16_t>(this->parent_->get_virtual_home_angle()), // Target angle
            0x0001,                                                         // Enable
            static_cast<uint16_t>(this->parent_->get_homing_speed()),       // Speed in RPM
            this->parent_->get_homing_direction()                           // Direction (0=CW, 1=CCW)
        };

        auto cmd = std::make_unique<MultiWriteCommand>(
            ModbusRegisters::MultiWrite::VIRTUAL_HOMING_PARAMS, params);
        cmd->set_completion_callback([](BaseCommand *, bool success)
                                     {
          if (success) {
            ESP_LOGI(TAG, "Virtual homing started");
          } });
        this->parent_->get_command_queue()->enqueue(std::move(cmd));
      }
      else
      {
        // Real homing with endstop
        std::vector<uint16_t> params = {
            this->parent_->get_homing_direction(),                    // Direction (0=CW, 1=CCW)
            static_cast<uint16_t>(this->parent_->get_homing_speed()), // Speed in RPM
            0x0000,                                                   // Timeout (0=no timeout)
            0x0000                                                    // Padding
        };

        auto cmd = std::make_unique<MultiWriteCommand>(
            ModbusRegisters::MultiWrite::HOMING_PARAMS, params);
        cmd->set_completion_callback([](BaseCommand *, bool success)
                                     {
          if (success) {
            ESP_LOGI(TAG, "Homing started (endstop mode)");
          } });
        this->parent_->get_command_queue()->enqueue(std::move(cmd));
      }
    }

    void Servo42dMotorControl::reset_position()
    {
      // Software baseline reset - make current position the new zero point
      int32_t encoder_value = this->parent_->get_encoder_ticks();
      this->parent_->set_encoder_base_value(encoder_value);
      this->parent_->set_current_position(0);
      ESP_LOGI(TAG, "Position reset - baseline set to %d encoder ticks", encoder_value);
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
      if (mode == 0 || mode == 1 || mode == 3 || mode == 4) {
        auto holding_current = this->parent_->get_holding_current_percent();
        auto hold_cmd = std::make_unique<WriteCommand>(
            ModbusRegisters::Write::HOLDING_CURRENT_PERCENT, holding_current);
        hold_cmd->set_completion_callback([holding_current](BaseCommand*, bool success) {
          if (success) {
            ESP_LOGI(TAG, "Holding current set to %d%%", holding_current);
          }
        });
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
