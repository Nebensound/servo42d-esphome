#include "servo42d.h"
#include "servo42d_modbus_commands.h"
#include "esphome/core/log.h"

namespace esphome
{
  namespace servo42d_rs485
  {

    static const char *const TAG = "servo42d_rs485";

    void Servo42dRs485::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up Servo42D RS485...");

      // Initialize command queue
      this->command_queue_ = std::make_unique<CommandQueue>();

      // Wait a bit for motor to be ready
      this->set_interval("init_delay", 500, [this]()
                         {
    // Initial configuration sequence
    
    // 1. Enable MODBUS-RTU mode (CRITICAL!)
    auto enable_modbus = std::make_unique<WriteCommand>(
        ModbusRegisters::Write::MODBUS_RTU_ENABLE, 0x0001);
    enable_modbus->set_completion_callback([](BaseCommand*, bool success) {
      if (success) {
        ESP_LOGI(TAG, "MODBUS-RTU enabled successfully");
      } else {
        ESP_LOGW(TAG, "Failed to enable MODBUS-RTU - motor may not respond!");
      }
    });
    this->command_queue_->enqueue(std::move(enable_modbus));
    
    // 2. Set work mode to SR_vFOC (mode 5 - recommended for serial control)
    auto set_mode = std::make_unique<WriteCommand>(
        ModbusRegisters::Write::WORK_MODE, ModbusRegisters::WorkMode::SR_VFOC);
    set_mode->set_completion_callback([](BaseCommand*, bool success) {
      if (success) {
        ESP_LOGI(TAG, "Work mode set to SR_vFOC");
      }
    });
    this->command_queue_->enqueue(std::move(set_mode));
    
    // 3. Set subdivision (microsteps)
    auto set_subdivision = std::make_unique<WriteCommand>(
        ModbusRegisters::Write::SUBDIVISION, this->microsteps_);
    set_subdivision->set_completion_callback([this](BaseCommand*, bool success) {
      if (success) {
        ESP_LOGI(TAG, "Subdivision set to %d microsteps", this->microsteps_);
      }
    });
    this->command_queue_->enqueue(std::move(set_subdivision));
    
    // 4. Query initial motor status
    this->query_motor_status();
    
    // Cancel this one-time setup interval
    this->cancel_interval("init_delay"); });
    }

    void Servo42dRs485::dump_config()
    {
      ESP_LOGCONFIG(TAG, "Servo42D RS485 Stepper:");
      ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
      ESP_LOGCONFIG(TAG, "  Steps per revolution: %.1f", this->steps_per_revolution_);
      ESP_LOGCONFIG(TAG, "  Microsteps: %d", this->microsteps_);
      ESP_LOGCONFIG(TAG, "  Sleep when done: %s", YESNO(this->sleep_when_done_));
      ESP_LOGCONFIG(TAG, "  Max speed: %.1f steps/s", this->max_speed_);
      ESP_LOGCONFIG(TAG, "  Acceleration: %.1f steps/s²", this->acceleration_);
      ESP_LOGCONFIG(TAG, "  Deceleration: %.1f steps/s²", this->deceleration_);
    }

    void Servo42dRs485::loop()
    {
      // Process command queue
      if (this->command_queue_)
      {
        // CommandQueue loop would be called here if it had one
        // For now, commands are processed via MODBUS callbacks
      }
    }

    void Servo42dRs485::update()
    {
      // Regular status polling
      this->query_motor_status();

      // Optionally query other parameters
      if (this->get_update_interval() < 500)
      {
        // Fast polling - also get speed
        this->query_motor_speed();
      }
    }

    void Servo42dRs485::on_modbus_data(const std::vector<uint8_t> &data)
    {
      ESP_LOGV(TAG, "Received MODBUS data: %zu bytes", data.size());

      // Forward response to command queue
      if (this->command_queue_)
      {
        this->command_queue_->handle_response(data);
      }
    }

    void Servo42dRs485::on_modbus_error(uint8_t function_code, uint8_t exception_code)
    {
      ESP_LOGW(TAG, "MODBUS error - Function: 0x%02X, Exception: 0x%02X",
               function_code, exception_code);

      // Forward error to command queue
      if (this->command_queue_)
      {
        this->command_queue_->handle_error(function_code, exception_code);
      }
    }

    // ============================================================================
    // Motor Control Methods
    // ============================================================================

    void Servo42dRs485::enable_motor(bool enable)
    {
      auto cmd = std::make_unique<WriteCommand>(
          ModbusRegisters::Write::MOTOR_ENABLE, enable ? 0x0001 : 0x0000);
      cmd->set_completion_callback([enable](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGI(TAG, "Motor %s", enable ? "enabled" : "disabled");
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    void Servo42dRs485::emergency_stop()
    {
      auto cmd = std::make_unique<WriteCommand>(
          ModbusRegisters::Write::EMERGENCY_STOP, 0x0001);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGW(TAG, "Emergency stop activated!");
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    void Servo42dRs485::release_protection()
    {
      auto cmd = std::make_unique<WriteCommand>(
          ModbusRegisters::Write::RELEASE_PROTECTION, 0x0001);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGI(TAG, "Protection released");
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    void Servo42dRs485::calibrate_motor()
    {
      auto cmd = std::make_unique<WriteCommand>(
          ModbusRegisters::Write::CALIBRATE_MOTOR, 0x0001);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGI(TAG, "Motor calibration started");
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    void Servo42dRs485::go_to_zero(bool enable, uint16_t speed, uint16_t direction)
    {
      // Prepare zero mode parameters
      std::vector<uint16_t> params(4);
      params[0] = 0x0000; // Single turn mode
      params[1] = enable ? 0x0001 : 0x0000;
      params[2] = speed;
      params[3] = direction;

      auto cmd = std::make_unique<MultiWriteCommand>(
          ModbusRegisters::MultiWrite::ZERO_MODE_PARAMS, params);
      cmd->set_completion_callback([enable](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGI(TAG, "Homing %s", enable ? "started" : "stopped");
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    // ============================================================================
    // Position Control Methods
    // ============================================================================

    void Servo42dRs485::move_to_position_mode1(uint16_t direction, uint16_t acceleration,
                                               uint16_t speed, uint16_t pulses)
    {
      std::vector<uint16_t> params = {direction, acceleration, speed, pulses};

      auto cmd = std::make_unique<MultiWriteCommand>(
          ModbusRegisters::MultiWrite::POSITION_MODE_1, params);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGD(TAG, "Position mode 1 move started");
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    void Servo42dRs485::move_to_position_mode2(uint16_t acceleration, uint16_t speed,
                                               uint32_t abs_pulses)
    {
      std::vector<uint16_t> params(4);
      params[0] = acceleration;
      params[1] = speed;
      params[2] = static_cast<uint16_t>(abs_pulses >> 16);    // High word
      params[3] = static_cast<uint16_t>(abs_pulses & 0xFFFF); // Low word

      auto cmd = std::make_unique<MultiWriteCommand>(
          ModbusRegisters::MultiWrite::POSITION_MODE_2, params);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGD(TAG, "Position mode 2 move started");
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    void Servo42dRs485::move_to_position_mode3(uint16_t acceleration, uint16_t speed,
                                               int32_t rel_axis)
    {
      std::vector<uint16_t> params(4);
      params[0] = acceleration;
      params[1] = speed;
      params[2] = static_cast<uint16_t>(rel_axis >> 16);    // High word
      params[3] = static_cast<uint16_t>(rel_axis & 0xFFFF); // Low word

      auto cmd = std::make_unique<MultiWriteCommand>(
          ModbusRegisters::MultiWrite::POSITION_MODE_3, params);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGD(TAG, "Position mode 3 move started");
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    void Servo42dRs485::move_to_position_mode4(uint16_t acceleration, uint16_t speed,
                                               int32_t abs_axis)
    {
      std::vector<uint16_t> params(4);
      params[0] = acceleration;
      params[1] = speed;
      params[2] = static_cast<uint16_t>(abs_axis >> 16);    // High word
      params[3] = static_cast<uint16_t>(abs_axis & 0xFFFF); // Low word

      auto cmd = std::make_unique<MultiWriteCommand>(
          ModbusRegisters::MultiWrite::POSITION_MODE_4, params);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGD(TAG, "Position mode 4 move started");
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    // ============================================================================
    // Status Query Methods
    // ============================================================================

    void Servo42dRs485::query_motor_status()
    {
      auto cmd = std::make_unique<ReadCommand>(ModbusRegisters::Read::MOTOR_STATUS, 1);
      auto cmd_ptr = cmd.get(); // Store pointer before moving
      cmd->set_completion_callback([this, cmd_ptr](BaseCommand *, bool success)
                                   {
    if (success) {
      const auto& values = cmd_ptr->get_values();
      if (!values.empty()) {
        this->motor_status_ = static_cast<uint8_t>(values[0]);
        ESP_LOGV(TAG, "Motor status: %d", this->motor_status_);
      }
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    void Servo42dRs485::query_encoder_value()
    {
      auto cmd = std::make_unique<ReadCommand>(ModbusRegisters::Read::ENCODER_VALUE_CARRY, 3);
      auto cmd_ptr = cmd.get();
      cmd->set_completion_callback([this, cmd_ptr](BaseCommand *, bool success)
                                   {
    if (success) {
      const auto& values = cmd_ptr->get_values();
      if (values.size() >= 3) {
        Int48 encoder = Int48::from_registers(values.data());
        this->encoder_value_ = encoder.to_int64();
        ESP_LOGV(TAG, "Encoder value: %" PRId64, this->encoder_value_);
      }
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    void Servo42dRs485::query_motor_speed()
    {
      auto cmd = std::make_unique<ReadCommand>(ModbusRegisters::Read::MOTOR_SPEED, 1);
      auto cmd_ptr = cmd.get();
      cmd->set_completion_callback([this, cmd_ptr](BaseCommand *, bool success)
                                   {
    if (success) {
      const auto& values = cmd_ptr->get_values();
      if (!values.empty()) {
        this->motor_speed_ = static_cast<int16_t>(values[0]);
        ESP_LOGV(TAG, "Motor speed: %d RPM", this->motor_speed_);
      }
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    void Servo42dRs485::query_pulse_count()
    {
      auto cmd = std::make_unique<ReadCommand>(ModbusRegisters::Read::PULSE_COUNT, 2);
      auto cmd_ptr = cmd.get();
      cmd->set_completion_callback([this, cmd_ptr](BaseCommand *, bool success)
                                   {
    if (success) {
      const auto& values = cmd_ptr->get_values();
      if (values.size() >= 2) {
        this->pulse_count_ = (static_cast<int32_t>(values[0]) << 16) | values[1];
        ESP_LOGV(TAG, "Pulse count: %d", this->pulse_count_);
      }
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    void Servo42dRs485::query_angle_error()
    {
      auto cmd = std::make_unique<ReadCommand>(ModbusRegisters::Read::ANGLE_ERROR, 2);
      auto cmd_ptr = cmd.get();
      cmd->set_completion_callback([this, cmd_ptr](BaseCommand *, bool success)
                                   {
    if (success) {
      const auto& values = cmd_ptr->get_values();
      if (values.size() >= 2) {
        this->angle_error_ = (static_cast<int32_t>(values[0]) << 16) | values[1];
        ESP_LOGV(TAG, "Angle error: %d", this->angle_error_);
      }
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    // ============================================================================
    // Configuration Methods
    // ============================================================================

    void Servo42dRs485::set_work_mode(uint16_t mode)
    {
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::WORK_MODE, mode);
      cmd->set_completion_callback([mode](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGI(TAG, "Work mode set to %d", mode);
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    void Servo42dRs485::set_working_current(uint16_t current_ma)
    {
      if (current_ma > 5200)
      {
        ESP_LOGW(TAG, "Working current limited to 5200mA (requested: %d)", current_ma);
        current_ma = 5200;
      }

      auto cmd = std::make_unique<WriteCommand>(
          ModbusRegisters::Write::WORKING_CURRENT, current_ma);
      cmd->set_completion_callback([current_ma](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGI(TAG, "Working current set to %dmA", current_ma);
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    void Servo42dRs485::set_subdivision(uint16_t subdivision)
    {
      if (subdivision < 1 || subdivision > 256)
      {
        ESP_LOGW(TAG, "Invalid subdivision: %d (must be 1-256)", subdivision);
        return;
      }

      auto cmd = std::make_unique<WriteCommand>(
          ModbusRegisters::Write::SUBDIVISION, subdivision);
      cmd->set_completion_callback([subdivision](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGI(TAG, "Subdivision set to %d", subdivision);
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    void Servo42dRs485::set_en_pin_mode(uint16_t mode)
    {
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::EN_PIN_MODE, mode);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGD(TAG, "EN pin mode updated");
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    void Servo42dRs485::set_direction(uint16_t direction)
    {
      auto cmd = std::make_unique<WriteCommand>(ModbusRegisters::Write::DIRECTION, direction);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGD(TAG, "Direction updated");
    } });
      this->command_queue_->enqueue(std::move(cmd));
    }

    // ============================================================================
    // Status Access
    // ============================================================================

    bool Servo42dRs485::is_motor_moving() const
    {
      using namespace ModbusRegisters::MotorStatus;
      return motor_status_ == MOTOR_SPEED_UP ||
             motor_status_ == MOTOR_SPEED_DOWN ||
             motor_status_ == MOTOR_FULL_SPEED ||
             motor_status_ == MOTOR_IS_HOMING;
    }

    // ============================================================================
    // Conversion Helpers
    // ============================================================================

    uint16_t Servo42dRs485::steps_per_second_to_rpm_(float steps_per_second) const
    {
      // Convert steps/s to RPM based on steps_per_revolution
      float rpm = (steps_per_second / this->steps_per_revolution_) * 60.0f;

      // Clamp to motor limits (typically 0-3000 RPM)
      if (rpm < 0)
        rpm = 0;
      if (rpm > 3000)
        rpm = 3000;

      return static_cast<uint16_t>(rpm);
    }

    float Servo42dRs485::rpm_to_steps_per_second_(uint16_t rpm) const
    {
      return (rpm / 60.0f) * this->steps_per_revolution_;
    }

    uint16_t Servo42dRs485::acceleration_to_internal_(float steps_per_second_sq) const
    {
      // Motor acceleration is 0-255, where higher value = faster acceleration
      // This is a simplified conversion - may need tuning

      float revolutions_per_second_sq = steps_per_second_sq / this->steps_per_revolution_;

      // Scale to 0-255 range (this is approximate)
      uint16_t accel = static_cast<uint16_t>(revolutions_per_second_sq * 10.0f);

      if (accel > 255)
        accel = 255;
      if (accel < 1)
        accel = 1;

      return accel;
    }

  } // namespace servo42d_rs485
} // namespace esphome
