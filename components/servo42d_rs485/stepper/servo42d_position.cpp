#include "servo42d_position.h"
#include "servo42d.h"
#include "servo42d_modbus_commands.h"
#include <cmath>
#include "esphome/core/log.h"

namespace esphome
{
  namespace servo42d_rs485
  {

    static const char *const TAG = "servo42d_rs485.position";

    // ============================================================================
    // Position Control Methods
    // ============================================================================

    void Servo42dPosition::move_to_position_mode1(uint16_t direction, uint16_t acceleration,
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
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dPosition::move_to_position_mode2(uint16_t acceleration, uint16_t speed,
                                                  int32_t abs_steps)
    {
      int8_t microsteps = this->parent_->get_microsteps();

      // Mode 2 expects "pulses" which represent full steps (not microsteps).
      // Our public API uses microsteps, so convert to full steps by dividing by the microstep setting.
      // Do NOT multiply by microsteps here.
      int32_t motor_pulses = abs_steps / microsteps;

      // Register 0xFE: acceleration, speed, abs_pulses_high, abs_pulses_low (per manual section 8.3.5 table)
      std::vector<uint16_t> params(4);
      params[0] = acceleration;                                         // Acceleration first!
      params[1] = speed;                                                // Then speed
      params[2] = static_cast<uint16_t>((motor_pulses >> 16) & 0xFFFF); // High word (preserves sign)
      params[3] = static_cast<uint16_t>(motor_pulses & 0xFFFF);         // Low word

      ESP_LOGD(TAG, "Mode 2: abs_steps=%d (pulses=%d), accel=%u, speed=%u",
               abs_steps, motor_pulses, acceleration, speed);

      auto cmd = std::make_unique<MultiWriteCommand>(
          ModbusRegisters::MultiWrite::POSITION_MODE_2, params);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGD(TAG, "Position mode 2 move command sent");
    } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dPosition::move_to_position_mode3(uint16_t acceleration, uint16_t speed,
                                                  int32_t rel_steps)
    {
      float steps_per_revolution = this->parent_->get_steps_per_revolution();
      int32_t encoder_ticks;

      // If steps_per_revolution is set (> 0), convert steps to encoder ticks
      // Otherwise, ESPHome "steps" ARE encoder ticks (direct mode)
      if (steps_per_revolution > 0.0f)
      {
        // Convert ESPHome steps to encoder ticks (motor's native unit)
        // Motor uses 16384 encoder ticks per revolution
        double encoder_ticks_d = (static_cast<double>(rel_steps) * 16384.0) / static_cast<double>(steps_per_revolution);
        encoder_ticks = static_cast<int32_t>(encoder_ticks_d >= 0.0 ? (encoder_ticks_d + 0.5) : (encoder_ticks_d - 0.5));

        ESP_LOGD(TAG, "move_to_position_mode3: %d steps → %d encoder ticks (steps/rev=%.1f)",
                 rel_steps, encoder_ticks, steps_per_revolution);
      }
      else
      {
        // Direct mode: ESPHome steps = encoder ticks
        encoder_ticks = rel_steps;
        ESP_LOGD(TAG, "move_to_position_mode3: %d encoder ticks (direct mode)", encoder_ticks);
      }

      std::vector<uint16_t> params(4);
      params[0] = acceleration;                                  // Acceleration first (per manual section 8.3.5)
      params[1] = speed;                                         // Then speed
      params[2] = static_cast<uint16_t>(encoder_ticks >> 16);    // High word
      params[3] = static_cast<uint16_t>(encoder_ticks & 0xFFFF); // Low word

      auto cmd = std::make_unique<MultiWriteCommand>(
          ModbusRegisters::MultiWrite::POSITION_MODE_3, params);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGD(TAG, "Position mode 3 move started");
    } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dPosition::move_to_position_mode4(uint16_t acceleration, uint16_t speed,
                                                  int32_t abs_steps)
    {
      float steps_per_revolution = this->parent_->get_steps_per_revolution();
      int32_t encoder_ticks;

      // If steps_per_revolution is set (> 0), convert steps to encoder ticks
      // Otherwise, ESPHome "steps" ARE encoder ticks (direct mode)
      if (steps_per_revolution > 0.0f)
      {
        // Convert ESPHome steps to encoder ticks (motor's native unit)
        // Motor uses 16384 encoder ticks per revolution
        double encoder_ticks_d = (static_cast<double>(abs_steps) * 16384.0) / static_cast<double>(steps_per_revolution);
        encoder_ticks = static_cast<int32_t>(encoder_ticks_d >= 0.0 ? (encoder_ticks_d + 0.5) : (encoder_ticks_d - 0.5));

        ESP_LOGD(TAG, "move_to_position_mode4: %d steps → %d encoder ticks (steps/rev=%.1f)",
                 abs_steps, encoder_ticks, steps_per_revolution);
      }
      else
      {
        // Direct mode: ESPHome steps = encoder ticks
        encoder_ticks = abs_steps;
        ESP_LOGD(TAG, "move_to_position_mode4: %d encoder ticks (direct mode)", encoder_ticks);
      }

      std::vector<uint16_t> params(4);
      params[0] = acceleration;                                  // Acceleration first (per manual section 8.3.5)
      params[1] = speed;                                         // Then speed
      params[2] = static_cast<uint16_t>(encoder_ticks >> 16);    // High word
      params[3] = static_cast<uint16_t>(encoder_ticks & 0xFFFF); // Low word

      auto cmd = std::make_unique<MultiWriteCommand>(
          ModbusRegisters::MultiWrite::POSITION_MODE_4, params);
      cmd->set_completion_callback([](BaseCommand *, bool success)
                                   {
    if (success) {
      ESP_LOGD(TAG, "Position mode 4 move started");
    } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    // ============================================================================
    // Status Query Methods
    // ============================================================================

    void Servo42dPosition::query_motor_status()
    {
      auto cmd = std::make_unique<ReadCommand>(ModbusRegisters::Read::MOTOR_STATUS, 1);
      auto cmd_ptr = cmd.get(); // Store pointer before moving
      cmd->set_completion_callback([this, cmd_ptr](BaseCommand *, bool success)
                                   {
    if (success) {
      const auto& values = cmd_ptr->get_values();
      if (!values.empty()) {
        uint8_t motor_status = static_cast<uint8_t>(values[0]);
        this->parent_->set_motor_status(motor_status);
        ESP_LOGV(TAG, "Motor status: %d", motor_status);
      }
    } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dPosition::query_encoder_value()
    {
      // Read encoder value (carry mode) - Register 0x0030
      // Returns 3 registers: carry (int32) + value (uint16)
      ESP_LOGD(TAG, "query_encoder_value: Requesting encoder from register 0x0030 (3 words)");
      auto cmd = std::make_unique<ReadCommand>(ModbusRegisters::Read::ENCODER_VALUE_CARRY, 3);
      auto cmd_ptr = cmd.get();
      cmd->set_completion_callback([this, cmd_ptr](BaseCommand *, bool success)
                                   {
    ESP_LOGD(TAG, "query_encoder_value: Response received, success=%d", success);
    if (success) {
      const auto &values = cmd_ptr->get_values();
      ESP_LOGD(TAG, "query_encoder_value: Got %zu values", values.size());
      if (values.size() >= 3) {
        // Decode: Register[0,1] = carry (int32, high,low), Register[2] = value (uint16)
        uint32_t hi = static_cast<uint32_t>(static_cast<uint16_t>(values[0]));
        uint32_t lo = static_cast<uint32_t>(static_cast<uint16_t>(values[1]));
        int32_t carry = static_cast<int32_t>((hi << 16) | lo);
        uint16_t value = static_cast<uint16_t>(values[2]);

        ESP_LOGD(TAG, "query_encoder_value: raw values[0]=0x%04X, [1]=0x%04X, [2]=0x%04X", 
                 values[0], values[1], values[2]);
        ESP_LOGD(TAG, "query_encoder_value: carry=%d (0x%08X), value=%u", 
                 carry, static_cast<uint32_t>(carry), value);

        // Total encoder ticks (value ranges 0..16384 per revolution)
        int64_t total_ticks = static_cast<int64_t>(carry) * 16384LL + static_cast<int64_t>(value);

        // Store raw encoder ticks (for get_encoder_ticks())
        this->parent_->set_encoder_value(total_ticks);

        // Map encoder ticks to ESPHome "steps"
        double steps_per_revolution = static_cast<double>(this->parent_->get_steps_per_revolution());
        int32_t encoder_position_steps;
        if (steps_per_revolution > 0.0) {
          // Use rounding to reduce drift
          double steps_d = (static_cast<double>(total_ticks) * steps_per_revolution) / 16384.0;
          encoder_position_steps = static_cast<int32_t>(steps_d >= 0.0 ? (steps_d + 0.5) : (steps_d - 0.5));
        } else {
          // Direct mode: encoder ticks are treated as steps
          encoder_position_steps = static_cast<int32_t>(total_ticks);
        }

        // Encoder is the TRUTH - current_position = encoder_position - offset
        int32_t position_offset = this->parent_->get_position_offset();
        int32_t current_position = encoder_position_steps - position_offset;

        this->parent_->set_encoder_position(encoder_position_steps);
        this->parent_->set_current_position(current_position);

        ESP_LOGD(TAG, "Encoder: carry=%d, value=%u → ticks=%lld → steps=%d, offset=%d → current_pos=%d",
                 carry, value, static_cast<long long>(total_ticks), encoder_position_steps, position_offset, current_position);
      } else {
        ESP_LOGW(TAG, "query_encoder_value: Expected 3 values, got %zu", values.size());
      }
    } else {
      ESP_LOGW(TAG, "query_encoder_value: Command failed!");
    } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dPosition::query_motor_speed()
    {
      auto cmd = std::make_unique<ReadCommand>(ModbusRegisters::Read::MOTOR_SPEED, 1);
      auto cmd_ptr = cmd.get();
      cmd->set_completion_callback([this, cmd_ptr](BaseCommand *, bool success)
                                   {
    if (success) {
      const auto& values = cmd_ptr->get_values();
      if (!values.empty()) {
        int16_t motor_speed = static_cast<int16_t>(values[0]);
        this->parent_->set_motor_speed(motor_speed);
        ESP_LOGV(TAG, "Motor speed: %d RPM", motor_speed);
      }
    } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dPosition::query_pulse_count()
    {
      auto cmd = std::make_unique<ReadCommand>(ModbusRegisters::Read::PULSE_COUNT, 2);
      auto cmd_ptr = cmd.get();
      cmd->set_completion_callback([this, cmd_ptr](BaseCommand *, bool success)
                                   {
    if (success) {
      const auto& values = cmd_ptr->get_values();
      if (values.size() >= 2) {
        int32_t pulse_count = (static_cast<int32_t>(values[0]) << 16) | values[1];
        this->parent_->set_pulse_count(pulse_count);
        // When using Mode 2, pulse_count IS our current position in steps
        this->parent_->set_current_position(pulse_count);
        ESP_LOGV(TAG, "Pulse count / current position: %d", pulse_count);
      }
    } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dPosition::query_angle_error()
    {
      auto cmd = std::make_unique<ReadCommand>(ModbusRegisters::Read::ANGLE_ERROR, 2);
      auto cmd_ptr = cmd.get();
      cmd->set_completion_callback([this, cmd_ptr](BaseCommand *, bool success)
                                   {
    if (success) {
      const auto& values = cmd_ptr->get_values();
      if (values.size() >= 2) {
        int32_t angle_error = (static_cast<int32_t>(values[0]) << 16) | values[1];
        this->parent_->set_angle_error(angle_error);
        ESP_LOGV(TAG, "Angle error: %d", angle_error);
      }
    } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    void Servo42dPosition::query_protection_status()
    {
      auto cmd = std::make_unique<ReadCommand>(ModbusRegisters::Read::PROTECTION_STATUS, 1);
      auto cmd_ptr = cmd.get();
      cmd->set_completion_callback([this, cmd_ptr](BaseCommand *, bool success)
                                   {
    if (success) {
      const auto &values = cmd_ptr->get_values();
      if (!values.empty()) {
        uint8_t status = static_cast<uint8_t>(values[0] & 0xFF);
        // Notify parent; parent will compare prev/curr and handle callback/logging
        this->parent_->process_protection_status(status);
      }
    } });
      this->parent_->get_command_queue()->enqueue(std::move(cmd));
    }

    // ============================================================================
    // Status Access
    // ============================================================================

    bool Servo42dPosition::is_motor_moving() const
    {
      uint8_t motor_status = this->parent_->get_motor_status();
      using namespace ModbusRegisters::MotorStatus;
      return motor_status == MOTOR_SPEED_UP ||
             motor_status == MOTOR_SPEED_DOWN ||
             motor_status == MOTOR_FULL_SPEED ||
             motor_status == MOTOR_IS_HOMING;
    }

  } // namespace servo42d_rs485
} // namespace esphome
