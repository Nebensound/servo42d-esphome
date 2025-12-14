#include "servoxxd_stepper_engine.h"
#include "servoxxd.h"
#include "servoxxd_command_decoder.h"
#include "servoxxd_commands.h"
#include "servoxxd_command_factory.h"
#include "servoxxd_transport.h"
#include <cmath>
#include <cstring>

namespace esphome
{
  namespace servoxxd
  {

    static const char *TAG_ENGINE = "servoxxd.stepper_engine";

    // ============================================================================
    // Constructor / Destructor
    // ============================================================================

    StepperEngine::StepperEngine(ServoXxd *parent, ITransport *transport,
                                 uint32_t command_timeout_ms)
        : parent_(parent),
          queue_(nullptr),
          state_(State::SettingUp),
          emergency_flag_(false),
          current_speed_(0.0f, SpeedUnit::RPM, parent),
          motor_enabled_(false),
          protection_triggered_(false),
          state_enter_time_(0),
          disable_pending_(false)
    {
      // Initialize state entry time to current time for proper timeout tracking
      state_enter_time_ = millis();

      // Create CommandQueue (Layer 3) if transport provided
      if (transport != nullptr)
      {
        queue_ = new CommandQueue(transport, command_timeout_ms);
        ESP_LOGD(TAG_ENGINE, "StepperEngine initialized with transport: timeout=%ums",
                 command_timeout_ms);
      }
      else
      {
        ESP_LOGD(TAG_ENGINE, "StepperEngine initialized WITHOUT transport (stub mode)");
      }
    }

    StepperEngine::~StepperEngine()
    {
      if (queue_)
      {
        delete queue_;
        queue_ = nullptr;
      }
    }

    // ============================================================================
    // Hardware Polling Methods
    // ============================================================================

    void StepperEngine::poll_motor_speed()
    {
      // Enqueue read command for motor speed (Commandtype 0x32 READ_CURRENT_SPEED)
      // Expected response: speed_rpm (int16_t) = 2 bytes

      queue_->enqueue(CommandFactory::read_current_speed(), [this](bool success, const Command &cmd)
                      {
        if (success) {
          Speed speed = CommandDecoder::read_current_speed(cmd);
          process_speed_update(speed);
        } }, Priority::BACKGROUND);
    }

    void StepperEngine::poll_motor_status()
    {
      // Enqueue read command for motor status (Commandtype 0x3A READ_MOTOR_STATUS)
      // Expected response: status (uint8_t, 0=STOP, 1=MOVING, 2=HOMING) = 2 bytes (1 register)
      queue_->enqueue(CommandFactory::read_motor_status(), [this](bool success, const Command &cmd)
                      {
        if (!success)
        {
          ESP_LOGW(TAG_ENGINE, "Failed to read motor status");
          return;
        }

        CommandDecoder::MotorStatus status = CommandDecoder::read_motor_status(cmd);
        process_motor_status_update(status); }, Priority::BACKGROUND);
    }

    void StepperEngine::poll_protection_status()
    {
      // Enqueue read command for protection status (Commandtype 0x3E READ_PROTECTION_STATUS)
      // Expected response: protection (uint8_t, 0 = OK, 1 = protected) = 2 bytes (1 register)
      queue_->enqueue(CommandFactory::read_protection_status(), [this](bool success, const Command &cmd)
                      {
        if (success) {
          auto ps = CommandDecoder::read_protection_status(cmd);
          process_protection_update(ps.protected_state ? 1 : 0);
        } }, Priority::BACKGROUND);
    }

    // ============================================================================
    // Motor Setup
    // ============================================================================

    void StepperEngine::setup_motor()
    {
      if (!queue_)
      {
        ESP_LOGE(TAG_ENGINE, "Cannot setup motor: CommandQueue not initialized");
        return;
      }

      // Explicitly transition to SettingUp state
      transition_to(State::SettingUp);

      ESP_LOGCONFIG(TAG_ENGINE, "Enqueuing motor initialization sequence...");

      // 0. Restart motor to ensure clean state (Commandtype 0x41 RESTART)
      // Motor needs ~3-4s to reboot before accepting configuration commands
      queue_->enqueue(CommandFactory::restart(), 
                      [](bool success, const Command &)
                      {
                        if (success)
                        {
                          ESP_LOGD(TAG_ENGINE, "✓ Motor restart initiated, waiting 4000ms...");
                        }
                        else
                        {
                          ESP_LOGW(TAG_ENGINE, "✗ Failed to restart motor");
                        }
                      }, 
                      Priority::NORMAL, 
                      4000); // Wait 4 seconds after restart

      // 1. Read all current configuration from motor (after restart)
      // This allows us to verify the motor's current state before applying new settings
      queue_->enqueue(CommandFactory::read_all_config(),
                      [this](bool success, const Command &cmd)
                      {
                        if (success)
                        {
                          ESP_LOGD(TAG_ENGINE, "✓ Current motor configuration read (%zu bytes)", cmd.response.size());
                          
                          // Decode and log individual configuration values
                          auto config = CommandDecoder::read_all_config(cmd);
                          
                          // Define common string arrays for configuration value names
                          static const char *mode_names[] = {"CR_OPEN", "CR_CLOSE", "CR_vFOC", "SR_OPEN", "SR_CLOSE", "SR_vFOC"};
                          static constexpr size_t mode_names_count = sizeof(mode_names) / sizeof(mode_names[0]);
                          static const char *en_pin_names[] = {"LOW", "HIGH", "ALWAYS"};
                          static constexpr size_t en_pin_names_count = sizeof(en_pin_names) / sizeof(en_pin_names[0]);
                          
                          // Log control settings
                          uint8_t mode_idx = static_cast<uint8_t>(config.mode);
                          ESP_LOGD(TAG_ENGINE, "  Control mode: %s", (mode_idx < mode_names_count) ? mode_names[mode_idx] : "UNKNOWN");
                          ESP_LOGD(TAG_ENGINE, "  Working current: %u mA", config.working_current_ma);
                          ESP_LOGD(TAG_ENGINE, "  Holding current: %u%%", config.holding_current_percent);
                          ESP_LOGD(TAG_ENGINE, "  Microstepping: 1/%u", config.subdivision);
                          
                          // Log pin settings
                          uint8_t en_pin_idx = static_cast<uint8_t>(config.en_pin_active);
                          ESP_LOGD(TAG_ENGINE, "  EN pin active: %s", (en_pin_idx < en_pin_names_count) ? en_pin_names[en_pin_idx] : "UNKNOWN");
                          ESP_LOGD(TAG_ENGINE, "  Shaft reversed: %s", config.shaft_reversed ? "yes" : "no");
                          
                          // Log display and protection
                          ESP_LOGD(TAG_ENGINE, "  Auto screen off: %s", config.auto_screen_off ? "enabled" : "disabled");
                          ESP_LOGD(TAG_ENGINE, "  Protection: 0x%02X", config.protect_enable);
                          ESP_LOGD(TAG_ENGINE, "  Keys: %s", config.key_lock ? "locked" : "unlocked");
                          
                          // Log communication settings
                          ESP_LOGD(TAG_ENGINE, "  Baud rate code: %u", config.baud_rate);
                          ESP_LOGD(TAG_ENGINE, "  Slave address: %u", config.slave_address);
                          ESP_LOGD(TAG_ENGINE, "  MODBUS: %s", config.modbus_enable ? "enabled" : "disabled");
                          
                          // Log homing settings
                          ESP_LOGD(TAG_ENGINE, "  Homing trigger: %s", 
                                   (config.homing_trigger == EndstopTrigger::TRIGGER_LOW) ? "LOW" : "HIGH");
                          ESP_LOGD(TAG_ENGINE, "  Homing direction: %s", 
                                   (config.homing_direction == Direction::CW) ? "CW" : "CCW");
                          ESP_LOGD(TAG_ENGINE, "  Homing speed: %u RPM", config.homing_speed_rpm);
                          ESP_LOGD(TAG_ENGINE, "  Endlimit: %s", config.endlimit_enable ? "enabled" : "disabled");
                          ESP_LOGD(TAG_ENGINE, "  Sensorless mode: %s", config.nolimit_mode ? "enabled" : "disabled");
                          ESP_LOGD(TAG_ENGINE, "  Sensorless current: %u mA", config.nolimit_current_ma);
                          
                          // Log zero mode settings
                          ESP_LOGD(TAG_ENGINE, "  0_Mode: %u, Task: %u, Speed: %u, Direction: %s",
                                   config.zero_mode, config.zero_task, config.zero_speed,
                                   (config.zero_direction == Direction::CW) ? "CW" : "CCW");
                          
                          // Now enqueue configuration updates only for values that differ
                          ESP_LOGCONFIG(TAG_ENGINE, "Checking which configuration values need updates...");
                          
                          // Check and update microstepping
                          uint8_t desired_microstepping = parent_->get_microstepping();
                          if (config.subdivision != desired_microstepping)
                          {
                            ESP_LOGD(TAG_ENGINE, "  Microstepping differs: %u → %u", config.subdivision, desired_microstepping);
                            queue_->enqueue(CommandFactory::set_subdivision(desired_microstepping),
                                            [desired_microstepping](bool success, const Command &)
                                            {
                                              if (success)
                                              {
                                                ESP_LOGD(TAG_ENGINE, "✓ Microstepping updated to %u", desired_microstepping);
                                              }
                                              else
                                              {
                                                ESP_LOGW(TAG_ENGINE, "✗ Failed to update microstepping");
                                              }
                                            });
                          }
                          else
                          {
                            ESP_LOGD(TAG_ENGINE, "  Microstepping unchanged: %u", config.subdivision);
                          }
                          
                          // Check and update EN pin active level
                          EnPinActive desired_en_pin = parent_->get_en_pin_active();
                          if (config.en_pin_active != desired_en_pin)
                          {
                            ESP_LOGD(TAG_ENGINE, "  EN pin active differs: %s → %s", 
                                     en_pin_names[static_cast<uint8_t>(config.en_pin_active)],
                                     en_pin_names[static_cast<uint8_t>(desired_en_pin)]);
                            queue_->enqueue(CommandFactory::set_en_pin_active(desired_en_pin),
                                            [desired_en_pin, en_pin_names](bool success, const Command &)
                                            {
                                              if (success)
                                              {
                                                ESP_LOGD(TAG_ENGINE, "✓ EN pin active updated to %s", 
                                                         en_pin_names[static_cast<uint8_t>(desired_en_pin)]);
                                              }
                                              else
                                              {
                                                ESP_LOGW(TAG_ENGINE, "✗ Failed to update EN pin active level");
                                              }
                                            });
                          }
                          else
                          {
                            ESP_LOGD(TAG_ENGINE, "  EN pin active unchanged");
                          }
                          
                          // Check and update auto screen off
                          bool desired_auto_screen_off = parent_->get_auto_screen_off();
                          if (config.auto_screen_off != desired_auto_screen_off)
                          {
                            ESP_LOGD(TAG_ENGINE, "  Auto screen off differs: %s → %s",
                                     config.auto_screen_off ? "enabled" : "disabled",
                                     desired_auto_screen_off ? "enabled" : "disabled");
                            queue_->enqueue(CommandFactory::set_auto_screen_off(desired_auto_screen_off),
                                            [desired_auto_screen_off](bool success, const Command &)
                                            {
                                              if (success)
                                              {
                                                ESP_LOGD(TAG_ENGINE, "✓ Auto screen off updated to %s", 
                                                         desired_auto_screen_off ? "enabled" : "disabled");
                                              }
                                              else
                                              {
                                                ESP_LOGW(TAG_ENGINE, "✗ Failed to update auto screen off");
                                              }
                                            });
                          }
                          else
                          {
                            ESP_LOGD(TAG_ENGINE, "  Auto screen off unchanged");
                          }
                          
                          // Check and update key lock
                          bool desired_lock_keys = parent_->get_lock_keys_at_startup();
                          if (config.key_lock != desired_lock_keys)
                          {
                            ESP_LOGD(TAG_ENGINE, "  Key lock differs: %s → %s",
                                     config.key_lock ? "locked" : "unlocked",
                                     desired_lock_keys ? "locked" : "unlocked");
                            queue_->enqueue(CommandFactory::set_lock_keys(desired_lock_keys),
                                            [desired_lock_keys](bool success, const Command &)
                                            {
                                              if (success)
                                              {
                                                ESP_LOGD(TAG_ENGINE, "✓ Keys updated to %s", 
                                                         desired_lock_keys ? "locked" : "unlocked");
                                              }
                                              else
                                              {
                                                ESP_LOGW(TAG_ENGINE, "✗ Failed to update key lock");
                                              }
                                            });
                          }
                          else
                          {
                            ESP_LOGD(TAG_ENGINE, "  Key lock unchanged");
                          }
                          
                          // Always set EN trigger config to safe defaults (both disabled)
                          // This is a safety feature that should always be configured
                          ESP_LOGD(TAG_ENGINE, "  Setting EN trigger config to safe defaults (always set)");
                          queue_->enqueue(CommandFactory::set_en_trigger_config(),
                                          [](bool success, const Command &)
                                          {
                                            if (success)
                                            {
                                              ESP_LOGD(TAG_ENGINE, "✓ EN trigger config set to safe defaults");
                                            }
                                            else
                                            {
                                              ESP_LOGW(TAG_ENGINE, "✗ Failed to set EN trigger configuration");
                                            }
                                          });
                          
                          // Check and update control mode
                          ControlMode desired_mode = parent_->get_control_mode();
                          if (config.mode != desired_mode)
                          {
                            uint8_t current_mode_idx = static_cast<uint8_t>(config.mode);
                            uint8_t desired_mode_idx = static_cast<uint8_t>(desired_mode);
                            ESP_LOGD(TAG_ENGINE, "  Control mode differs: %s → %s",
                                     (current_mode_idx < mode_names_count) ? mode_names[current_mode_idx] : "UNKNOWN",
                                     (desired_mode_idx < mode_names_count) ? mode_names[desired_mode_idx] : "UNKNOWN");
                            
                            const char *mode_name = (desired_mode_idx < mode_names_count) ? mode_names[desired_mode_idx] : "UNKNOWN";
                            
                            queue_->enqueue(CommandFactory::set_control_mode(desired_mode),
                                            [mode_name](bool success, const Command &)
                                            {
                                              if (success)
                                              {
                                                ESP_LOGD(TAG_ENGINE, "✓ Control mode updated to %s", mode_name);
                                              }
                                              else
                                              {
                                                ESP_LOGW(TAG_ENGINE, "✗ Failed to update control mode");
                                              }
                                            });
                          }
                          else
                          {
                            ESP_LOGD(TAG_ENGINE, "  Control mode unchanged");
                          }
                          
                          // Check and update holding current (only for SR_OPEN and SR_CLOSE modes)
                          ControlMode control_mode = parent_->get_control_mode();
                          if (control_mode == ControlMode::SR_OPEN || control_mode == ControlMode::SR_CLOSE)
                          {
                            uint8_t desired_holding_percent = parent_->get_holding_current_percent();
                            if (config.holding_current_percent != desired_holding_percent)
                            {
                              ESP_LOGD(TAG_ENGINE, "  Holding current differs: %u%% → %u%%",
                                       config.holding_current_percent, desired_holding_percent);
                              queue_->enqueue(CommandFactory::set_holding_current_percent(desired_holding_percent),
                                              [desired_holding_percent](bool success, const Command &)
                                              {
                                                if (success)
                                                {
                                                  ESP_LOGD(TAG_ENGINE, "✓ Holding current updated to %u%%", desired_holding_percent);
                                                }
                                                else
                                                {
                                                  ESP_LOGW(TAG_ENGINE, "✗ Failed to update holding current");
                                                }
                                              });
                            }
                            else
                            {
                              ESP_LOGD(TAG_ENGINE, "  Holding current unchanged: %u%%", config.holding_current_percent);
                            }
                          }
                          else
                          {
                            ESP_LOGD(TAG_ENGINE, "  Holding current: skipped (not applicable in vFOC mode)");
                          }
                        }
                        else
                        {
                          ESP_LOGW(TAG_ENGINE, "✗ Failed to read motor configuration");
                          ESP_LOGW(TAG_ENGINE, "   Will apply all configuration settings as fallback");
                        }
                      });

      // 2. OPTIONAL: Clear any error/protection states first
      // Only send if motor is actually in error state (motor will reject with 0xFFFF otherwise)
      // Note: This is non-critical and failure is expected if motor is healthy
      queue_->enqueue(CommandFactory::release_protection(),
                      [](bool success, const Command &)
                      {
                        if (success)
                        {
                          ESP_LOGD(TAG_ENGINE, "✓ Protection state cleared");
                        }
                        else
                        {
                          ESP_LOGD(TAG_ENGINE, "  Release protection not needed (motor not in error state)");
                        }
                      });

      // Note: Configuration update commands are now enqueued conditionally
      // in the read_all_config() callback above (lines 137-342)
      // Only values that differ from desired configuration are updated

      // Note: SET_ZERO command is NOT called here during setup.
      // Position zeroing should be done explicitly via set_zero() or during homing.
      // Automatically resetting position during motor initialization could cause unexpected behavior.

      // 10. Configure homing parameters (if homing is enabled)
      // Homing parameters are static (set once at startup, not changed at runtime)
      // Note: Actual homing execution happens via home() method
      auto &homing = parent_->homing_;

      switch (homing.mode)
      {
      case HomingMode::NO_HOMING:
      {
        // No homing configured - disable 0_Mode
        ESP_LOGD(TAG_ENGINE, "No homing configured - disabling 0_Mode");

        queue_->enqueue(CommandFactory::set_zero_mode(),
                        [](bool success, const Command &)
                        {
                          if (success)
                          {
                            ESP_LOGD(TAG_ENGINE, "✓ 0_Mode disabled (no homing)");
                          }
                          else
                          {
                            ESP_LOGW(TAG_ENGINE, "✗ Failed to disable 0_Mode");
                          }
                        });
        break;
      }

      case HomingMode::ENDSTOP:
      {
        // ENDSTOP mode: Set homing parameters via Commandtype 0x90
        ESP_LOGD(TAG_ENGINE, "ENDSTOP homing: Configuring parameters via Commandtype 0x90");
        ESP_LOGD(TAG_ENGINE, "  Hm_Dir=%s, Hm_Speed=%.1f RPM, Trigger=%s, EndLimit=enabled",
                 homing.direction == HomingDirection::CW ? "CW" : "CCW",
                 homing.speed.rpm(),
                 homing.endstop_trigger == EndstopTrigger::TRIGGER_HIGH ? "HIGH" : "LOW");

        // Convert homing parameters to hardware values
        Direction hw_direction = (homing.direction == HomingDirection::CW) ? Direction::CW : Direction::CCW;
        bool endlimit_enable = true; // Enable endstop limit for ENDSTOP mode

        // Disable no-limit homing (SENSORLESS) for ENDSTOP mode
        queue_->enqueue(CommandFactory::set_nolimit_homing_params(),
                        [](bool success, const Command &)
                        {
                          if (success)
                          {
                            ESP_LOGD(TAG_ENGINE, "✓ No-limit homing disabled for ENDSTOP");
                          }
                          else
                          {
                            ESP_LOGW(TAG_ENGINE, "✗ Failed to disable no-limit homing");
                          }
                        });

        queue_->enqueue(CommandFactory::set_homing_parameters(
                            homing.endstop_trigger, hw_direction, homing.speed, endlimit_enable),
                        [](bool success, const Command &)
                        {
                          if (success)
                          {
                            ESP_LOGD(TAG_ENGINE, "✓ ENDSTOP homing parameters configured");
                          }
                          else
                          {
                            ESP_LOGW(TAG_ENGINE, "✗ Failed to configure ENDSTOP homing parameters");
                          }
                        });

        // Disable 0_Mode for ENDSTOP (spec: 0x9A disable)
        queue_->enqueue(CommandFactory::set_zero_mode(),
                        [](bool success, const Command &)
                        {
                          if (success)
                          {
                            ESP_LOGD(TAG_ENGINE, "✓ 0_Mode disabled for ENDSTOP");
                          }
                          else
                          {
                            ESP_LOGW(TAG_ENGINE, "✗ Failed to disable 0_Mode");
                          }
                        });
        break;
      }

      case HomingMode::SENSORLESS:
      {
        // SENSORLESS mode: Set no-limit home parameters once
        Position reverse_angle = Position::from_steps(0, parent_);

        uint16_t homing_current = homing.current_ma;
        queue_->enqueue(CommandFactory::set_nolimit_homing_params(reverse_angle, true, homing_current),
                        [homing_current](bool success, const Command &)
                        {
                          (void)homing_current; // Used in logging
                          if (success)
                          {
                            ESP_LOGD(TAG_ENGINE, "✓ SENSORLESS homing parameters configured (current=%umA)", homing_current);
                          }
                          else
                          {
                            ESP_LOGW(TAG_ENGINE, "✗ Failed to configure SENSORLESS homing parameters");
                          }
                        });

        // Disable 0_Mode for SENSORLESS (spec: 0x9A disable)
        queue_->enqueue(CommandFactory::set_zero_mode(),
                        [](bool success, const Command &)
                        {
                          if (success)
                          {
                            ESP_LOGD(TAG_ENGINE, "✓ 0_Mode disabled for SENSORLESS");
                          }
                          else
                          {
                            ESP_LOGW(TAG_ENGINE, "✗ Failed to disable 0_Mode");
                          }
                        });
        break;
      }

      case HomingMode::VIRTUAL:
      {
        // VIRTUAL mode: Enable 0_Mode with direction and speed parameters
        // Only configure if currently disabled to avoid resetting zero point repeatedly
        ESP_LOGD(TAG_ENGINE, "VIRTUAL homing: Checking motor status");

        CommandFactory::ZeroModeMode mode = homing.direction == HomingDirection::NEAREST
                                                ? CommandFactory::ZeroModeMode::NEAR_MODE
                                                : CommandFactory::ZeroModeMode::DIR_MODE;

        Direction hw_direction = (homing.direction == HomingDirection::CW) ? Direction::CW : Direction::CCW;

        queue_->enqueue(CommandFactory::read_motor_status(),
                        [this, mode, hw_direction, homing](bool success, const Command &cmd)
                        {
                          if (!success)
                          {
                            ESP_LOGW(TAG_ENGINE, "✗ Failed to read motor status");
                            return;
                          }

                          CommandDecoder::MotorStatus status = CommandDecoder::read_motor_status(cmd);

                          // Only configure if motor is not currently homing
                          if (status != CommandDecoder::MotorStatus::HOMING)
                          {
                            ESP_LOGD(TAG_ENGINE, "Motor not homing, configuring 0_Mode now");
                            queue_->enqueue(CommandFactory::set_zero_mode(mode,
                                                                          CommandFactory::ZeroModeTask::SET,
                                                                          homing.level,
                                                                          hw_direction),
                                            [](bool success, const Command &)
                                            {
                                              if (success)
                                              {
                                                ESP_LOGD(TAG_ENGINE, "✓ 0_Mode configured for VIRTUAL homing");
                                              }
                                              else
                                              {
                                                ESP_LOGW(TAG_ENGINE, "✗ Failed to configure 0_Mode");
                                              }
                                            });
                          }
                          else
                          {
                            ESP_LOGD(TAG_ENGINE, "✓ Motor already homing, skipping 0_Mode configuration");
                          }
                        });

        break;
      }
      }

      // 11. Trigger homing if at_startup is enabled
      // Queue will automatically execute this after RESTART delay (4000ms)
      if (homing.at_startup && homing.mode != HomingMode::NO_HOMING)
      {
        ESP_LOGCONFIG(TAG_ENGINE, "Homing at startup enabled - enqueuing home() command");
        // Simply call home() - it will enqueue the appropriate homing command
        // The queue ensures it executes after all setup commands (including RESTART delay)
        home();
      }

      // 12. Final setup completion marker
      // Enqueue a final "dummy" command to mark setup completion
      // When this callback executes, we know all setup commands completed successfully
      queue_->enqueue(CommandFactory::read_motor_status(), [this](bool success, const Command &)
                      {
                        if (success)
                        {
                          ESP_LOGI(TAG_ENGINE, "✓ Motor setup completed successfully");
                          // Only transition to Idle if not already in Homing state
                          // (homing at_startup may have already started)
                          if (state_ == State::SettingUp)
                          {
                            transition_to(State::Idle);
                          }
                        }
                        else
                        {
                          ESP_LOGE(TAG_ENGINE, "✗ Motor setup failed - final status check unsuccessful");
                          transition_to(State::Error);
                        } }, Priority::NORMAL);

      ESP_LOGCONFIG(TAG_ENGINE, "Setup: %d commands enqueued (will execute via CommandQueue)", 13);
    }

    // ============================================================================
    // Main Update Loop
    // ============================================================================

    void StepperEngine::update()
    {
      // 1. Update CommandQueue (process timeouts, execute next command)
      if (queue_)
      {
        queue_->update();
      }

      // 2. Check state-specific timeouts
      check_state_timeouts();

      // 3. Process buffered commands
      if (disable_pending_ && state_ == State::Idle)
      {
        disable_pending_ = false;
        disable(); // Execute buffered disable
      }
    }

    // ============================================================================
    // Hardware Polling
    // ============================================================================

    void StepperEngine::poll_hardware()
    {
      // Poll all status values in sequence
      poll_encoder_position();
      poll_motor_speed();
      poll_motor_status(); // Also handles homing state detection
      // TODO: Register 0x3E might not exist in hardware - investigate
      // poll_protection_status();
    }

    // ============================================================================
    // Movement Commands
    // ============================================================================

    void StepperEngine::move_to(const Position &target, std::optional<Speed> speed,
                                std::optional<Acceleration> accel)
    {
      // Use default values if not provided
      Speed speed_units = speed.has_value() ? speed.value() : parent_->get_default_speed();
      Acceleration accel_units = accel.has_value() ? accel.value() : parent_->get_default_acceleration();

      // Validation: only allowed in Idle state (Position Mode)
      if (!validate_command(__func__, {State::Idle, State::Moving, State::Stopping}))
      {
        return;
      }
      ESP_LOGD(TAG_ENGINE, "move_to(): target=%lld steps, speed=%.2f RPM, accel=%.2f RPM/s",
               static_cast<long long>(target.get_steps()),
               speed_units.rpm(),
               accel_units.get_rpm_per_sec());

      // Note: target_pos_ already updated by caller (ServoXxd::move_to or set_target_pos)
      // Simply send new move command - hardware will update mid-movement
      queue_->enqueue(CommandFactory::move_position_mode_2(target, speed_units, accel_units), nullptr);

      if (state_ == State::Moving || state_ == State::Stopping)
      {
        return;
      }
      transition_to(State::Moving);
    }

    void StepperEngine::stop(std::optional<Acceleration> decel)
    {
      // Validation: allowed in Moving, Running, Homing, Calibrating, Stopping states
      if (state_ == State::Idle)
      {
        ESP_LOGD(TAG_ENGINE, "stop(): Already stopped, no-op");
        return;
      }

      if (!validate_command(__func__, {State::Moving, State::Running, State::Homing, State::Calibrating, State::Stopping}))
      {
        return;
      }

      ESP_LOGD(TAG_ENGINE, "stop(): decel=%.2f RPM/s", decel.has_value() ? decel->get_rpm_per_sec() : 0.0f);

      // Send stop command via queue (Commandtype 0xFE STOP_POSITION_MODE_2)
      Acceleration decel_units = decel.has_value() ? decel.value() : parent_->get_default_acceleration();
      queue_->enqueue(CommandFactory::stop_position_mode_2(decel_units), nullptr);

      transition_to(State::Stopping);
    }

    void StepperEngine::emergency_stop()
    {
      ESP_LOGW(TAG_ENGINE, "emergency_stop(): Immediate halt, clearing queue");

      emergency_flag_ = true;

      // Clear command queue and send emergency stop
      if (queue_)
      {
        queue_->clear(); // Clear all pending commands

        // Send emergency stop command to hardware (Commandtype 0xF7 EMERGENCY_STOP)
        queue_->enqueue(CommandFactory::emergency_stop(), nullptr, Priority::CRITICAL);
      }

      transition_to(State::Error);
    }

    void StepperEngine::home()
    {

      // Get homing configuration from parent
      auto &homing = parent_->homing_;

      // Check if homing is configured
      if (homing.mode == HomingMode::NO_HOMING)
      {
        ESP_LOGW(TAG_ENGINE, "home(): No homing configured - action ignored");
        return;
      }

      ESP_LOGD(TAG_ENGINE, "home(): Starting homing sequence (mode=%d)",
               static_cast<int>(homing.mode));

      // Note: Homing parameters are already configured in setup_motor()
      // This method only triggers the homing sequence

      switch (homing.mode)
      {
      case HomingMode::VIRTUAL:
      {
        // TODO: Implement VIRTUAL homing properly
        ESP_LOGW(TAG_ENGINE, "VIRTUAL homing not yet implemented");
        /*
        // Virtual homing: Move to position 0 using normal positioning
        // Movement limited to ±180° (one revolution max)
        ESP_LOGD(TAG_ENGINE, "  VIRTUAL homing: Moving to position 0");

        Position target = Position::from_steps(0, parent_);

        // Use ZeroingSpeed level to select appropriate speed
        // Map VERY_SLOW(0)→60 RPM, SLOW(1)→120, MEDIUM(2)→180, FAST(3)→240, VERY_FAST(4)→300
        float rpm = 60.0f + (static_cast<uint8_t>(homing.level) * 60.0f);
        Speed speed(rpm, SpeedUnit::RPM, parent_);

        // Get current position
        Position current = Position::from_steps(parent_->current_position, parent_);
        float current_steps = current.get_steps();
        float steps_per_rev = parent_->get_steps_per_revolution();

        // Calculate delta to position 0
        float delta = 0.0f - current_steps;

        // Normalize delta to [-steps_per_rev/2, +steps_per_rev/2] for NEAREST behavior
        while (delta > steps_per_rev / 2.0f)
          delta -= steps_per_rev;
        while (delta < -steps_per_rev / 2.0f)
          delta += steps_per_rev;

        // Store shortest path distance
        float shortest_distance = std::abs(delta);

        // Apply direction constraint
        if (homing.direction == HomingDirection::CW)
        {
          // Force clockwise: if shortest path is CCW (delta < 0), go the long way CW
          if (delta < 0)
            delta = steps_per_rev + delta; // Positive = CW
        }
        else if (homing.direction == HomingDirection::CCW)
        {
          // Force counter-clockwise: if shortest path is CW (delta > 0), go the long way CCW
          if (delta > 0)
            delta = delta - steps_per_rev; // Negative = CCW
        }
        // NEAREST: delta already contains shortest path

        // Validate ±180° constraint
        // Only NEAREST is guaranteed to be within one revolution
        // CW/CCW can require up to a full revolution if forcing the "wrong" direction
        if (homing.direction == HomingDirection::NEAREST)
        {
          // NEAREST is always ≤ 180°
          if (shortest_distance > steps_per_rev / 2.0f + 1.0f) // +1 for float tolerance
          {
            ESP_LOGE(TAG_ENGINE, "✗ VIRTUAL homing: Internal error - NEAREST path exceeds 180°");
            return;
          }
        }
        else
        {
          // CW/CCW: Warn if forced direction requires >180°
          if (std::abs(delta) > steps_per_rev / 2.0f)
          {
            ESP_LOGW(TAG_ENGINE, "VIRTUAL homing: Forced %s direction requires %.1f° movement (>180°)",
                     homing.direction == HomingDirection::CW ? "CW" : "CCW",
                     std::abs(delta) / steps_per_rev * 360.0f);
          }

          // Hard limit: Cannot move more than one full revolution
          if (std::abs(delta) > steps_per_rev)
          {
            ESP_LOGE(TAG_ENGINE, "✗ VIRTUAL homing: Movement exceeds one revolution (%.1f steps > %.1f)",
                     std::abs(delta), steps_per_rev);
            ESP_LOGE(TAG_ENGINE, "  Current position too far from zero - use set_zero() first");
            return;
          }
        }

        // Move to position 0
        move_to(target, speed, parent_->get_default_acceleration());

        ESP_LOGD(TAG_ENGINE, "✓ VIRTUAL homing: Moving %.1f steps (%.1f°) to position 0",
                 delta, delta / steps_per_rev * 360.0f);
        */
        break;
      }

      case HomingMode::ENDSTOP:
      {
        // ENDSTOP homing: Trigger homing sequence (parameters already set in setup)
        ESP_LOGD(TAG_ENGINE, "ENDSTOP homing: Starting sequence (speed=%.1f RPM)",
                 homing.speed.rpm());

        queue_->enqueue(CommandFactory::go_home(),
                        [this](bool success, const Command &)
                        {
                          if (success)
                          {
                            ESP_LOGD(TAG_ENGINE, "✓ ENDSTOP homing started");
                          }
                          else
                          {
                            transition_to(State::Error);
                            ESP_LOGW(TAG_ENGINE, "✗ Failed to start ENDSTOP homing");
                          }
                        });
        break;
      }

      case HomingMode::SENSORLESS:
      {
        // TODO: Implement SENSORLESS homing properly
        ESP_LOGW(TAG_ENGINE, "SENSORLESS homing not yet implemented");
        /*
        // SENSORLESS homing: Start movement (stall detection parameters already set)
        ESP_LOGD(TAG_ENGINE, "  SENSORLESS homing: Starting movement (current=%umA, speed=%.1f RPM)",
                 homing.current_ma, homing.speed.rpm());

        // Move in homing direction until stall detected
        int32_t large_target = (homing.direction == HomingDirection::CW) ? 1000000 : -1000000;
        Position target = Position::from_steps(large_target, parent_);

        queue_->enqueue(CommandFactory::move_position_mode_3(
            homing.speed,
            parent_->get_default_acceleration(),
            target),
                        [this](bool success, const Command &)
                        {
                          if (success)
                          {
                            ESP_LOGD(TAG_ENGINE, "✓ SENSORLESS homing movement started");
                          }
                          else
                          {
                            ESP_LOGW(TAG_ENGINE, "✗ Failed to start SENSORLESS homing");
                            transition_to(State::Error);
                          }
                        });
        */
        break;
      }

      default:
        ESP_LOGE(TAG_ENGINE, "home(): Invalid homing mode: %d", static_cast<int>(homing.mode));
        return;
      }

      transition_to(State::Homing);
    }

    void StepperEngine::run_continuous(std::optional<Speed> speed,
                                       std::optional<Acceleration> accel)
    {
      // Validation: allowed in Idle or Running states (Speed Mode)
      if (!validate_command(__func__, {State::Idle, State::Running}))
      {
        return;
      }

      // Use default values if not provided (requires parent defaults)
      Speed speed_obj = speed.has_value() ? speed.value() : parent_->get_default_speed();
      Acceleration accel_obj = accel.has_value() ? accel.value() : parent_->get_default_acceleration();

      ESP_LOGD(TAG_ENGINE, "run_continuous(): speed=%.2f RPM, accel=%.2f RPM/s",
               speed_obj.rpm(), accel_obj.get_rpm_per_sec());

      // Send speed command via queue (Commandtype 0xF6 MOVE_SPEED_MODE)
      queue_->enqueue(CommandFactory::move_speed_mode(speed_obj, accel_obj), nullptr);

      transition_to(State::Running);
    }

    // ============================================================================
    // Configuration Commands
    // ============================================================================

    void StepperEngine::enable()
    {
      // Allow enable from Disabled, Idle, or Error states
      if (!validate_command(__func__, {State::Disabled, State::Idle}))
      {
        return;
      }

      // If already in Idle state, motor is already enabled
      if (state_ == State::Idle)
      {
        ESP_LOGD(TAG_ENGINE, "enable(): Motor already enabled (state=Idle)");
        return;
      }

      ESP_LOGD(TAG_ENGINE, "enable(): Enabling motor");

      // Send enable command via queue (Commandtype 0xF3 ENABLE_MOTOR)
      queue_->enqueue(CommandFactory::enable_motor(true), nullptr);

      transition_to(State::Idle);
    }

    void StepperEngine::disable()
    {
      // Validate allowed states (explicitly reject Homing and Calibrating)
      if (!validate_command(__func__, {State::Idle, State::Disabled, State::Error, State::Moving, State::Running, State::Stopping}))
      {
        return;
      }

      // During motion: stop first, then disable will be triggered after stop completes
      if (state_ == State::Moving || state_ == State::Running || state_ == State::Stopping)
      {
        ESP_LOGI(TAG_ENGINE, "disable(): Motor in motion - stopping motor first, disable will follow after stop completes");
        disable_pending_ = true;
        stop(); // Stop first, disable() will be called again from update() when Idle is reached
        return;
      }

      // Already disabled - no-op
      if (state_ == State::Disabled)
      {
        ESP_LOGD(TAG_ENGINE, "disable(): Motor already disabled");
        return;
      }

      ESP_LOGD(TAG_ENGINE, "disable(): Disabling motor");

      // Send disable command via queue (Commandtype 0xF3 ENABLE_MOTOR with false)
      queue_->enqueue(CommandFactory::enable_motor(false), nullptr);

      transition_to(State::Disabled);
    }

    void StepperEngine::release_protection()
    {
      if (!validate_command(__func__, {State::Error, State::Idle, State::Disabled}))
      {
        return;
      }

      ESP_LOGD(TAG_ENGINE, "release_protection(): Attempting to clear error state");

      // Always clear internal flags
      protection_triggered_ = false;
      emergency_flag_ = false;

      // Send release protection command via queue (Commandtype 0x3D RELEASE_PROTECTION)
      queue_->enqueue(CommandFactory::release_protection(), [](bool success, const Command &)
                      {
                        if (success)
                        {
                          ESP_LOGD(TAG_ENGINE, "✓ Release protection command sent");
                        }
                        else
                        {
                          ESP_LOGW(TAG_ENGINE, "✗ Failed to send release protection");
                        } });

      // Transition from Error to Idle to allow re-enabling
      if (state_ == State::Error)
      {
        ESP_LOGD(TAG_ENGINE, "  Transitioning from Error to Idle");
        transition_to(State::Idle);
      }
    }

    void StepperEngine::restart()
    {
      ESP_LOGD(TAG_ENGINE, "restart(): Full motor restart");

      // Clear all errors
      protection_triggered_ = false;
      emergency_flag_ = false;

      // Clear queue
      if (queue_)
      {
        queue_->clear();
      }

      // Send restart command to hardware (Commandtype 0x41 RESTART with value 0x0001)
      // Motor needs 3-4 seconds to fully restart - use queue delay mechanism
      queue_->enqueue(CommandFactory::restart(), nullptr, Priority::NORMAL, 4000);

      ESP_LOGD(TAG_ENGINE, "  Motor will restart, next command delayed 4000ms");

      transition_to(State::Idle);
    }

    void StepperEngine::calibrate()
    {
      if (!validate_command(__func__, {State::Idle, State::Disabled}))
      {
        return;
      }

      ESP_LOGD(TAG_ENGINE, "calibrate(): Starting encoder calibration");

      // Send calibrate encoder command via queue (Commandtype 0x80 CALIBRATE_ENCODER)
      queue_->enqueue(CommandFactory::calibrate_encoder(), [this](bool success, const Command &)
                      {
                        if (success)
                        {
                          ESP_LOGD(TAG_ENGINE, "✓ Calibration command sent");
                        }
                        else
                        {
                          ESP_LOGW(TAG_ENGINE, "✗ Failed to start calibration");
                          transition_to(State::Error);
                        } });

      transition_to(State::Calibrating);
    }

    void StepperEngine::key_lock()
    {
      ESP_LOGD(TAG_ENGINE, "key_lock(): Locking physical keys");

      // Send key lock command via queue (Commandtype 0x8F SET_LOCK_KEYS)
      queue_->enqueue(CommandFactory::set_lock_keys(true), nullptr);
    }

    void StepperEngine::key_unlock()
    {
      ESP_LOGD(TAG_ENGINE, "key_unlock(): Unlocking physical keys");

      // Send key unlock command via queue (Commandtype 0x8F SET_LOCK_KEYS)
      queue_->enqueue(CommandFactory::set_lock_keys(false), nullptr);
    }

    void StepperEngine::set_zero()
    {
      if (!validate_command(__func__, {State::Idle}))
      {
        return;
      }

      ESP_LOGD(TAG_ENGINE, "set_zero(): Sending SET_ZERO command to hardware");

      // Update position tracking only after hardware confirms
      queue_->enqueue(CommandFactory::set_zero(), [this](bool success, const Command &)
                      {
        if (!success) {
          ESP_LOGW(TAG_ENGINE, "set_zero: Hardware command failed");
          return;
        }
        
        // Hardware confirmed - reset parent's offset and position tracking
        parent_->position_offset_ = Position(0.0f, PositionUnit::STEPS, parent_);
        parent_->current_position = 0;
        
        ESP_LOGI(TAG_ENGINE, "set_zero: Hardware confirmed, encoder and offset reset to zero"); });
    }

    // ============================================================================
    // Status Queries
    // ============================================================================

    bool StepperEngine::is_moving() const
    {
      return state_ == State::Moving || state_ == State::Running ||
             state_ == State::Homing || state_ == State::Calibrating ||
             state_ == State::Stopping;
    }

    const char *StepperEngine::state_to_string(State state)
    {
      switch (state)
      {
      case State::Disabled:
        return "Disabled";
      case State::SettingUp:
        return "SettingUp";
      case State::Idle:
        return "Idle";
      case State::Moving:
        return "Moving";
      case State::Running:
        return "Running";
      case State::Homing:
        return "Homing";
      case State::Calibrating:
        return "Calibrating";
      case State::Stopping:
        return "Stopping";
      case State::Error:
        return "Error";
      default:
        return "Unknown";
      }
    }

    // ============================================================================
    // Callbacks Registration
    // ============================================================================

    void StepperEngine::set_position_update_callback(std::function<void(Position)> cb)
    {
      position_callback_ = cb;
    }

    void StepperEngine::set_speed_update_callback(std::function<void(Speed)> cb)
    {
      speed_callback_ = cb;
    }

    void StepperEngine::set_protection_callback(std::function<void()> cb)
    {
      protection_callback_ = cb;
    }

    void StepperEngine::set_motor_status_callback(std::function<void(bool)> cb)
    {
      motor_status_callback_ = cb;
    }

    // ============================================================================
    // Transport Access
    // ============================================================================

    ITransport *StepperEngine::get_transport() const
    {
      return queue_ ? queue_->get_transport() : nullptr;
    }

    // ============================================================================
    // Private Methods - State Machine
    // ============================================================================

    void StepperEngine::transition_to(State new_state)
    {
      if (state_ == new_state)
      {
        return; // No change
      }

      [[maybe_unused]] State old_state = state_;
      state_ = new_state;
      state_enter_time_ = millis(); // Track state entry time for timeout monitoring

      ESP_LOGD(TAG_ENGINE, "State transition: %s → %s",
               state_to_string(old_state), state_to_string(new_state));

      // State-specific initialization
      switch (new_state)
      {
      case State::Moving:
        // Start polling position more frequently (optional)
        break;

      case State::Idle:
        // Reset target position
        parent_->target_pos_ = parent_->current_pos_;
        break;

      case State::Error:
        // Stop all motion immediately
        if (queue_)
        {
          queue_->clear();
        }
        break;

      default:
        break;
      }
    }

    bool StepperEngine::validate_command(const char *func_name,
                                         std::initializer_list<State> allowed_states)
    {
      // Special handling for SettingUp state: Only allow critical commands
      if (state_ == State::SettingUp)
      {
        // During setup, only allow emergency stop, normal stop, and protection release
        bool is_critical = (strcmp(func_name, "emergency_stop") == 0 ||
                            strcmp(func_name, "stop") == 0 ||
                            strcmp(func_name, "release_protection") == 0);

        if (!is_critical)
        {
          ESP_LOGW(TAG_ENGINE, "%s(): Rejected during setup - motor still initializing",
                   func_name);
          return false;
        }
        // Critical command during SettingUp - allow it
        return true;
      }

      for (State allowed : allowed_states)
      {
        if (state_ == allowed)
        {
          return true; // Command allowed
        }
      }

      // Command not allowed in current state
      ESP_LOGW(TAG_ENGINE, "%s(): Rejected (state=%s)", func_name, state_to_string(state_));
      return false;
    }

    void StepperEngine::check_state_timeouts()
    {
      // State-specific timeout monitoring
      uint32_t now = millis();
      uint32_t state_duration = now - state_enter_time_;

      switch (state_)
      {
      case State::SettingUp:
        // Maximum setup duration: 30 seconds
        if (state_duration > 30000)
        {
          ESP_LOGE(TAG_ENGINE, "Setup timeout after %u ms", state_duration);
          handle_error("Setup timeout - motor not responding");
        }
        break;

      case State::Homing:
        // Maximum homing duration: 600 seconds
        if (state_duration > 600000)
        {
          ESP_LOGE(TAG_ENGINE, "Homing timeout after %u ms", state_duration);
          handle_error("Homing timeout");
        }
        break;

      case State::Calibrating:
        // Maximum calibration duration: 120 seconds
        if (state_duration > 120000)
        {
          ESP_LOGE(TAG_ENGINE, "Calibration timeout after %u ms", state_duration);
          handle_error("Calibration timeout");
        }
        break;

      case State::Stopping:
        // Maximum stop duration: 50 seconds
        if (state_duration > 50000)
        {
          ESP_LOGE(TAG_ENGINE, "Stopping timeout after %u ms", state_duration);
          // Force transition to Idle even if not at standstill
          transition_to(State::Idle);
        }
        break;

      default:
        // No timeout for other states
        break;
      }
    }

    void StepperEngine::poll_encoder_position(std::function<void(const Position &)> callback)
    {
      // Enqueue read command for encoder position (Commandtype 0x30 READ_ENCODER_CARRY)
      // Expected response: carry (int32_t) + value (uint16_t) = 6 bytes
      queue_->enqueue(CommandFactory::read_encoder_carry(), [this, callback](bool success, const Command &cmd)
                      {
        if (success) {
          auto position = CommandDecoder::read_encoder_carry(cmd, parent_);
          process_encoder_update(position);
          parent_->set_current_pos(position);
        
          // Invoke user callback if provided
          if (callback) {
            callback(position);
          }
        } }, Priority::BACKGROUND);
    }

    // ============================================================================
    // Private Methods - Event Processing
    // ============================================================================

    void StepperEngine::process_encoder_update(const Position &position)
    {
      Position old_position = parent_->current_pos_;
      parent_->set_current_pos(position);

      // Check if target reached (in Moving state)
      if (state_ == State::Moving && is_target_reached())
      {
        ESP_LOGD(TAG_ENGINE, "Target position reached");
        transition_to(State::Idle);
      }

      // Invoke callback if position changed (polled every 100ms, no threshold needed)
      if (parent_->current_pos_.get_steps() != old_position.get_steps() && position_callback_)
      {
        position_callback_(parent_->current_pos_);
      }
    }

    void StepperEngine::process_speed_update(const Speed &speed)
    {
      Speed old_speed = current_speed_;
      current_speed_ = speed;

      // Check if standstill reached (in Stopping state)
      if (state_ == State::Stopping && speed.rpm() == 0)
      {
        ESP_LOGV(TAG_ENGINE, "Standstill reached (speed=0), transitioning to Idle");
        transition_to(State::Idle);
      }

      // Invoke callback if speed changed
      if (speed.rpm() != old_speed.rpm() && speed_callback_)
      {
        speed_callback_(current_speed_);
      }
    }

    void StepperEngine::process_motor_status_update(CommandDecoder::MotorStatus status)
    {
      // Track motor enable state for callback
      bool enabled = (status != CommandDecoder::MotorStatus::STOP && status != CommandDecoder::MotorStatus::FAIL);
      bool old_enabled = motor_enabled_;
      motor_enabled_ = enabled;

      // Notify parent if motor enable state changed
      if (enabled != old_enabled && motor_status_callback_)
      {
        motor_status_callback_(enabled);
      }

      // Hardware status validates Engine state - Engine state is leading
      // Only handle critical errors or completion signals
      switch (status)
      {
      case CommandDecoder::MotorStatus::FAIL:
        // Critical error: Always transition to Error state
        if (state_ != State::Error)
        {
          ESP_LOGW(TAG_ENGINE, "Motor status: FAIL - Hardware reports failure");
          ESP_LOGW(TAG_ENGINE, "  Possible causes: locked rotor, motor not connected, calibration needed");
          transition_to(State::Error);
        }
        break;

      case CommandDecoder::MotorStatus::STOP:
        // Hardware reports standstill - validate against Engine expectations
        // If Engine expects motion but hardware stopped → Error
        if (state_ == State::Moving || state_ == State::Running)
        {
          ESP_LOGW(TAG_ENGINE, "Unexpected stop: Engine expected motion but hardware stopped");
          transition_to(State::Stopping);
        }
        // If Engine is Stopping and hardware confirms → Idle
        else if (state_ == State::Stopping)
        {
          ESP_LOGV(TAG_ENGINE, "Motor status: STOP confirmed, transitioning to Idle");
          transition_to(State::Idle);
        }
        // If Engine is Homing/Calibrating and hardware stopped → Check completion separately
        break;

      case CommandDecoder::MotorStatus::SPEED_UP:
      case CommandDecoder::MotorStatus::SPEED_DOWN:
      case CommandDecoder::MotorStatus::FULL_SPEED:
        // Hardware reports motion - validate against Engine expectations
        // If Engine expects Idle but hardware moving → Inconsistency warning
        if (state_ == State::Idle || state_ == State::Disabled)
        {
          ESP_LOGW(TAG_ENGINE, "Unexpected motion: Hardware moving but engine state is %s",
                   state_to_string(state_));
          // Don't change state - let engine commands control state
        }
        break;

      case CommandDecoder::MotorStatus::HOMING:
        // Hardware reports homing - validate against Engine expectations
        // If Engine is NOT in Homing state → Someone else started homing (physical buttons?)
        if (state_ != State::Homing)
        {
          ESP_LOGW(TAG_ENGINE, "Unexpected homing: Hardware homing but engine state is %s",
                   state_to_string(state_));
          ESP_LOGW(TAG_ENGINE, "  Possible cause: Manual homing via physical buttons");
          // Sync engine state to hardware reality
          transition_to(State::Homing);
        }
        break;

      case CommandDecoder::MotorStatus::CALIBRATING:
        // Hardware reports calibration - validate against Engine expectations
        // If Engine is NOT in Calibrating state → Manual calibration started
        if (state_ != State::Calibrating)
        {
          ESP_LOGW(TAG_ENGINE, "Unexpected calibration: Hardware calibrating but engine state is %s",
                   state_to_string(state_));
          // Sync engine state to hardware reality
          transition_to(State::Calibrating);
        }
        break;
      }
    }

    void StepperEngine::process_protection_update(uint8_t protected_status)
    {
      bool old_protection = protection_triggered_;
      protection_triggered_ = (protected_status != 0);

      // Transition to Error state if protection triggered
      if (protection_triggered_ && !old_protection)
      {
        ESP_LOGE(TAG_ENGINE, "Protection triggered! Status=0x%02X", protected_status);
        handle_error("Locked-rotor protection triggered");

        if (protection_callback_)
        {
          protection_callback_();
        }
      }
    }

    bool StepperEngine::is_target_reached()
    {
      // Check if current position is within tolerance of target
      float tolerance = 5.0f; // steps
      float delta = std::abs(parent_->current_pos_.get_steps() - parent_->target_pos_.get_steps());
      return delta <= tolerance;
    }

    void StepperEngine::handle_error(const char *error_message)
    {
      ESP_LOGE(TAG_ENGINE, "Error: %s", error_message);
      transition_to(State::Error);
    }

  } // namespace servoxxd
} // namespace esphome
