#include "servoxxd_stepper_engine.h"
#include "servoxxd.h"
#include "servoxxd_command_decoder.h"
#include "servoxxd_commands.h"
#include "servoxxd_command_factory.h"
#include "servoxxd_transport.h"
#include <cmath>
#include <cstring>

namespace esphome {
namespace servoxxd {

static const char *TAG_ENGINE = "servoxxd.stepper_engine";

// ============================================================================
// Constructor / Destructor
// ============================================================================

StepperEngine::StepperEngine(ServoXxd *parent, ITransport *transport, uint32_t command_timeout_ms)
    : parent_(parent),
      queue_(nullptr),
      state_(State::SettingUp),
      emergency_flag_(false),
      current_speed_(0.0f, SpeedUnit::RPM, parent),
      protection_triggered_(false),
      state_enter_time_(0),
      disable_pending_(false) {
  // Initialize state entry time to current time for proper timeout tracking
  state_enter_time_ = millis();

  // Create CommandQueue (Layer 3) if transport provided
  if (transport != nullptr) {
    queue_ = new CommandQueue(transport, command_timeout_ms);
    ESP_LOGD(TAG_ENGINE, "StepperEngine initialized with transport: timeout=%ums", command_timeout_ms);
  } else {
    ESP_LOGD(TAG_ENGINE, "StepperEngine initialized WITHOUT transport (stub mode)");
  }
}

StepperEngine::~StepperEngine() {
  if (queue_) {
    delete queue_;
    queue_ = nullptr;
  }
}

// ============================================================================
// Hardware Polling Methods
// ============================================================================

void StepperEngine::poll_motor_speed() {
  // Enqueue read command for motor speed (Commandtype 0x32 READ_CURRENT_SPEED)
  // Expected response: speed_rpm (int16_t) = 2 bytes

  queue_->enqueue(
      CommandFactory::read_current_speed(),
      [this](bool success, const Command &cmd) {
        if (success) {
          Speed speed = CommandDecoder::read_current_speed(cmd, parent_);
          process_speed_update(speed);
        }
      },
      Priority::BACKGROUND);
}

void StepperEngine::poll_motor_status() {
  // Enqueue read command for motor status (Commandtype 0xF1 READ_MOTOR_STATUS)
  // Expected response: status (uint8_t: 0=fail, 1=stop, 2=speed_up, 3=speed_down, 4=full_speed, 5=homing,
  // 6=calibrating)
  queue_->enqueue(
      CommandFactory::read_motor_status(),
      [this](bool success, const Command &cmd) {
        if (!success) {
          ESP_LOGW(TAG_ENGINE, "Failed to read motor status");
          return;
        }

        CommandDecoder::MotorStatus status = CommandDecoder::read_motor_status(cmd);
        process_motor_status_update(status);
      },
      Priority::BACKGROUND);
}

void StepperEngine::poll_protection_status() {
  // Enqueue read command for protection status (Commandtype 0x3E READ_PROTECTION_STATUS)
  // Expected response: protection (uint8_t, 0 = OK, 1 = protected) = 2 bytes (1 register)
  queue_->enqueue(
      CommandFactory::read_protection_status(),
      [this](bool success, const Command &cmd) {
        if (success) {
          auto ps = CommandDecoder::read_protection_status(cmd);
          process_protection_update(ps.protected_state ? 1 : 0);
        }
      },
      Priority::BACKGROUND);
}

// ============================================================================
// Motor Setup
// ============================================================================

void StepperEngine::setup_motor() {
  // Explicitly transition to SettingUp state
  transition_to(State::SettingUp);

  if (!queue_) {
    parent_->status_set_error("CommandQueue not initialized");
    parent_->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG_ENGINE, "Enqueuing motor initialization sequence...");

  // 0. Restart motor to ensure clean state (Commandtype 0x41 RESTART)
  // Motor needs ~3-4s to reboot before accepting configuration commands
  queue_->enqueue(
      CommandFactory::restart(),
      [](bool success, const Command &) {
        if (success) {
          ESP_LOGD(TAG_ENGINE, "✓ Motor restart initiated, waiting 4000ms...");
        } else {
          ESP_LOGW(TAG_ENGINE, "✗ Failed to restart motor");
        }
      },
      Priority::NORMAL,
      4000);  // Wait 4 seconds after restart

  // 1. Read all current configuration from motor (after restart)
  // This allows us to verify the motor's current state before applying new settings
  queue_->enqueue(CommandFactory::read_all_config(), [this](bool success, const Command &cmd) {
    if (!success) {
      ESP_LOGE(TAG_ENGINE, "✗ Failed to read motor configuration - aborting setup!");
      transition_to(State::Error);
      parent_->status_set_error("Failed to read motor configuration");
      parent_->mark_failed();
      // Clear all pending commands to abort setup sequence
      if (queue_) {
        queue_->clear();
      }
      return;
    }

    ESP_LOGD(TAG_ENGINE, "✓ Current motor configuration read (%zu bytes)", cmd.response.size());

    // DEBUG: Output raw response bytes as hex dump
    if (!cmd.response.empty()) {
      std::string hex_dump;
      for (size_t i = 0; i < cmd.response.size(); ++i) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X ", cmd.response[i]);
        hex_dump += buf;
        if ((i + 1) % 16 == 0 && i + 1 < cmd.response.size()) {
          hex_dump += "\n                                ";
        }
      }
      ESP_LOGD(TAG_ENGINE, "  Raw config bytes: %s", hex_dump.c_str());
    }

    // Decode and log individual configuration values
    auto config = CommandDecoder::read_all_config(cmd, parent_);

    // Log control settings
    ESP_LOGD(TAG_ENGINE, "  Control mode: %s", control_mode_to_string(config.mode));
    ESP_LOGD(TAG_ENGINE, "  Working current: %u mA", config.working_current_ma);
    ESP_LOGD(TAG_ENGINE, "  Holding current: %s", holding_current_percent_to_string(config.holding_current_percent));
    ESP_LOGD(TAG_ENGINE, "  Microstepping: 1/%u", config.subdivision);

    // Log pin settings
    ESP_LOGD(TAG_ENGINE, "  EN pin active: %s", en_pin_active_to_string(config.en_pin_active));
    ESP_LOGD(TAG_ENGINE, "  Shaft direction: %s", direction_to_string(config.direction));

    // Log display and protection
    ESP_LOGD(TAG_ENGINE, "  Auto screen off: %s", screen_mode_to_string(config.screen_mode));
    ESP_LOGD(TAG_ENGINE, "  Protection: %s", protection_mode_to_string(config.protection));
    ESP_LOGD(TAG_ENGINE, "  Keys: %s", keypad_lock_to_string(config.keypad_lock));

    // Generate list of command types needed to update configuration
    ESP_LOGCONFIG(TAG_ENGINE, "Checking which configuration values need updates...");
    // Get desired config from parent
    ConfigData desired_config = parent_->config_;
    // Get list of commands that need to be executed
    std::vector<Commandtype> update_commands = config.get_update_command_types(desired_config);

    if (update_commands.empty()) {
      ESP_LOGD(TAG_ENGINE, "  No configuration updates needed - all values match");
    } else {
      ESP_LOGD(TAG_ENGINE, "  %zu configuration value(s) need updating", update_commands.size());

      // Process each command type in the optimal order (already sorted by get_update_command_types)
      for (const auto &cmd_type : update_commands) {
        switch (cmd_type) {
          case Commandtype::SET_WORK_MODE: {
            ESP_LOGD(TAG_ENGINE, "  Control mode: %s → %s", control_mode_to_string(config.mode),
                     control_mode_to_string(desired_config.mode));

            ControlMode desired_mode = desired_config.mode;

            queue_->enqueue(CommandFactory::set_control_mode(desired_config.mode), [this, desired_mode](
                                                                                       bool success, const Command &) {
              if (state_ == State::Error) {
                return;  // Setup already aborted
              }
              if (success) {
                ESP_LOGD(TAG_ENGINE, "✓ Control mode updated to %s", control_mode_to_string(desired_mode));
              } else {
                ESP_LOGE(TAG_ENGINE, "✗ Failed to update control mode");
                transition_to(State::Error);
                parent_->status_set_error("Failed to update control mode");
                parent_->mark_failed();
                if (queue_) {
                  queue_->clear();
                }
              }
            });
            break;
          }

          case Commandtype::SET_HOLDING_CURRENT_PERCENT: {
            ESP_LOGD(TAG_ENGINE, "  Holding current: %s → %s",
                     holding_current_percent_to_string(config.holding_current_percent),
                     holding_current_percent_to_string(desired_config.holding_current_percent));

            HoldingCurrentPercent desired_holding_current = desired_config.holding_current_percent;

            queue_->enqueue(CommandFactory::set_holding_current_percent(desired_holding_current),
                            [this, desired_holding_current](bool success, const Command &) {
                              if (state_ == State::Error) {
                                return;  // Setup already aborted
                              }
                              if (success) {
                                ESP_LOGD(TAG_ENGINE, "✓ Holding current updated to %s",
                                         holding_current_percent_to_string(desired_holding_current));
                              } else {
                                ESP_LOGE(TAG_ENGINE, "✗ Failed to update holding current");
                                transition_to(State::Error);
                                parent_->status_set_error("Failed to update holding current");
                                parent_->mark_failed();
                                if (queue_) {
                                  queue_->clear();
                                }
                              }
                            });
            break;
          }

          case Commandtype::SET_WORKING_CURRENT_RUNTIME: {
            ESP_LOGD(TAG_ENGINE, "  Working current: %u mA → %u mA", config.working_current_ma,
                     desired_config.working_current_ma);

            uint16_t desired_current_ma = desired_config.working_current_ma;

            queue_->enqueue(CommandFactory::set_working_current(desired_current_ma),
                            [this, desired_current_ma](bool success, const Command &) {
                              if (state_ == State::Error) {
                                return;  // Setup already aborted
                              }
                              if (success) {
                                ESP_LOGD(TAG_ENGINE, "✓ Working current updated to %u mA", desired_current_ma);
                              } else {
                                ESP_LOGE(TAG_ENGINE, "✗ Failed to update working current");
                                transition_to(State::Error);
                                parent_->status_set_error("Failed to update working current");
                                parent_->mark_failed();
                                if (queue_) {
                                  queue_->clear();
                                }
                              }
                            });
            break;
          }

          case Commandtype::SET_SUBDIVISION: {
            ESP_LOGD(TAG_ENGINE, "  Microstepping: %u → %u", config.subdivision, desired_config.subdivision);

            uint8_t desired_microstepping = desired_config.subdivision;

            queue_->enqueue(CommandFactory::set_subdivision(desired_microstepping), [this, desired_microstepping](
                                                                                        bool success, const Command &) {
              if (state_ == State::Error) {
                return;  // Setup already aborted
              }
              if (success) {
                ESP_LOGD(TAG_ENGINE, "✓ Microstepping updated to %u", desired_microstepping);
              } else {
                // Motor rejected SET_SUBDIVISION command
                // This is expected in vFOC modes (hardware limitation per manual)
                ESP_LOGE(TAG_ENGINE, "✗ Failed to update microstepping to %u - Hardware rejected command",
                         desired_microstepping);
                ESP_LOGE(TAG_ENGINE, "  Possible causes: Motor in vFOC mode (SET_SUBDIVISION only valid for "
                                     "OPEN/CLOSE modes)");
                ESP_LOGE(TAG_ENGINE, "  This should have been caught by YAML validation - please report this bug!");
                transition_to(State::Error);
                parent_->status_set_error("Failed to update microstepping");
                parent_->mark_failed();
                if (queue_) {
                  queue_->clear();
                }
              }
            });
            break;
          }

          case Commandtype::SET_EN_PIN_ACTIVE: {
            ESP_LOGD(TAG_ENGINE, "  EN pin active: %s → %s", en_pin_active_to_string(config.en_pin_active),
                     en_pin_active_to_string(desired_config.en_pin_active));

            EnPinActive desired_en_pin = desired_config.en_pin_active;

            queue_->enqueue(CommandFactory::set_en_pin_active(desired_en_pin), [this, desired_en_pin](bool success,
                                                                                                      const Command &) {
              if (state_ == State::Error) {
                return;  // Setup already aborted
              }
              if (success) {
                ESP_LOGD(TAG_ENGINE, "✓ EN pin active updated to %s", en_pin_active_to_string(desired_en_pin));
              } else {
                ESP_LOGE(TAG_ENGINE, "✗ Failed to update EN pin active level");
                transition_to(State::Error);
                parent_->status_set_error("Failed to update EN pin active level");
                parent_->mark_failed();
                if (queue_) {
                  queue_->clear();
                }
              }
            });
            break;
          }

          case Commandtype::SET_DIR_MOTOR_ROTATION: {
            ESP_LOGD(TAG_ENGINE, "  Direction: %s → %s", direction_to_string(config.direction),
                     direction_to_string(desired_config.direction));

            Direction desired_direction = desired_config.direction;

            queue_->enqueue(CommandFactory::set_dir_motor_rotation(desired_direction),
                            [this, desired_direction](bool success, const Command &) {
                              if (state_ == State::Error) {
                                return;  // Setup already aborted
                              }
                              if (success) {
                                ESP_LOGD(TAG_ENGINE, "✓ Direction updated to %s",
                                         direction_to_string(desired_direction));
                              } else {
                                ESP_LOGE(TAG_ENGINE, "✗ Failed to update direction");
                                transition_to(State::Error);
                                parent_->status_set_error("Failed to update direction");
                                parent_->mark_failed();
                                if (queue_) {
                                  queue_->clear();
                                }
                              }
                            });
            break;
          }

          case Commandtype::SET_AUTO_SCREEN_OFF: {
            ESP_LOGD(TAG_ENGINE, "  Auto screen off: %s → %s", screen_mode_to_string(config.screen_mode),
                     screen_mode_to_string(desired_config.screen_mode));

            ScreenMode desired_screen_mode = desired_config.screen_mode;

            queue_->enqueue(CommandFactory::set_auto_screen_off(desired_screen_mode),
                            [this, desired_screen_mode](bool success, const Command &) {
                              if (state_ == State::Error) {
                                return;  // Setup already aborted
                              }
                              if (success) {
                                ESP_LOGD(TAG_ENGINE, "✓ Auto screen off updated to %s",
                                         screen_mode_to_string(desired_screen_mode));
                              } else {
                                ESP_LOGE(TAG_ENGINE, "✗ Failed to update auto screen off");
                                transition_to(State::Error);
                                parent_->status_set_error("Failed to update auto screen off");
                                parent_->mark_failed();
                                if (queue_) {
                                  queue_->clear();
                                }
                              }
                            });
            break;
          }

          case Commandtype::SET_PROTECT_ENABLE: {
            ESP_LOGD(TAG_ENGINE, "  Protection mode: %s → %s", protection_mode_to_string(config.protection),
                     protection_mode_to_string(desired_config.protection));

            ProtectionMode desired_protection = desired_config.protection;

            queue_->enqueue(CommandFactory::set_protect_enable(protection_mode_to_bool(desired_protection)),
                            [this, desired_protection](bool success, const Command &) {
                              if (state_ == State::Error) {
                                return;  // Setup already aborted
                              }
                              if (success) {
                                ESP_LOGD(TAG_ENGINE, "✓ Protection mode updated to %s",
                                         protection_mode_to_string(desired_protection));
                              } else {
                                ESP_LOGE(TAG_ENGINE, "✗ Failed to update protection mode");
                                transition_to(State::Error);
                                parent_->status_set_error("Failed to update protection mode");
                                parent_->mark_failed();
                                if (queue_) {
                                  queue_->clear();
                                }
                              }
                            });
            break;
          }

          case Commandtype::SET_LOCK_KEYS: {
            ESP_LOGD(TAG_ENGINE, "  Key lock: %s → %s", keypad_lock_to_string(config.keypad_lock),
                     keypad_lock_to_string(desired_config.keypad_lock));

            KeypadLock desired_keypad_lock = desired_config.keypad_lock;

            queue_->enqueue(CommandFactory::set_lock_keys(desired_keypad_lock), [this, desired_keypad_lock](
                                                                                    bool success, const Command &) {
              if (state_ == State::Error) {
                return;  // Setup already aborted
              }
              if (success) {
                ESP_LOGD(TAG_ENGINE, "✓ Keys updated to %s", keypad_lock_to_string(desired_keypad_lock));
              } else {
                ESP_LOGE(TAG_ENGINE, "✗ Failed to update key lock");
                transition_to(State::Error);
                parent_->status_set_error("Failed to update key lock");
                parent_->mark_failed();
                if (queue_) {
                  queue_->clear();
                }
              }
            });
            break;
          }

          case Commandtype::SET_EN_TRIGGER_CONFIG: {
            ESP_LOGD(TAG_ENGINE, "  Setting EN trigger config to safe defaults (always set)");

            queue_->enqueue(CommandFactory::set_en_trigger_config(), [this](bool success, const Command &) {
              if (state_ == State::Error) {
                return;  // Setup already aborted
              }
              if (success) {
                ESP_LOGD(TAG_ENGINE, "✓ EN trigger config set to safe defaults");
              } else {
                ESP_LOGE(TAG_ENGINE, "✗ Failed to set EN trigger configuration");
                transition_to(State::Error);
                parent_->status_set_error("Failed to set EN trigger configuration");
                parent_->mark_failed();
                if (queue_) {
                  queue_->clear();
                }
              }
            });
            break;
          }

          case Commandtype::SET_HOMING_PARAMETERS: {
            // Only configure if ENDSTOP mode is enabled
            auto &homing = parent_->homing_;
            if (homing.mode != HomingMode::ENDSTOP) {
              break;  // Skip if not ENDSTOP mode
            }

            ESP_LOGD(TAG_ENGINE, "  Homing parameters: mode=ENDSTOP, trigger=%s, dir=%s, speed=%.1f RPM",
                     homing.endstop_trigger == EndstopTrigger::TRIGGER_LOW ? "LOW" : "HIGH",
                     homing.direction == HomingDirection::CW ? "CW" : "CCW", homing.speed.rpm());

            // Convert HomingDirection to Direction
            Direction dir = (homing.direction == HomingDirection::CW) ? Direction::CW : Direction::CCW;

            // Enable EndLimit for ENDSTOP homing mode (required for GO_HOME to work)
            queue_->enqueue(CommandFactory::set_homing_parameters(homing.endstop_trigger, dir, homing.speed, false),
                            [this](bool success, const Command &) {
                              if (state_ == State::Error) {
                                return;  // Setup already aborted
                              }
                              if (success) {
                                ESP_LOGD(TAG_ENGINE, "✓ Homing parameters configured (EndLimit enabled)");
                              } else {
                                ESP_LOGE(TAG_ENGINE, "✗ Failed to configure homing parameters");
                                transition_to(State::Error);
                                parent_->status_set_error("Failed to configure homing parameters");
                                parent_->mark_failed();
                                if (queue_) {
                                  queue_->clear();
                                }
                              }
                            });
            break;
          }

          case Commandtype::SET_NOLIMIT_HOMING_PARAMS: {
            // TODO: Implement SENSORLESS homing setup
            ESP_LOGW(TAG_ENGINE, "  SENSORLESS homing setup not yet implemented");
            break;
          }

          case Commandtype::SET_LIMIT_PORT_REMAP:
          case Commandtype::SET_ZERO_MODE:
            // TODO: Implement handlers for these command types
            ESP_LOGW(TAG_ENGINE, "  Command type 0x%04X not yet implemented", static_cast<uint16_t>(cmd_type));
            break;

          default:
            ESP_LOGW(TAG_ENGINE, "  Unknown command type: 0x%04X", static_cast<uint16_t>(cmd_type));
            break;
        }
      }
    }

    // Enqueue setup completion marker AFTER all config updates
    // Queue guarantees FIFO order, so this runs after all config commands
    queue_->enqueue(
        CommandFactory::read_motor_status(),
        [this](bool success, const Command &) {
          // If already in Error state, setup was aborted by earlier failure
          if (state_ == State::Error) {
            return;  // Don't overwrite original error message
          }

          if (!success) {
            ESP_LOGE(TAG_ENGINE, "✗ Motor setup failed - final status check unsuccessful");
            transition_to(State::Error);
            parent_->setup_state_ = SetupState::FAILED;
            parent_->status_set_error("Motor setup failed");
            parent_->mark_failed();
            return;
          }

          // All setup commands completed successfully
          ESP_LOGI(TAG_ENGINE, "✓ Motor setup completed successfully");

          // Signal ESPHome that setup is complete
          parent_->setup_state_ = SetupState::COMPLETED;

          // Start hardware polling NOW (not in setup())
          parent_->set_interval("hardware_poll", 200, [this]() {
            if (parent_->engine_ != nullptr) {
              parent_->engine_->poll_hardware();
            }
          });

          ESP_LOGD(TAG_ENGINE, "  Hardware polling started (200ms interval)");
          ESP_LOGCONFIG("servoxxd", "ServoXxd Modbus setup complete");

          // Transition to Idle (ready for commands)
          if (state_ == State::SettingUp) {
            transition_to(State::Idle);
          }

          // Execute homing at startup if configured
          if (parent_->homing_.at_startup && parent_->homing_.mode != HomingMode::NO_HOMING) {
            ESP_LOGI(TAG_ENGINE, "Executing homing at startup (mode=%d)", static_cast<int>(parent_->homing_.mode));
            // Delay homing slightly to ensure motor is fully ready
            parent_->set_timeout("homing_at_startup", 5000, [this]() { this->home(); });
          }
        },
        Priority::NORMAL);
  });

  ESP_LOGCONFIG(TAG_ENGINE, "Setup: Motor initialization sequence enqueued");
}

// ============================================================================
// Main Update Loop
// ============================================================================

void StepperEngine::update() {
  // 1. Update CommandQueue (process timeouts, execute next command)
  if (queue_) {
    queue_->update();
  }

  // 2. Check state-specific timeouts
  check_state_timeouts();

  // 3. Process buffered commands
  if (disable_pending_ && state_ == State::Idle) {
    disable_pending_ = false;
    disable();  // Execute buffered disable
  }
}

// ============================================================================
// Hardware Polling
// ============================================================================

void StepperEngine::poll_hardware() {
  if (parent_->setup_state_ == SetupState::COMPLETED) {
    // Poll all status values in sequence
    poll_encoder_position();
    poll_motor_speed();
    poll_motor_status();  // Handles homing completion: HOMING(5) → STOP(1)
    // TODO: Register 0x3E might not exist in hardware - investigate
    // poll_protection_status();
  }
}

// ============================================================================
// Movement Commands
// ============================================================================

void StepperEngine::move_to(const Position &target, std::optional<Speed> speed, std::optional<Acceleration> accel) {
  // Use default values if not provided
  Speed speed_units = speed.has_value() ? speed.value() : parent_->get_default_speed();
  Acceleration accel_units = accel.has_value() ? accel.value() : parent_->get_default_acceleration();

  // Validation: only allowed in Idle state (Position Mode)
  if (!validate_command(__func__, {State::Idle, State::Moving, State::Stopping})) {
    return;
  }
  ESP_LOGD(TAG_ENGINE, "move_to(): target=%lld steps, speed=%.2f RPM, accel=%.2f RPM/s",
           static_cast<long long>(target.get_steps()), speed_units.rpm(), accel_units.get_rpm_per_sec());

  // Update target position for target-reached detection
  parent_->set_target_pos(target);

  // Send move command to hardware using Mode 4 (absolute by encoder ticks)
  // Mode 4 uses absolute encoder position - target is sent directly to hardware
  queue_->enqueue(CommandFactory::move_position_mode_4(speed_units, accel_units, target),
                  [this](bool success, const Command &) {
                    if (!success) {
                      ESP_LOGW(TAG_ENGINE, "move_to: Failed to send move command to hardware");
                      return;
                    }

                    // Only transition to Moving if not already in motion
                    // During Moving/Stopping: just update target (override behavior)
                    if (state_ == State::Moving) {
                      return;
                    }
                    transition_to(State::Moving);
                  });
}

void StepperEngine::stop(std::optional<Acceleration> decel) {
  // Validation: allowed in Moving, Running, Homing, Calibrating, Stopping states
  if (state_ == State::Idle) {
    ESP_LOGD(TAG_ENGINE, "stop(): Already stopped, no-op");
    return;
  }

  if (!validate_command(__func__,
                        {State::Moving, State::Running, State::Homing, State::Calibrating, State::Stopping})) {
    return;
  }

  ESP_LOGD(TAG_ENGINE, "stop(): decel=%.2f RPM/s", decel.has_value() ? decel->get_rpm_per_sec() : 0.0f);

  // Send stop command via queue (Commandtype 0xFE STOP_POSITION_MODE_2)
  Acceleration decel_units = decel.has_value() ? decel.value() : parent_->get_default_acceleration();
  queue_->enqueue(CommandFactory::stop_position_mode_2(decel_units), nullptr);

  transition_to(State::Stopping);
}

void StepperEngine::emergency_stop() {
  ESP_LOGW(TAG_ENGINE, "emergency_stop(): Immediate halt, clearing queue");

  emergency_flag_ = true;

  // Clear command queue and send emergency stop
  if (queue_) {
    queue_->clear();  // Clear all pending commands

    // Send emergency stop command to hardware (Commandtype 0xF7 EMERGENCY_STOP)
    queue_->enqueue(CommandFactory::emergency_stop(), nullptr, Priority::CRITICAL);
  }

  transition_to(State::Error);
}

void StepperEngine::home() {
  // Get homing configuration from parent
  auto &homing = parent_->homing_;

  // Check if homing is configured
  if (homing.mode == HomingMode::NO_HOMING) {
    ESP_LOGW(TAG_ENGINE, "home(): No homing configured - action ignored");
    return;
  }

  ESP_LOGD(TAG_ENGINE, "home(): Starting homing sequence (mode=%d)", static_cast<int>(homing.mode));

  // Note: Homing parameters are already configured in setup_motor()
  // This method only triggers the homing sequence

  switch (homing.mode) {
    case HomingMode::VIRTUAL: {
      // TODO: Implement VIRTUAL homing - Move to position 0 using move_to()
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

    case HomingMode::ENDSTOP: {
      // ENDSTOP homing: Trigger homing sequence (parameters already set in setup)
      ESP_LOGD(TAG_ENGINE, "ENDSTOP homing: Starting sequence (speed=%.1f RPM)", homing.speed.rpm());

      queue_->enqueue(CommandFactory::go_home(), [this](bool success, const Command &) {
        if (success) {
          ESP_LOGD(TAG_ENGINE, "✓ ENDSTOP homing command sent");
          // Transition to Homing state - poll_homing_status() will monitor completion
          transition_to(State::Homing);
        } else {
          transition_to(State::Error);
          ESP_LOGW(TAG_ENGINE, "✗ Failed to start ENDSTOP homing");
        }
      });
      break;
    }

    case HomingMode::SENSORLESS: {
      // TODO: Implement SENSORLESS homing - Use stall detection with no-limit parameters
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

  // Note: State transition to Homing happens in the callback for ENDSTOP mode
  // For VIRTUAL/SENSORLESS modes (when implemented), they handle transitions themselves
}

void StepperEngine::run_continuous(std::optional<Speed> speed, std::optional<Acceleration> accel) {
  // Validation: allowed in Idle or Running states (Speed Mode)
  if (!validate_command(__func__, {State::Idle, State::Running})) {
    return;
  }

  // Use default values if not provided (requires parent defaults)
  Speed speed_obj = speed.has_value() ? speed.value() : parent_->get_default_speed();
  Acceleration accel_obj = accel.has_value() ? accel.value() : parent_->get_default_acceleration();

  ESP_LOGD(TAG_ENGINE, "run_continuous(): speed=%.2f RPM, accel=%.2f RPM/s", speed_obj.rpm(),
           accel_obj.get_rpm_per_sec());

  // Send speed command via queue (Commandtype 0xF6 MOVE_SPEED_MODE)
  queue_->enqueue(CommandFactory::move_speed_mode(speed_obj, accel_obj), nullptr);

  transition_to(State::Running);
}

// ============================================================================
// Configuration Commands
// ============================================================================

void StepperEngine::enable() {
  // Allow enable from Disabled, Idle, or Error states
  if (!validate_command(__func__, {State::Disabled, State::Idle})) {
    return;
  }

  // If already in Idle state, motor is already enabled
  if (state_ == State::Idle) {
    ESP_LOGD(TAG_ENGINE, "enable(): Motor already enabled (state=Idle)");
    return;
  }

  ESP_LOGD(TAG_ENGINE, "enable(): Enabling motor");

  // Send enable command via queue (Commandtype 0xF3 ENABLE_MOTOR)
  queue_->enqueue(CommandFactory::enable_motor(true), nullptr);

  transition_to(State::Idle);
}

void StepperEngine::disable() {
  // Validate allowed states (explicitly reject Homing and Calibrating)
  if (!validate_command(__func__,
                        {State::Idle, State::Disabled, State::Error, State::Moving, State::Running, State::Stopping})) {
    return;
  }

  // During motion: stop first, then disable will be triggered after stop completes
  if (state_ == State::Moving || state_ == State::Running || state_ == State::Stopping) {
    ESP_LOGI(TAG_ENGINE, "disable(): Motor in motion - stopping motor first, disable will follow after stop completes");
    disable_pending_ = true;
    stop();  // Stop first, disable() will be called again from update() when Idle is reached
    return;
  }

  // Already disabled - no-op
  if (state_ == State::Disabled) {
    ESP_LOGD(TAG_ENGINE, "disable(): Motor already disabled");
    return;
  }

  ESP_LOGD(TAG_ENGINE, "disable(): Disabling motor");

  // Send disable command via queue (Commandtype 0xF3 ENABLE_MOTOR with false)
  queue_->enqueue(CommandFactory::enable_motor(false), nullptr);

  transition_to(State::Disabled);
}

void StepperEngine::release_protection() {
  if (!validate_command(__func__, {State::Error, State::Idle, State::Disabled})) {
    return;
  }

  ESP_LOGD(TAG_ENGINE, "release_protection(): Attempting to clear error state");

  // Always clear internal flags
  protection_triggered_ = false;
  emergency_flag_ = false;

  // Send release protection command via queue (Commandtype 0x3D RELEASE_PROTECTION)
  queue_->enqueue(CommandFactory::release_protection(), [](bool success, const Command &) {
    if (success) {
      ESP_LOGD(TAG_ENGINE, "✓ Release protection command sent");
    } else {
      ESP_LOGW(TAG_ENGINE, "✗ Failed to send release protection");
    }
  });

  // Transition from Error to Idle to allow re-enabling
  if (state_ == State::Error) {
    ESP_LOGD(TAG_ENGINE, "  Transitioning from Error to Idle");
    transition_to(State::Idle);
  }
}

void StepperEngine::restart() {
  ESP_LOGD(TAG_ENGINE, "restart(): Full motor restart");

  // Clear all errors
  protection_triggered_ = false;
  emergency_flag_ = false;

  // Clear queue
  if (queue_) {
    queue_->clear();
  }

  // Send restart command to hardware (Commandtype 0x41 RESTART with value 0x0001)
  // Motor needs 3-4 seconds to fully restart - use queue delay mechanism
  queue_->enqueue(CommandFactory::restart(), nullptr, Priority::NORMAL, 4000);

  ESP_LOGD(TAG_ENGINE, "  Motor will restart, next command delayed 4000ms");

  transition_to(State::Idle);
}

void StepperEngine::calibrate() {
  if (!validate_command(__func__, {State::Idle, State::Disabled})) {
    return;
  }

  ESP_LOGD(TAG_ENGINE, "calibrate(): Starting encoder calibration");

  // Send calibrate encoder command via queue (Commandtype 0x80 CALIBRATE_ENCODER)
  queue_->enqueue(CommandFactory::calibrate_encoder(), [this](bool success, const Command &) {
    if (success) {
      ESP_LOGD(TAG_ENGINE, "✓ Calibration command sent");
    } else {
      ESP_LOGW(TAG_ENGINE, "✗ Failed to start calibration");
      transition_to(State::Error);
    }
  });

  transition_to(State::Calibrating);
}

void StepperEngine::key_lock() {
  ESP_LOGD(TAG_ENGINE, "key_lock(): Locking physical keys");

  // Send key lock command via queue (Commandtype 0x8F SET_LOCK_KEYS)
  queue_->enqueue(CommandFactory::set_lock_keys(KeypadLock::LOCKED), nullptr);
}

void StepperEngine::key_unlock() {
  ESP_LOGD(TAG_ENGINE, "key_unlock(): Unlocking physical keys");

  // Send key unlock command via queue (Commandtype 0x8F SET_LOCK_KEYS)
  queue_->enqueue(CommandFactory::set_lock_keys(KeypadLock::UNLOCKED), nullptr);
}

void StepperEngine::set_zero() {
  if (!validate_command(__func__, {State::Idle})) {
    return;
  }

  ESP_LOGD(TAG_ENGINE, "set_zero(): Sending SET_ZERO command to hardware");

  // Update position tracking only after hardware confirms
  queue_->enqueue(CommandFactory::set_zero(), [this](bool success, const Command &) {
    if (!success) {
      ESP_LOGW(TAG_ENGINE, "set_zero: Hardware command failed");
      return;
    }

    // Hardware confirmed - reset position tracking to zero
    parent_->position_offset_ = Position(0.0f, PositionUnit::STEPS, parent_);
    parent_->set_current_pos(Position(0.0f, PositionUnit::STEPS, parent_));

    ESP_LOGI(TAG_ENGINE, "set_zero: Hardware confirmed, encoder and offset reset to zero");
  });
}

// ============================================================================
// Status Queries
// ============================================================================

bool StepperEngine::is_moving() const {
  return state_ == State::Moving || state_ == State::Running || state_ == State::Homing ||
         state_ == State::Calibrating || state_ == State::Stopping;
}

const char *StepperEngine::state_to_string(State state) {
  switch (state) {
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
// Transport Access
// ============================================================================

ITransport *StepperEngine::get_transport() const { return queue_ ? queue_->get_transport() : nullptr; }

// ============================================================================
// Private Methods - State Machine
// ============================================================================

void StepperEngine::transition_to(State new_state) {
  if (state_ == new_state) {
    return;  // No change
  }

  [[maybe_unused]] State old_state = state_;
  state_ = new_state;
  state_enter_time_ = millis();  // Track state entry time for timeout monitoring

  ESP_LOGD(TAG_ENGINE, "State transition: %s → %s", state_to_string(old_state), state_to_string(new_state));

  // State-specific initialization
  switch (new_state) {
    case State::Moving:
      // Start polling position more frequently (optional)
      break;

    case State::Idle:
      // Reset target position
      parent_->target_pos_ = parent_->current_pos_;
      break;

    case State::Error:
      // Stop all motion immediately
      if (queue_) {
        queue_->clear();
      }
      break;

    default:
      break;
  }
}

bool StepperEngine::validate_command(const char *func_name, std::initializer_list<State> allowed_states) {
  // Special handling for SettingUp state: Only allow critical commands
  if (state_ == State::SettingUp) {
    // During setup, only allow emergency stop, normal stop, and protection release
    bool is_critical = (strcmp(func_name, "emergency_stop") == 0 || strcmp(func_name, "stop") == 0 ||
                        strcmp(func_name, "release_protection") == 0);

    if (!is_critical) {
      ESP_LOGW(TAG_ENGINE, "%s(): Rejected during setup - motor still initializing", func_name);
      return false;
    }
    // Critical command during SettingUp - allow it
    return true;
  }

  for (State allowed : allowed_states) {
    if (state_ == allowed) {
      return true;  // Command allowed
    }
  }

  // Command not allowed in current state
  ESP_LOGW(TAG_ENGINE, "%s(): Rejected (state=%s)", func_name, state_to_string(state_));
  return false;
}

void StepperEngine::check_state_timeouts() {
  // State-specific timeout monitoring
  uint32_t now = millis();
  uint32_t state_duration = now - state_enter_time_;

  switch (state_) {
    case State::SettingUp:
      // Maximum setup duration: 30 seconds
      if (state_duration > 30000) {
        ESP_LOGE(TAG_ENGINE, "Setup timeout after %u ms", state_duration);
        handle_error("Setup timeout - motor not responding");
      }
      break;

    case State::Homing:
      // Maximum homing duration: 600 seconds
      if (state_duration > 600000) {
        ESP_LOGE(TAG_ENGINE, "Homing timeout after %u ms", state_duration);
        handle_error("Homing timeout");
      }
      break;

    case State::Calibrating:
      // Maximum calibration duration: 120 seconds
      if (state_duration > 120000) {
        ESP_LOGE(TAG_ENGINE, "Calibration timeout after %u ms", state_duration);
        handle_error("Calibration timeout");
      }
      break;

    case State::Stopping:
      // Maximum stop duration: 50 seconds
      if (state_duration > 50000) {
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

void StepperEngine::poll_encoder_position(std::function<void(const Position &)> callback) {
  // Enqueue read command for encoder position (Commandtype 0x30 READ_ENCODER_CARRY)
  // Expected response: carry (int32_t) + value (uint16_t) = 6 bytes
  queue_->enqueue(
      CommandFactory::read_encoder_carry(),
      [this, callback](bool success, const Command &cmd) {
        if (success) {
          auto position = CommandDecoder::read_encoder_carry(cmd, parent_);
          process_encoder_update(position);
          parent_->set_current_pos(position);
        }
      },
      Priority::BACKGROUND);
}

// ============================================================================
// Private Methods - Event Processing
// ============================================================================

void StepperEngine::process_encoder_update(const Position &position) {
  Position old_position = parent_->current_pos_;
  parent_->set_current_pos(position);

  // Check if target reached (in Moving state)
  if (state_ == State::Moving && is_target_reached()) {
    ESP_LOGD(TAG_ENGINE, "Target position reached");
    transition_to(State::Idle);
  }

  // Check if homing completed (in Homing state)
  if (state_ == State::Homing && parent_->current_position == 0) {
    ESP_LOGI(TAG_ENGINE, "✓ Homing completed successfully (position reached zero)");
    transition_to(State::Idle);
  }
}

void StepperEngine::process_speed_update(const Speed &speed) {
  Speed old_speed = current_speed_;
  current_speed_ = speed;

  // Check if standstill reached (in Stopping state)
  if (state_ == State::Stopping && speed.rpm() == 0) {
    ESP_LOGV(TAG_ENGINE, "Standstill reached (speed=0), transitioning to Idle");
    transition_to(State::Idle);
  }
}

void StepperEngine::process_motor_status_update(CommandDecoder::MotorStatus status) {
  // Hardware status validates Engine state - Engine state is leading
  // Only handle critical errors or completion signals
  switch (status) {
    case CommandDecoder::MotorStatus::FAIL:
      // Hardware docs: FAIL (status=0) means "read fail" - no valid status available
      // This is NORMAL when motor is idle/disabled, not an error condition
      // Only log at VERBOSE level to avoid spam
      ESP_LOGVV(TAG_ENGINE, "Motor status: FAIL (no valid status available - motor idle/disabled)");
      break;

    case CommandDecoder::MotorStatus::STOP:
      // Hardware reports standstill - validate against Engine expectations
      // If Engine expects motion but hardware stopped → Error
      if (state_ == State::Moving || state_ == State::Running) {
        ESP_LOGW(TAG_ENGINE, "Unexpected stop: Engine expected motion but hardware stopped");
        transition_to(State::Stopping);
      }
      // If Engine is Stopping and hardware confirms → Idle
      else if (state_ == State::Stopping) {
        ESP_LOGV(TAG_ENGINE, "Motor status: STOP confirmed, transitioning to Idle");
        transition_to(State::Idle);
      }
      // If Engine is Homing and hardware stopped → Homing completed successfully
      else if (state_ == State::Homing) {
        ESP_LOGI(TAG_ENGINE, "✓ Homing completed successfully (status: HOMING → STOP)");
        // Reset position to zero after successful homing
        Position zero_pos = Position::from_steps(0, parent_);
        parent_->set_current_pos(zero_pos);
        parent_->set_target_pos(zero_pos);
        transition_to(State::Idle);
      }
      // If Engine is Calibrating and hardware stopped → Calibration completed
      else if (state_ == State::Calibrating) {
        ESP_LOGI(TAG_ENGINE, "✓ Calibration completed (status: CALIBRATING → STOP)");
        transition_to(State::Idle);
      }
      break;

    case CommandDecoder::MotorStatus::SPEED_UP:
    case CommandDecoder::MotorStatus::SPEED_DOWN:
    case CommandDecoder::MotorStatus::FULL_SPEED:
      // Hardware reports motion - validate against Engine expectations
      // If Engine expects Idle but hardware moving → Inconsistency warning
      if (state_ == State::Idle || state_ == State::Disabled) {
        ESP_LOGW(TAG_ENGINE, "Unexpected motion: Hardware moving but engine state is %s", state_to_string(state_));
        // Don't change state - let engine commands control state
      }
      break;

    case CommandDecoder::MotorStatus::HOMING:
      // Hardware reports homing in progress
      // If Engine is in Homing state → Check if homing completed (position reached zero/endstop)
      if (state_ == State::Homing) {
        // Motor stays in HOMING status until manually stopped or endstop reached
        // Check if position is at zero (homing completed)
        ESP_LOGD(TAG_ENGINE, "HOMING status: current_position=%d, checking for completion...", parent_->current_position);
        if (parent_->current_position == 0) {
          ESP_LOGI(TAG_ENGINE, "✓ Homing completed successfully (position reached zero)");
          transition_to(State::Idle);
        } else {
          ESP_LOGV(TAG_ENGINE, "Homing still in progress (position=%d)", parent_->current_position);
        }
      }
      // If Engine is NOT in Homing state → Someone else started homing (physical buttons?)
      // CRITICAL: Ignore hardware status sync during SettingUp to prevent state overwrites
      else if (state_ != State::SettingUp) {
        ESP_LOGW(TAG_ENGINE, "Unexpected homing: Hardware homing but engine state is %s", state_to_string(state_));
        ESP_LOGW(TAG_ENGINE, "  Possible cause: Manual homing via physical buttons");
        // Sync engine state to hardware reality
        transition_to(State::Homing);
      }
      break;

    case CommandDecoder::MotorStatus::CALIBRATING:
      // Hardware reports calibration - validate against Engine expectations
      // If Engine is NOT in Calibrating state → Manual calibration started
      // CRITICAL: Ignore hardware status sync during SettingUp to prevent state overwrites
      if (state_ != State::Calibrating && state_ != State::SettingUp) {
        ESP_LOGW(TAG_ENGINE, "Unexpected calibration: Hardware calibrating but engine state is %s",
                 state_to_string(state_));
        // Sync engine state to hardware reality
        transition_to(State::Calibrating);
      }
      break;
  }
}

void StepperEngine::process_protection_update(uint8_t protected_status) {
  bool old_protection = protection_triggered_;
  protection_triggered_ = (protected_status != 0);

  // Transition to Error state if protection triggered
  if (protection_triggered_ && !old_protection) {
    ESP_LOGE(TAG_ENGINE, "Protection triggered! Status=0x%02X", protected_status);
    handle_error("Locked-rotor protection triggered");
  }
}

bool StepperEngine::is_target_reached() {
  // Check if current position is within tolerance of target
  Position delta = parent_->current_pos_ - parent_->target_pos_;
  Position tolerance = Position::from_degrees(1.0f, parent_);
  // Use absolute value to check distance in both directions
  return delta.abs() <= tolerance;
}

void StepperEngine::handle_error(const char *error_message) {
  ESP_LOGE(TAG_ENGINE, "Error: %s", error_message);
  transition_to(State::Error);
}

}  // namespace servoxxd
}  // namespace esphome
