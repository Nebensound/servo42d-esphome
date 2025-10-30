# TODO - Next Implementation Steps

## Übersicht: Implementierungsstatus

### ✅ Vollständig implementiert
- **Type Classes** (Speed, Acceleration, Position): Alle 18 units mit optimierten Getter-Typen
- **Unit Tests**: 50+ Speed tests, 44+ Acceleration tests, Position tests - alle passing
- **Spec Dokumentation**: Type optimization rationale vollständig dokumentiert

### 🔵 Teilweise implementiert (Skeleton vorhanden)
- **Position Class**: Header vollständig, factory methods teilweise implementiert
- **ServoXxdModbus**: Skeleton mit 13 TODOs
- **StepperEngine**: Skeleton mit 25 TODOs, State machine definiert
- **CommandQueue**: Header vorhanden, keine Implementation
- **Action Classes**: 18 Templates definiert, keine play() methods

### ❌ Nicht implementiert
- **Modbus Command Classes**: ReadCommand, WriteCommand, MultiWriteCommand fehlen komplett
- **State Machine Logic**: Transitions, validations nicht implementiert
- **Polling Strategy**: Encoder/Speed/Status queries nicht implementiert
- **Error Handling**: Timeout, protection, recovery nicht implementiert

---

## Kritischer Pfad: Implementierungsreihenfolge

Die Implementierung muss dieser Reihenfolge folgen, da jede Stufe auf der vorherigen aufbaut:

### Phase 1: Foundation (BLOCKIEREND für alles andere)

#### 1.1 Position Class Factory Methods (Priority: CRITICAL) 🔵 IN PROGRESS
**Blockiert:** Alle Actions, die Position-Objekte benötigen

**Status:** 2/7 factory methods implementiert, 5 noch offen

**Fehlende Implementierungen:**
- [ ] `Position::from_degrees()` - add parent parameter, implement conversion
- [ ] `Position::from_radians()` - add parent parameter, implement conversion  
- [ ] `Position::from_arcminutes()` - add parent parameter, implement conversion
- [ ] `Position::from_arcseconds()` - add parent parameter, implement conversion
- [ ] `Position::from_steps()` - verify parent parameter handling

**Zusätzlich:**
- [ ] Operator methods (+ - * /) - propagate parent pointer
- [ ] Test factory methods mit verschiedenen steps_per_rev/microsteps Werten

**Zeitaufwand:** 2-3 Stunden

#### 1.2 Modbus Command Classes (Priority: CRITICAL) ❌ NICHT IMPLEMENTIERT
**Blockiert:** CommandQueue, StepperEngine, ServoXxdModbus - ALLES

**Spec-Referenz:** 02-cpp-interface.md, Lines 1045-1100

**Zu implementieren:**
```cpp
// Neue Dateien: servoxxd_modbus_commands.h/.cpp

enum class CommandState {
  PENDING, EXECUTING, COMPLETED, FAILED, TIMEOUT
};

class BaseCommand {
 public:
  virtual void execute(modbus::ModbusDevice* device) = 0;
  virtual void process_response(const std::vector<uint8_t>& data) = 0;
  virtual uint8_t get_function_code() const = 0;
  virtual const char* get_command_name() const = 0;
  
  CommandState state_;
  uint16_t register_address_;
  uint32_t timeout_ms_;
  uint32_t start_time_ms_;
  std::function<void(bool success)> completion_callback_;
  std::function<void(const std::vector<uint8_t>&)> data_callback_;
};

class ReadCommand : public BaseCommand {
  // Function code 0x04 - Read Input Registers
  // Constructor: ReadCommand(address, quantity)
  // Response: quantity×2 bytes
};

class WriteCommand : public BaseCommand {
  // Function code 0x06 - Write Single Register
  // Constructor: WriteCommand(address, value)
  // Response: echo of address + value
};

class MultiWriteCommand : public BaseCommand {
  // Function code 0x10 - Write Multiple Registers
  // Constructor: MultiWriteCommand(address, values[])
  // Response: echo of address + quantity
};
```

**Tasks:**
- [ ] Create `servoxxd_modbus_commands.h` with class definitions
- [ ] Create `servoxxd_modbus_commands.cpp` with implementations
- [ ] Implement state machine transitions (PENDING → EXECUTING → terminal states)
- [ ] Implement execute() for all 3 command types
- [ ] Implement process_response() with validation
- [ ] Add timeout tracking (start_time_ms_ + timeout_ms_)
- [ ] Add completion/data callbacks
- [ ] Add unit tests for each command type

**Zeitaufwand:** 6-8 Stunden

#### 1.3 CommandQueue Implementation (Priority: CRITICAL) ❌ NICHT IMPLEMENTIERT
**Blockiert:** StepperEngine, alle Modbus-Kommunikation

**Spec-Referenz:** 02-cpp-interface.md, Lines 950-1050

**Status:** Header existiert (servoxxd_command_queue.h), keine Implementation

**Zu implementieren:**
```cpp
class CommandQueue {
 public:
  // Queue management
  void enqueue(std::unique_ptr<BaseCommand> cmd);
  void execute_next();  // Start next PENDING command if not is_executing_
  void check_timeout(); // Called in loop(), detects TIMEOUT state
  
  // Modbus callbacks (called from StepperEngine)
  void on_response_received(const std::vector<uint8_t>& data);
  void on_error_received(uint8_t function_code, uint8_t exception_code);
  
  bool is_executing() const { return is_executing_; }
  bool is_empty() const { return commands_.empty(); }
  
 private:
  std::deque<std::unique_ptr<BaseCommand>> commands_;
  bool is_executing_ = false;  // Execution guard (mutex)
  modbus::ModbusDevice* device_ = nullptr;
};
```

**Implementation Requirements (from spec):**
1. **Single-flight execution:** Only one command in EXECUTING state at a time
2. **Deduplication:** Skip identical pending READ commands
3. **Timeout handling:** check_timeout() detects stuck commands, clears is_executing_
4. **Tail-recursive processing:** After response/error/timeout, call execute_next()
5. **Non-blocking:** enqueue() returns immediately

**Tasks:**
- [ ] Implement enqueue() with deduplication logic
- [ ] Implement execute_next() with is_executing_ guard
- [ ] Implement on_response_received() - clear guard, process next
- [ ] Implement on_error_received() - clear guard, process next
- [ ] Implement check_timeout() - detect timeouts, clear guard, process next
- [ ] Add logging for queue state changes
- [ ] Add unit tests for queue behavior

**Zeitaufwand:** 4-6 Stunden

---

### Phase 2: Core Logic (benötigt Phase 1)

#### 2.1 ServoXxdModbus Base Class Integration (Priority: HIGH) 🔵 SKELETON VORHANDEN
**Blockiert:** setup/loop lifecycle, ESPHome integration

**Status:** 13 TODOs in servoxxd_modbus.cpp

**Spec-Referenz:** 02-cpp-interface.md, Lines 20-50

**Tasks:**
- [ ] `setup()` - Lines 43, 52, 61, 67
  - Query initial motor state (position, speed, status)
  - Send initial configuration to hardware (microsteps, current, mode, etc.)
  - Enable homing_at_startup if configured
  - Error handling: mark_failed() on communication errors
  
- [ ] `loop()` - Lines 83, 87
  - Sync position with ESPHome base class (current_position → target_position)
  - Call engine_->update() for state machine + polling
  - Call queue_->check_timeout() for stuck command detection
  - Call queue_->execute_next() for opportunistic execution
  
- [ ] `dump_config()` - Line 133
  - Log component configuration (steps_per_rev, microsteps, mode, etc.)
  - Log motor state (position, speed, enabled/disabled)
  - Add ESP_LOGCONFIG calls
  
- [ ] Modbus Callbacks - Lines 147, 157
  - Implement on_modbus_data() - delegate to engine_->on_modbus_response()
  - Implement on_modbus_error() - delegate to engine_->on_modbus_error()
  
- [ ] Mode Validation - Lines 178, 191, 216
  - Check Position/Speed Mode in set_target(), home(), run_continuous()
  - Log warnings for invalid mode operations
  - Prevent undefined behavior

**Zeitaufwand:** 4-5 Stunden

#### 2.2 StepperEngine State Machine (Priority: HIGH) 🔵 SKELETON VORHANDEN
**Blockiert:** Alle Movement Actions, Homing, Error Handling

**Status:** 25 TODOs in servoxxd_stepper_engine.cpp

**Spec-Referenz:** 02-cpp-interface.md, Lines 52-245

**State Machine Transitions (aus Spec):**

| From → To | Trigger | Tasks |
|-----------|---------|-------|
| Disabled → Idle | enable() | Line 206: Send enable command via queue |
| Idle → Disabled | disable() | Line 230: Send disable command via queue |
| Idle → Moving | move_to() | Lines 102, 111: Check Position Mode, send move command |
| Idle → Running | run_continuous() | Lines 183, 188: Check Speed Mode, send speed command |
| Idle → Homing | home() | Lines 165, 169-170: Check Position Mode, query config, send command |
| Moving → Idle | Target reached | Line 376: Parse encoder position, compare with target |
| Running → Stopping | stop() | Line 133: Send stop command |
| Stopping → Idle | Speed = 0 | Line 376: Parse speed response |
| * → Error | Protection | Line 390: Handle critical errors |

**Target Override Strategy (aus Spec):**
- **Line 92:** Send new target during motion (requires hardware test!)
- Hardware test procedure documented in spec (Lines 225-275)
- Choose ONE strategy: Immediate update OR coalescing buffer

**Tasks:**
- [ ] State transition validation - prevent invalid transitions
- [ ] Implement enable/disable state changes
- [ ] Implement move_to with Position Mode check
- [ ] Implement stop with deceleration
- [ ] Implement emergency_stop - immediate halt, set Error state
- [ ] Implement home with homing config query
- [ ] Implement run_continuous with Speed Mode check
- [ ] Implement target override strategy (test hardware first!)
- [ ] Implement response parsing (position, speed, status, protection)
- [ ] Implement error handling and transition to Error state
- [ ] Implement state-specific timeouts
- [ ] Add state transition logging

**Zeitaufwand:** 8-10 Stunden

#### 2.3 Polling Strategy (Priority: HIGH) ❌ NICHT IMPLEMENTIERT
**Blockiert:** Position tracking, Speed monitoring, Error detection

**Spec-Referenz:** 02-cpp-interface.md, Lines 60-130

**Polled Status Values:**
1. **Encoder Position** (Command 0x30) - continuously, even when disabled
2. **Motor Speed** (Command 0x32) - for state transitions (Stopping → Idle)
3. **Motor Status** (Command 0x3A) - enabled/disabled state
4. **Protection Status** (Command 0x3E) - locked-rotor detection

**Implementation Requirements:**
- Default poll interval: 200ms (configurable)
- Enqueue read commands in update() when poll_interval elapsed
- Process responses in on_modbus_response()
- Update internal state (current_position_, current_speed_, etc.)
- Trigger state transitions based on polled values
- Call position_update_callback_ if position changed significantly (>10 steps)

**Tasks:**
- [ ] Add poll_interval_ms_ member (default: 200)
- [ ] Add last_poll_time_ms_ member
- [ ] Implement update() poll cycle
- [ ] Enqueue ReadCommand for encoder (0x30)
- [ ] Enqueue ReadCommand for speed (0x32)
- [ ] Enqueue ReadCommand for motor status (0x3A)
- [ ] Enqueue ReadCommand for protection status (0x3E)
- [ ] Parse responses and update internal state
- [ ] Trigger state transitions (e.g., Moving → Idle when target reached)
- [ ] Call position_update_callback_ on significant changes
- [ ] Handle protection status → transition to Error state

**Zeitaufwand:** 4-6 Stunden

---

### Phase 3: Actions & Polishing (benötigt Phase 1 + 2)

#### 3.1 Action Classes play() Methods (Priority: MEDIUM) 🔵 TEMPLATES VORHANDEN
**Blockiert:** YAML actions functionality

**Status:** 18 action templates in servoxxd_actions.h, 11+ TODOs

**Spec-Referenz:** 02-cpp-interface.md, Lines 380-430

**Action Categories:**

**Both Modes (8 actions):**
- [ ] EnableAction::play() - Line 382: Call parent_->enable()
- [ ] DisableAction::play() - Line 383: Call parent_->disable()
- [ ] EmergencyStopAction::play() - Line 384: Call parent_->emergency_stop()
- [ ] StopAction::play() - Line 385: Call parent_->stop(deceleration)
- [ ] CalibrateAction::play() - Line 386: Call parent_->calibrate()
- [ ] ReleaseProtectionAction::play() - Line 387: Call parent_->release_protection()
- [ ] RestartAction::play() - Line 388: Call parent_->restart()
- [ ] KeyLockAction::play() - Line 389: Call parent_->key_lock()
- [ ] KeyUnlockAction::play() - Line 390: Call parent_->key_unlock()

**Position Mode (4 actions):**
- [ ] SetTargetAction::play() - Line 399: Evaluate templatable position, call parent_->set_target(position)
- [ ] ReportPositionAction::play() - Line 403: Evaluate templatable position, call parent_->report_position(position)
- [ ] HomeAction::play() - Line 405: Call parent_->home(no_restart)
- [ ] SetZeroAction::play() - Line 406: Call parent_->set_zero()

**Speed Mode (1 action):**
- [ ] RunContinuousAction::play() - Line 417: Evaluate templatable speed/accel, call parent_->run_continuous(speed, accel)

**Runtime Config (6 actions):**
- [ ] SetWorkModeAction::play() - Line 425: Call parent_->set_work_mode(mode)
- [ ] SetWorkingCurrentAction::play() - Line 426: Evaluate templatable current, call parent_->set_working_current(current)
- [ ] SetHoldingCurrentPercentAction::play() - Line 427: Evaluate templatable percent, call parent_->set_holding_current_percent(percent)
- [ ] SetMicrosteppingAction::play() - Line 428: Evaluate templatable subdivision, call parent_->set_microstepping(subdivision)
- [ ] SetSpeedAction::play() - Line 429: Evaluate templatable speed, call parent_->set_speed(speed)
- [ ] SetAccelerationAction::play() - Line 430: Evaluate templatable accel, call parent_->set_acceleration(accel)

**Additional Requirements:**
- [ ] Define WorkMode enum - Line 392 (SR_OPEN, SR_CLOSE, SR_VFOC)
- [ ] Implement templatable value evaluation (value.value() for std::optional)
- [ ] Add unit conversion support (Speed, Acceleration, Position constructors)

**Zeitaufwand:** 6-8 Stunden

#### 3.2 Error Handling & Recovery (Priority: MEDIUM) ❌ NICHT IMPLEMENTIERT
**Spec-Referenz:** 02-cpp-interface.md, Lines 160-200

**Error Sources:**
1. **Modbus Timeout** - No response within timeout_ms
2. **Protection Triggered** - Locked-rotor protection status != 0
3. **Communication Errors** - Modbus exception codes
4. **Emergency Stop** - User-triggered emergency halt

**Recovery Paths:**
- Error state → release_protection() → Idle state
- Timeout → retry with exponential backoff (max 3 retries)
- Communication error → log + continue (non-critical)

**Tasks:**
- [ ] Implement handle_error() in StepperEngine
- [ ] Add error state transition logic
- [ ] Implement retry logic with exponential backoff
- [ ] Add emergency flag for emergency_stop()
- [ ] Implement release_protection() recovery
- [ ] Add error logging with context (command, state, timestamp)

**Zeitaufwand:** 3-4 Stunden

#### 3.3 Hardware Integration Tests (Priority: MEDIUM) ❌ NICHT IMPLEMENTIERT

**Test Categories:**
1. **Target Override Test** - Determine hardware capability
2. **Movement Tests** - move_to, home, run_continuous
3. **Mode Switching** - Position ↔ Speed Mode
4. **Protection Tests** - Trigger locked-rotor, verify Error state
5. **Config Tests** - Change microsteps, current, speed at runtime

**Tasks:**
- [ ] Create test_hardware_target_override.yaml
- [ ] Document target override strategy choice
- [ ] Create test_hardware_movement_complete.yaml
- [ ] Create test_hardware_error_recovery.yaml
- [ ] Create test_hardware_runtime_config.yaml
- [ ] Document test results

**Zeitaufwand:** 6-8 Stunden

---

## Detaillierte TODOs nach Komponente

### Position Class (2-3h)

**servoxxd_position.cpp:**
- [ ] Line 115-130: Implement `from_degrees()` with parent parameter
- [ ] Line 155-170: Implement `from_radians()` with parent parameter
- [ ] Line 185-200: Implement `from_arcminutes()` with parent parameter
- [ ] Line 215-230: Implement `from_arcseconds()` with parent parameter
- [ ] Line 245-260: Verify `from_steps()` parent parameter handling
- [ ] Operator+ propagate parent pointer
- [ ] Operator- propagate parent pointer
- [ ] Operator* propagate parent pointer
- [ ] Operator/ propagate parent pointer

### Modbus Commands (6-8h)

**NEW FILE: servoxxd_modbus_commands.h:**
- [ ] Define CommandState enum
- [ ] Define BaseCommand abstract class
- [ ] Define ReadCommand class
- [ ] Define WriteCommand class
- [ ] Define MultiWriteCommand class

**NEW FILE: servoxxd_modbus_commands.cpp:**
- [ ] Implement ReadCommand::execute()
- [ ] Implement ReadCommand::process_response()
- [ ] Implement WriteCommand::execute()
- [ ] Implement WriteCommand::process_response()
- [ ] Implement MultiWriteCommand::execute()
- [ ] Implement MultiWriteCommand::process_response()
- [ ] Implement timeout tracking logic
- [ ] Implement state transition logic
- [ ] Add unit tests

### CommandQueue (4-6h)

**servoxxd_command_queue.cpp:**
- [ ] Implement enqueue() with deduplication
- [ ] Implement execute_next() with is_executing_ guard
- [ ] Implement on_response_received()
- [ ] Implement on_error_received()
- [ ] Implement check_timeout()
- [ ] Add queue state logging
- [ ] Add unit tests

### ServoXxdModbus (4-5h)

**servoxxd_modbus.cpp:**
- [ ] Line 43: mark_failed() when Component base accessible
- [ ] Line 52: mark_failed() when Component base accessible
- [ ] Line 61: Query initial motor state (position, speed, status)
- [ ] Line 67: Send initial configuration (microsteps, current, mode)
- [ ] Line 83: Sync position with ESPHome base class
- [ ] Line 87: Handle polling via engine_->update()
- [ ] Line 98: Use proper LOG_STEPPER macro
- [ ] Line 133: Add detailed state info to dump_config()
- [ ] Line 147: Implement on_modbus_data() - delegate to engine
- [ ] Line 157: Implement on_modbus_error() - delegate to engine
- [ ] Line 178: Add Position Mode validation in set_target()
- [ ] Line 191: Add Position Mode validation in home()
- [ ] Line 216: Add Speed Mode validation in run_continuous()

### StepperEngine (8-10h)

**servoxxd_stepper_engine.cpp:**
- [ ] Line 92: Send new target during motion (hardware test!)
- [ ] Line 102: Check Position Mode in move_to()
- [ ] Line 111: Send move command via queue
- [ ] Line 133: Send stop command via queue
- [ ] Line 151: Send emergency stop to hardware
- [ ] Line 165: Check Position Mode in home()
- [ ] Line 169-170: Query homing config, send command
- [ ] Line 183: Check Speed Mode in run_continuous()
- [ ] Line 188: Send speed command via queue
- [ ] Line 206: Send enable command via queue
- [ ] Line 230: Send disable command via queue
- [ ] Line 253: Send release protection command
- [ ] Line 280: Send restart command
- [ ] Line 294: Send set zero command
- [ ] Line 376: Parse response for polled values (position, speed, status)
- [ ] Line 390: Handle critical errors (protection, timeout)
- [ ] Line 453: Implement state-specific timeouts
- [ ] Line 474: Enqueue read commands for polling (encoder)
- [ ] Line 487: Enqueue read commands for polling (speed, status, protection)
- [ ] Add state transition validation
- [ ] Add state transition logging
- [ ] Add position_update_callback_ invocation

### Action Classes (6-8h)

**servoxxd_actions.h:**
- [ ] Line 58: Define WorkMode enum (SR_OPEN, SR_CLOSE, SR_VFOC)
- [ ] Implement play() for EnableAction
- [ ] Implement play() for DisableAction
- [ ] Implement play() for EmergencyStopAction
- [ ] Implement play() for StopAction
- [ ] Implement play() for CalibrateAction
- [ ] Implement play() for ReleaseProtectionAction
- [ ] Implement play() for RestartAction
- [ ] Implement play() for KeyLockAction
- [ ] Implement play() for KeyUnlockAction
- [ ] Implement play() for SetTargetAction (templatable position)
- [ ] Implement play() for ReportPositionAction (templatable position)
- [ ] Implement play() for HomeAction
- [ ] Implement play() for SetZeroAction
- [ ] Implement play() for RunContinuousAction (templatable speed/accel)
- [ ] Implement play() for SetWorkModeAction
- [ ] Implement play() for SetWorkingCurrentAction (templatable)
- [ ] Implement play() for SetHoldingCurrentPercentAction (templatable)
- [ ] Implement play() for SetMicrosteppingAction (templatable)
- [ ] Implement play() for SetSpeedAction (templatable)
- [ ] Implement play() for SetAccelerationAction (templatable)

---

## Geschätzter Gesamtzeitaufwand

### Phase 1 (Foundation): 12-17 Stunden
- Position factory methods: 2-3h
- Modbus command classes: 6-8h
- CommandQueue: 4-6h

### Phase 2 (Core Logic): 16-21 Stunden
- ServoXxdModbus base integration: 4-5h
- StepperEngine state machine: 8-10h
- Polling strategy: 4-6h

### Phase 3 (Actions & Polish): 15-20 Stunden
- Action play() methods: 6-8h
- Error handling: 3-4h
- Hardware tests: 6-8h

**Total: 43-58 Stunden** (ca. 1-1.5 Wochen Vollzeit)

---

## Prioritäten-Zusammenfassung

### CRITICAL (Phase 1 - muss zuerst)
1. Position factory methods (2-3h)
2. Modbus command classes (6-8h) - **BLOCKIERT ALLES**
3. CommandQueue (4-6h) - **BLOCKIERT ALLES**

### HIGH (Phase 2 - benötigt Phase 1)
4. ServoXxdModbus base integration (4-5h)
5. StepperEngine state machine (8-10h)
6. Polling strategy (4-6h)

### MEDIUM (Phase 3 - benötigt Phase 1+2)
7. Action play() methods (6-8h)
8. Error handling (3-4h)
9. Hardware tests (6-8h)

---

## Cleanup

- [ ] `docs/structure.md` löschen/archivieren
- [ ] `docs/structure.yaml` löschen/archivieren
- [ ] `docs/to_delete/*` löschen/archivieren

