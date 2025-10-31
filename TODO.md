# TODO - Implementation Tasks

> **Spec:** [docs/specification/](docs/specification/)

## Status

✅ **Done:** Type Classes, Unit Tests, Spec Documentation  
🔵 **Skeleton:** Position (partial), ServoXxdModbus, StepperEngine, CommandQueue (header only), Actions (templates)  
❌ **Missing:** Modbus Commands, State Machine Logic, Polling, Error Handling

---

## Critical Path (Mind Dependencies!)

### Phase 1: Foundation ⚠️ BLOCKS EVERYTHING

#### 1.1 Position Class ✅ (COMPLETED)

**File:** `servoxxd_position.cpp` + `test_position.cpp`

- [x] `from_steps()` - parent pointer correctly propagated
- [x] Operators `+ - * /` - parent pointer propagation + parent compatibility checks
- [x] Split-format logic: All getters work directly without int64_t detour
- [x] Tests extended: operator*, operator/, parent handling, compatibility warnings
- [x] Tests with various steps_per_rev (200, 400, 3200, 102400 + fractional)

#### 1.2 Modbus ✅ (COMPLETED - NEEDS REFACTORING)

**Spec:** [02d-layer4-transport.md](docs/specification/02d-layer4-transport.md)  
**Files:** `servoxxd_modbus.h/.cpp`

- [x] Create header: CommandState enum + BaseCommand + Read/Write/MultiWrite classes
- [x] Implement execute() for all 3 types (0x04, 0x06, 0x10)
- [x] Implement process_response() with validation
- [x] State transitions: PENDING → EXECUTING → COMPLETED/FAILED/TIMEOUT
- [x] Timeout tracking + callbacks
- [x] Unit tests (14 tests passing)

**⚠️ Refactoring Required:** See Phase 1.2b below - Transport Abstraction

#### 1.2b Transport Abstraction ❌ (5-6h) 🎯 NEXT PRIORITY

**Spec:** [02d-layer4-transport.md](docs/specification/02d-layer4-transport.md) ✅ FINALIZED  
**Purpose:** Separate hardware-register commands from transport protocol (enables Serial support)

**Hardware Analysis Result:** ✅ Register-Adressen sind IDENTISCH zwischen Modbus und Serial!

- Beispiel: Encoder Addition = Register 0x31 (Modbus) = Command Code 0x31 (Serial)
- Nur Unterschied: Frame-Format (FA/FB vs Modbus) und Checksum (CRC8 vs CRC16)
- → Register-basierte Abstraktion ist perfekt für diesen Use Case!

**Step 1: Transport Interface** (30 min)

- [ ] Create `servoxxd_transport.h` with `ITransport` abstract interface
- [ ] Define `send_read()`, `send_write()`, `send_write_multiple()` methods
- [ ] Define `set_response_callback()`, `set_error_callback()` methods

**Step 2: Modbus Transport Implementation** (45 min)

- [ ] Create `servoxxd_modbus_transport.h/.cpp`
- [ ] Implement `ModbusTransport : public ITransport`
- [ ] Encapsulate Modbus function codes (0x04/0x06/0x10)
- [ ] Route ESPHome Modbus callbacks to generic transport callbacks

**Step 3: Update Commands** (1h)

- [ ] Rename `servoxxd_modbus.h` → `servoxxd_command.h`
- [ ] Change `device_` → `transport_` (ModbusDevice\* → ITransport\*)
- [ ] Update `execute()` to call `transport_->send_*()`
- [ ] Rename classes: `ReadCommand` → `ReadRegisterCommand`, etc.
- [ ] Remove Modbus function code knowledge from commands

**Step 4: Update CommandQueue** (30 min)

- [ ] Accept `ITransport*` instead of `ModbusDevice*`
- [ ] Wire transport callbacks to queue response handlers

**Step 5: Update StepperEngine + ServoXxd** (1h)

- [ ] Create `ModbusTransport` in `ServoXxd::setup()`
- [ ] Pass transport to `StepperEngine` and `CommandQueue`
- [ ] Wire ESPHome Modbus callbacks to `ModbusTransport`

**Step 6: Update Tests** (1h)

- [ ] Create `MockTransport` (simpler than MockModbusDevice)
- [ ] Update `test_modbus_commands.cpp` → `test_commands.cpp`
- [ ] Add `test_modbus_transport.cpp`
- [ ] Verify all 14 tests still pass

**Step 7: Documentation** (30 min)

- [ ] Update TODO.md marking refactoring complete
- [ ] Add future transport examples (SerialTransport stub)

**Benefits:**

- ✅ Commands are transport-agnostic (work with Modbus, Serial, CAN)
- ✅ Future-proof for different hardware protocols
- ✅ Cleaner separation: Commands know *what*, Transport knows *how*
- ✅ Easier testing with MockTransport

#### 1.3 Queue ❌ (4-6h) ⚠️ BLOCKS EVERYTHING (AFTER 1.2b)

**Spec:** [02c-layer3-command-queue.md](docs/specification/02c-layer3-command-queue.md)  
**File:** `servoxxd_queue.cpp` (header exists)

- [ ] `enqueue()` with deduplication (identical READ commands)
- [ ] `execute_next()` with is_executing_ guard (single-flight!)
- [ ] `on_response_received()` - clear guard, tail-recursive execute_next()
- [ ] `on_error_received()` - clear guard, tail-recursive execute_next()
- [ ] `check_timeout()` - detect stuck commands, clear guard
- [ ] Logging + unit tests

### Phase 2: Core Logic (requires Phase 1)

#### 2.1 ServoXxd Base Integration 🔵 (4-5h)

**Spec:** [02a-layer1-core.md](docs/specification/02a-layer1-core.md)  
**File:** `servoxxd_modbus.cpp` (13 TODOs)

- [ ] `setup()` (L43,52,61,67): Query motor state, send config, homing_at_startup, mark_failed()
- [ ] `loop()` (L83,87): Sync position, engine_->update(), queue_->check_timeout(), execute_next()
- [ ] `dump_config()` (L133): Log config + motor state
- [ ] Modbus callbacks (L147,157): Delegate to engine
- [ ] Mode validation (L178,191,216): Check Position/Speed Mode before Actions

#### 2.2 StepperEngine State Machine 🔵 (6-8h)

**Spec:** [02b-layer2-stepper-engine.md](docs/specification/02b-layer2-stepper-engine.md)  
**File:** `servoxxd_stepper_engine.cpp` (25 TODOs)

- [ ] 9 Transitions (L206,230,102/111,183/188,165/169,376,133): See State Table in Spec
- [ ] Target Override Strategy (L92): Hardware test → Immediate OR Coalescing
- [ ] State validation: Prevent invalid transitions
- [ ] enable/disable, move_to (Mode check), stop, emergency_stop
- [ ] Implement home with homing config query
- [ ] Implement run_continuous with Speed Mode check
- [ ] Implement target override strategy (test hardware first!)
- [ ] Implement response parsing (position, speed, status, protection)
- [ ] Implement error handling and transition to Error state
- [ ] Implement state-specific timeouts
- [ ] Add state transition logging

**Time estimate:** 8-10 hours

#### 2.3 Polling Strategy ❌ (3-4h)

**Spec:** [02b-layer2-stepper-engine.md L178-213](docs/specification/02b-layer2-stepper-engine.md)  
**File:** `servoxxd_stepper_engine.cpp` (NEW)

- [ ] poll_interval_ms_/last_poll_time_: Tracking (default 200ms)
- [ ] poll_encoder_position/motor_status/speed: 3 polling methods in update()
- [ ] Parse poll responses: Update current_position_, protection_status_
- [ ] State transitions: Moving → Idle when target reached

---

### Phase 3: Actions & Polishing (requires Phase 1 + 2)

#### 3.1 Action Classes 🔵 (6-8h)

**Spec:** [02-cpp-interface.md L231-293](docs/specification/02-cpp-interface.md)  
**File:** `servoxxd_actions.h` (18 templates, 11+ TODOs)

- [ ] 18 Action::play() (L382-430): Both Modes (9), Position (4), Speed (1), Runtime Config (6)
- [ ] WorkMode enum (L392): SR_OPEN, SR_CLOSE, SR_VFOC
- [ ] Templatable evaluation: value.value() for std::optional
- [ ] Unit conversion: Speed/Acceleration/Position constructors

#### 3.2 Error Handling ❌ (3-4h)

**Spec:** [02b-layer2-stepper-engine.md](docs/specification/02b-layer2-stepper-engine.md)  
**File:** `servoxxd_stepper_engine.cpp` (NEW)

- [ ] handle_error(): 4 sources (timeout, protection, comm errors, emergency_stop)
- [ ] Recovery: Error→release_protection()→Idle, retry with exponential backoff (max 3)
- [ ] Error logging: Command, state, timestamp

#### 3.3 Hardware Tests ⚠️ (2-3h, after 2.2+2.3)

**Spec:** [02b-layer2-stepper-engine.md L225-275](docs/specification/02b-layer2-stepper-engine.md)  
**File:** `test_hardware_target_override.yaml` (NEW)

- [ ] Target Override Test: move_to() during moving → immediate update OR rejected?
- [ ] Position Reached Detection: Polling threshold for small moves (1°)
- [ ] Update spec with results

---

## Detailed TODOs by Component

### Position Class (1-2h)

**File:** `servoxxd_position.cpp`

- [ ] `from_steps()` verify parent handling (only this method needs parent!)
- [ ] 4 operators: +, -, *, / propagate parent pointer

### Modbus ✅ (COMPLETED)

**Files:** `servoxxd_modbus.h/.cpp`

- [x] .h: CommandState enum, BaseCommand/ReadCommand/WriteCommand/MultiWriteCommand classes
- [x] .cpp: execute/process_response for 3 classes, timeout tracking, state transitions, unit tests

### Queue (4-6h)

**File:** `servoxxd_queue.cpp`

- [ ] enqueue (dedup), execute_next (guard), on_response/error, check_timeout, logging, unit tests

### ServoXxdModbus (4-5h)

**File:** `servoxxd_modbus.cpp`

- [ ] L43,52: mark_failed(); L61: Query motor state; L67: Send config; L83: Sync position; L87: engine_->update()
- [ ] L98: LOG_STEPPER; L133: dump_config(); L147,157: Modbus callbacks → engine
- [ ] L178,191,216: Mode validation (Position/Speed)

### StepperEngine (8-10h)

**File:** `servoxxd_stepper_engine.cpp`

- [ ] Commands (L92-294): move_to, stop, emergency_stop, home, run_continuous, enable/disable, release_protection, restart, set_zero
- [ ] Mode checks (L102,165,183): Position/Speed validation
- [ ] Polling (L376,474,487): Parse responses, enqueue reads (encoder, speed, status, protection)
- [ ] Error handling (L390,453): Critical errors, state timeouts
- [ ] State (validation, logging, position_update_callback_)

### Action Classes (6-8h)

**File:** `servoxxd_actions.h`

- [ ] L58: WorkMode enum (SR_OPEN, SR_CLOSE, SR_VFOC)
- [ ] 18 play() methods: Both Modes (9), Position (4), Speed (1), Runtime Config (6) - see Phase 3.1

---

## Total Time & Priorities

**Total: 42-57h** | Phase 1: 11-16h | Phase 2: 16-21h | Phase 3: 15-20h

**CRITICAL:** 1.1 Position ✅, 1.2 Modbus ✅, 1.3 Queue ⚠️  
**HIGH:** 2.1 ServoXxd Base, 2.2 StepperEngine, 2.3 Polling (after 3.3 Hardware Test)  
**MEDIUM:** 3.1 Actions, 3.2 Error Handling, 3.3 Hardware Tests

---

## Cleanup

- [ ] Delete/archive `docs/structure.md`
- [ ] Delete/archive `docs/structure.yaml`
- [ ] Delete/archive `docs/to_delete/*`

**Benefits:**

- ✅ Main file stays clear and focused
- ✅ API reference centrally available
- ✅ Layer details modular and maintainable
- ✅ Clear navigation with cross-references
- ✅ GitHub-friendly (all links work)

**Spec references updated:**

- All TODOs now reference correct line numbers in new structure
- Phase 1/2/3 tasks reference corresponding layer files
