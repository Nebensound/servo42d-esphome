# TODO - Implementation Tasks

> **Spec:** [docs/specification/](docs/specification/)

## Status

✅ **Fertig:** Type Classes, Unit Tests, Spec Dokumentation  
🔵 **Skeleton:** Position (teilweise), ServoXxdModbus, StepperEngine, CommandQueue (header only), Actions (templates)  
❌ **Fehlt:** Modbus Commands, State Machine Logic, Polling, Error Handling

---

## Kritischer Pfad (Abhängigkeiten beachten!)

### Phase 1: Foundation ⚠️ BLOCKIERT ALLES

#### 1.1 Position Class ✅ (ERLEDIGT)
**File:** `servoxxd_position.cpp` + `test_position.cpp`

- [x] `from_steps()` - parent pointer korrekt propagiert
- [x] Operators `+ - * /` - parent pointer propagation + parent compatibility checks
- [x] Split-Format-Logik: Alle Getter arbeiten direkt ohne int64_t Umweg
- [x] Tests erweitert: operator*, operator/, parent handling, compatibility warnings
- [x] Tests zum Addieren und Subtrahieren (und andere Operatoren) - `test_operator_edge_cases()` mit 10+ Edge Cases

#### 1.2 Modbus Command Classes ❌ (6-8h) ⚠️ BLOCKIERT ALLES
**Spec:** [02d-layer4-transport.md](docs/specification/02d-layer4-transport.md)  
**Files:** `servoxxd_command.h/.cpp` (NEU)

- [ ] Create header: CommandState enum + BaseCommand + ReadCommand/WriteCommand/WriteMultipleCommand classes
- [ ] Implement execute() for all 3 types (Modbus 0x04, 0x06, 0x10)
- [ ] Implement process_response() with validation
- [ ] State transitions: PENDING → EXECUTING → COMPLETED/FAILED/TIMEOUT
- [ ] Timeout tracking + callbacks
- [ ] Unit tests

#### 1.3 CommandQueue ❌ (4-6h) ⚠️ BLOCKIERT ALLES  
**Spec:** [02c-layer3-command-queue.md](docs/specification/02c-layer3-command-queue.md)  
**File:** `servoxxd_command_queue.cpp` (header exists)

- [ ] `enqueue()` mit Deduplication (identical READ commands)
- [ ] `execute_next()` mit is_executing_ guard (single-flight!)
- [ ] `on_response_received()` - clear guard, tail-recursive execute_next()
- [ ] `on_error_received()` - clear guard, tail-recursive execute_next()
- [ ] `check_timeout()` - detect stuck commands, clear guard
- [ ] Logging + unit tests

### Phase 2: Core Logic (benötigt Phase 1)

#### 2.1 ServoXxd Base Integration 🔵 (4-5h)
**Spec:** [02a-layer1-core.md](docs/specification/02a-layer1-core.md)  
**File:** `servoxxd_modbus.cpp` (13 TODOs)

- [ ] `setup()` (L43,52,61,67): Query motor state, send config, homing_at_startup, mark_failed()
- [ ] `loop()` (L83,87): Sync position, engine_->update(), queue_->check_timeout(), execute_next()
- [ ] `dump_config()` (L133): Log config + motor state
- [ ] Modbus callbacks (L147,157): Delegate to engine
- [ ] Mode validation (L178,191,216): Check Position/Speed Mode vor Actions


#### 2.2 StepperEngine State Machine 🔵 (6-8h)
**Spec:** [02b-layer2-stepper-engine.md](docs/specification/02b-layer2-stepper-engine.md)  
**File:** `servoxxd_stepper_engine.cpp` (25 TODOs)

- [ ] 9 Transitionen (L206,230,102/111,183/188,165/169,376,133): Siehe State Table in Spec
- [ ] Target Override Strategy (L92): Hardware test → Immediate OR Coalescing
- [ ] State validation: Verhindere invalide Transitionen
- [ ] enable/disable, move_to (Mode check), stop, emergency_stop
- [ ] Implement home with homing config query
- [ ] Implement run_continuous with Speed Mode check
- [ ] Implement target override strategy (test hardware first!)
- [ ] Implement response parsing (position, speed, status, protection)
- [ ] Implement error handling and transition to Error state
- [ ] Implement state-specific timeouts
- [ ] Add state transition logging

**Zeitaufwand:** 8-10 Stunden


#### 2.3 Polling Strategy ❌ (3-4h)
**Spec:** [02b-layer2-stepper-engine.md L178-213](docs/specification/02b-layer2-stepper-engine.md)  
**File:** `servoxxd_stepper_engine.cpp` (NEW)

- [ ] poll_interval_ms_/last_poll_time_: Tracking (default 200ms)
- [ ] poll_encoder_position/motor_status/speed: 3 Polling-Methoden in update()
- [ ] Parse poll responses: Update current_position_, protection_status_
- [ ] State transitions: Moving → Idle bei target reached

---


### Phase 3: Actions & Polishing (benötigt Phase 1 + 2)

#### 3.1 Action Classes 🔵 (6-8h)
**Spec:** [02-cpp-interface.md L231-293](docs/specification/02-cpp-interface.md)  
**File:** `servoxxd_actions.h` (18 templates, 11+ TODOs)

- [ ] 18 Action::play() (L382-430): Both Modes (9), Position (4), Speed (1), Runtime Config (6)
- [ ] WorkMode enum (L392): SR_OPEN, SR_CLOSE, SR_VFOC
- [ ] Templatable evaluation: value.value() für std::optional
- [ ] Unit conversion: Speed/Acceleration/Position constructors


#### 3.2 Error Handling ❌ (3-4h)
**Spec:** [02b-layer2-stepper-engine.md](docs/specification/02b-layer2-stepper-engine.md)  
**File:** `servoxxd_stepper_engine.cpp` (NEW)

- [ ] handle_error(): 4 Sources (timeout, protection, comm errors, emergency_stop)
- [ ] Recovery: Error→release_protection()→Idle, retry mit exponential backoff (max 3)
- [ ] Error logging: Command, state, timestamp


#### 3.3 Hardware Tests ⚠️ (2-3h, nach 2.2+2.3)
**Spec:** [02b-layer2-stepper-engine.md L225-275](docs/specification/02b-layer2-stepper-engine.md)  
**File:** `test_hardware_target_override.yaml` (NEW)

- [ ] Target Override Test: move_to() während moving → immediate update OR rejected?
- [ ] Position Reached Detection: Polling threshold für small moves (1°)
- [ ] Update spec mit Ergebnissen


---

## Detaillierte TODOs nach Komponente

### Position Class (1-2h)
**File:** `servoxxd_position.cpp`

- [ ] `from_steps()` verify parent handling (nur diese Methode braucht parent!)
- [ ] 4 operators: +, -, *, / propagate parent pointer


### Commands (6-8h)
**Files:** `servoxxd_command.h/.cpp` (NEW)

- [ ] .h: CommandState enum, BaseCommand/ReadCommand/WriteCommand/WriteMultipleCommand classes
- [ ] .cpp: execute/process_response for 3 classes, timeout tracking, state transitions, unit tests


### CommandQueue (4-6h)
**File:** `servoxxd_command_queue.cpp`

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
- [ ] 18 play() methods: Both Modes (9), Position (4), Speed (1), Runtime Config (6) - siehe Phase 3.1


---

## Gesamtzeitaufwand & Prioritäten

**Total: 42-57h** | Phase 1: 11-16h | Phase 2: 16-21h | Phase 3: 15-20h

**CRITICAL:** 1.1 Position, 1.2 Modbus Commands ⚠️, 1.3 CommandQueue ⚠️  
**HIGH:** 2.1 ServoXxd Base, 2.2 StepperEngine, 2.3 Polling (nach 3.3 Hardware Test)  
**MEDIUM:** 3.1 Actions, 3.2 Error Handling, 3.3 Hardware Tests

---

## Cleanup

- [ ] `docs/structure.md` löschen/archivieren
- [ ] `docs/structure.yaml` löschen/archivieren
- [ ] `docs/to_delete/*` löschen/archivieren



**Vorteile:**
- ✅ Hauptdatei bleibt übersichtlich und fokussiert
- ✅ API-Referenz zentral verfügbar
- ✅ Layer-Details modular und wartbar
- ✅ Klare Navigation mit Cross-References
- ✅ GitHub-friendly (alle Links funktionieren)

**Spec-Referenzen aktualisiert:**
- Alle TODOs verweisen jetzt auf korrekte Zeilennummern in neuer Struktur
- Phase 1/2/3 Aufgaben referenzieren entsprechende Layer-Dateien

