# Implementation TODO List

This document tracks all TODOs for the Servo42D ESPHome component implementation.

## ✅ Phase 0: YAML Schema & Code Generation (COMPLETED)
- [x] Add all configuration options to `__init__.py`
- [x] Create action classes in `servo42d_actions.h`
- [x] Add method signatures to `servo42d.h`
- [x] Add configuration member variables to `servo42d.h`
- [x] Create stub implementations in `servo42d.cpp`
- [x] Register all actions with ESPHome automation system
- [x] Create comprehensive example YAML
- [ ] Include Mplyer command
- [ ] Speed calculations

## 🔴 Phase 1: Critical Fix - Positioning & Baseline
**Priority: URGENT - Verifikation der Positionsführung**

### Positionierungspfad prüfen (servo42d.cpp/servo42d_position.cpp)
- [ ] Wechsel auf Mode 4 (absolute axis) evaluieren/umstellen, falls Mode 2 (absolute pulses) unzureichend ist
  - Aktuell: `send_absolute_move_()` nutzt Mode 2 (0x00FE) über `move_to_position_mode2()`
  - Alternative: Mode 4 (0x00F5) verwendet Encoder-Ticks; Konvertierung ist bereits in `move_to_position_mode4()` implementiert
- [x] Implementierung steps → encoder ticks ist vorhanden (Mode 3/4): `ticks = steps × (16384 / steps_per_revolution)`
- [ ] Baseline-/Offset-Logik prüfen: Offset beim ersten Bewegungsauftrag setzen (nicht beim ersten Encoder-Read)
- [x] Positions-Reporting verwendet korrekte Konvertierung (siehe `query_encoder_value()`)

**Test**: Motor bewegt sich reproduzierbar zur Sollposition; kein Drift, sauberes Stoppen

---

## 🟡 Phase 2: Core Actions Implementation
**Priority: HIGH - Basic motor control**

### Ort: `servo42d_motor_control.cpp` (delegiert aus `servo42d.cpp`)

#### 2.1 Enable/Disable (0xF3)
- [x] `enable_motor()` → Write(0xF3, 0x0001)
- [x] `disable_motor()` → Write(0xF3, 0x0000)

#### 2.2 Continuous Movement (0xF6, 0xF7)
- [x] `run_continuous()` → MultiWrite(0xF6) mit Payload {dir, acc, speed}; Speed-Umrechnung steps/s→RPM vorhanden
- [x] `stop_motor()` → MultiWrite(0xF6) mit speed=0 und sanfter Verzögerung (acc klein)
- [x] `emergency_stop()` → Write(0xF7, 0x0098)

#### 2.3 Homing
- [x] Homing korrekt gemäß Handbuch: MultiWrite(0x0090 {HmTrig, HmDir, HmSpeed, EndLimit}) + Write(0x0091, 0x0001)
- [ ] Optional: Zero-Mode (0x009A) als Power-On-GoZero separat exponieren/konfigurierbar machen
 - [x] Virtuelles Homing (noLimit): hm_ma (Homing-Strom) laut Manual modellabhängig berücksichtigen
   - [x] YAML Schema: neues Feld `homing_current` (A/mA) mit Typ-Default (28/35: 0.2A; 42: 0.8A; 57: 0.4A) und Max je Modell
   - [x] C++: Während Homing temporär Working Current auf `homing_current` setzen und nach Ende wiederherstellen
   - [x] Homing-Speed: steps/s → RPM korrekt umrechnen und clampen

#### 2.4 Position Management
- [x] `reset_position()` → Write(0x0092, 0x0001) + lokale Baseline synchronisieren

**Test**: Enable, run/stop, home, reset position – verifiziert mit CommandQueue-Callbacks/Logs

---

## 🟢 Phase 3: Advanced Actions Implementation
**Priority: MEDIUM - Configuration & Recovery**

### Ort: `servo42d_motor_control.cpp`

#### 3.1 Calibration & Recovery (0x80, 0x3D, 0x41)
- [x] `calibrate_motor()` → Write(0x80, 0x0001) – Hinweis: Motor bewegt sich
- [x] `release_protection()` → Write(0x3D, 0x0001)
- [x] `restart_motor()` → Write(0x41, 0x0001)

#### 3.2 Runtime Configuration (0x82, 0x83, 0x84, 0x8F, 0x9B)
- [x] `set_work_mode()` → Write(0x82, mode) + bei OPEN/CLOSE automatische Setzung HOLDING_CURRENT_PERCENT
- [x] `set_working_current_runtime()` → Write(0x83, current_ma)
- [x] `set_holding_current_percent_runtime()` → Write(0x9B, percent) – vFOC Hinweis beibehalten
- [x] `set_microstepping()` → Write(0x84, subdivision)
- [x] `key_lock()` / `key_unlock()` → Write(0x8F, 0x0001/0x0000)

**Test**: Modes/Currents/Microstepping zur Laufzeit ändern

---

## 🔵 Phase 4: Setup/Startup Configuration
**Priority: LOW - Startup behavior**

### Ort: `servo42d.cpp` – `setup()` Sequenz

#### 4.1 Apply Configuration at Startup
- [x] `control_mode_` setzen (0x82) – via `set_work_mode()` inkl. Holding-Current für OPEN/CLOSE
- [x] `working_current_` setzen (0x83)
- [x] `auto_screen_off_` setzen (0x0087)
- [x] `microsteps` setzen (0x0084)
- [x] Motor aktivieren (0x00F3)
- [ ] `lock_keys_at_startup_` berücksichtigen → Write(0x8F, 0/1)
- [ ] Optional: `holding_current_percent_` explizit setzen, falls OPEN/CLOSE und abweichend vom Default gewünscht

#### 4.2 Homing at Startup
- [ ] `home_at_startup_` Flag prüfen und `home()` aufrufen (real/zero-mode gemäß Konfig)
 - [ ] Bei virtuellem Homing Current-Override auch beim Autohoming nutzen

**Test**: Config wird beim Boot angewendet; optionales Homing läuft

---

## 📝 Phase 5: Documentation
**Priority: LOW - User documentation**

- [ ] README aktualisieren:
  - [ ] Alle Konfigurationsoptionen dokumentieren
  - [ ] Alle Aktionen mit Beispielen dokumentieren
  - [ ] Real Homing vs. Zero-Mode (Power-On-GoZero) erklären
  - [ ] Working vs. Holding Current erklären (vFOC Einschränkung)
  - [ ] Vergleichstabelle Work Modes
- [ ] Inline-Kommentare für komplexe Logik ergänzen
- [ ] Troubleshooting Guide erstellen

---

## 🧪 Testing Checklist

### Basic Functionality
- [ ] Motor enables/disables on command
- [ ] Position reporting shows correct values
- [ ] Motor moves correct distance for set_target
- [ ] Emergency stop immediately halts motor

### Homing
- [ ] Real homing works with endstop
- [ ] Virtual homing works without endstop
- [ ] home_at_startup triggers correctly
- [ ] Reset position zeros correctly

### Continuous Movement
- [ ] run_continuous CW works (MultiWrite 0xF6)
- [ ] run_continuous CCW works (MultiWrite 0xF6)
- [ ] stop_motor halts movement (speed=0, acc>0)
- [ ] Speed changes take effect

### Runtime Configuration
- [ ] Work mode change works (requires motor restart)
- [ ] Current changes take effect immediately
- [ ] Microstepping change works
- [ ] Key lock/unlock works

### Startup Behavior
- [ ] Config applied at boot (mode, current, microsteps, screen, enable)
- [ ] home_at_startup executes
- [ ] Keys locked at startup if configured
- [ ] Screen auto-off works


## Neue ToDos aus Code-Review

- [ ] Acceleration konfigurierbar machen und in `run_continuous/stop` sowie `compute_speed_and_accel_` konsistent nutzen
- [ ] Position-Hilfsmethoden (`Servo42dPosition`) auf typsichere Payload-Strukturen umstellen (PositionMode1–4)
- [ ] `home_at_startup_` und `lock_keys_at_startup_` in `setup()` berücksichtigen
- [ ] YAML Codegen (`__init__.py`): Offenen TODO zur Speed-Konvertierung (steps/s) schließen, wenn `steps_per_revolution` bekannt
- [ ] Optional: Zero-Mode (0x009A) als Konfiguration exponieren (Power-On-GoZero)
 - [ ] README dokumentiert `homing_current` (nur noLimit, Default je Modell, Verhalten: temporärer Override)
 - [x] Validierung: homing.direction=NEAREST nur bei virtuellem Homing (use_virtual_home=true)
---

## Implementation Notes

### Command Structure Reference
```cpp
// WriteCommand: Single register write
auto cmd = std::make_unique<WriteCommand>(register_address, value);

// MultiWriteCommand: Multiple consecutive registers
std::vector<uint16_t> params = {param1, param2, param3, param4};
auto cmd = std::make_unique<MultiWriteCommand>(start_register, params);

// Queue priority
command_queue_->enqueue(cmd);        // Normal priority (back of queue)
command_queue_->enqueue_front(cmd);  // High priority (front of queue)
```

### Encoder Conversion Formula
```cpp
// Motor encoder: 16384 ticks per revolution
// ESPHome steps: steps_per_revolution_ (e.g., 3200 for 200 steps × 16 microsteps)
// Conversion: encoder_ticks = steps × (16384 / steps_per_revolution_)
// Example: 1000 steps × (16384 / 3200) = 5120 encoder ticks
```

### Work Mode Limits
```cpp
// SR_OPEN:  Max 400 RPM,  Fixed current
// SR_CLOSE: Max 1500 RPM, Fixed current, holding current works
// SR_vFOC:  Max 3000 RPM, Adaptive current, holding current NOT supported
```

---

## Progress Tracking

| Phase | Status | Completion |
|-------|--------|------------|
| Phase 0: Schema & Code Gen | ✅ DONE | 100% |
| Phase 1: Critical Fix | 🔴 TODO | 0% |
| Phase 2: Core Actions | 🟡 TODO | 0% |
| Phase 3: Advanced Actions | 🟢 TODO | 0% |
| Phase 4: Startup Config | 🔵 TODO | 0% |
| Phase 5: Documentation | 📝 TODO | 0% |

**Overall Progress: 16.7% (1/6 phases complete)**
