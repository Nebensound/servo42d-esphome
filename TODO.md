# TODO - Next Implementation Steps

## C++ Implementation

### 1. Type Classes (Priority: HIGH) ✅ DONE

- [x] `Speed` class constructor mit 7 units → `int16_t rpm_` ✅
- [x] `Acceleration` class constructor mit 5 units → `uint8_t acc_` (0-255) ✅
- [x] `Position` class constructor mit 6 units → `int32_t revs_` + `uint16_t angle_ticks_` ✅
- [x] Unit conversion tests für alle 18 units ✅ (tests/unit/)
- [x] C++11 kompatibel (ESPHome Standard, keine std::numbers) ✅
- [x] Alle Unit Tests kompilieren und bestehen ✅

### 2. Core Infrastructure (Priority: HIGH) 🔵 SKELETON CREATED

- [x] `ServoXxdModbus` main component class - skeleton mit TODO-Kommentaren
- [x] `StepperEngine` state machine - skeleton mit TODO-Kommentaren
- [x] `CommandQueue` für Modbus-Befehle - skeleton mit TODO-Kommentaren
- [ ] Base class integration implementieren (setup, loop, dump_config)
- [ ] State machine logic implementieren
- [ ] Command queue logic implementieren
- [ ] Modbus command/response handling
- [ ] Polling strategy implementieren

### 3. Action Classes (Priority: MEDIUM) 🔵 SKELETON CREATED

- [x] 18 Action-Klassen als templates angelegt
- [ ] Unit support in allen Actions implementieren
- [ ] Templatable value handling implementieren
- [ ] play() methods für alle Actions

### 4. Testing (Priority: HIGH)

- [ ] Unit conversion accuracy tests
- [ ] Hardware integration tests
- [ ] Example configurations testen

## Cleanup

- [ ] `docs/structure.md` löschen/archivieren
- [ ] `docs/structure.yaml` löschen/archivieren
- [ ] `docs/to_delete/*` löschen/archivieren

