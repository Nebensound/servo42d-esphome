# Specification Quick Issues Reference

**For full details, see**: `VALIDATION-REPORT.md` (English) or `VALIDIERUNGSBERICHT.md` (German)

---

## Must Fix Before Implementation ❗

### 1. Speed Calibration Contradiction
**File**: `02-cpp-interface.md` lines 356-367  
**Issue**: Says calibrated for "16/32/64" but formula uses only "16"  
**Fix**: Clarify if calibrated for only 16 or all three values

### 2. German Text in English Spec
**File**: `02-cpp-interface.md` lines 412-451  
**Issue**: Acceleration section mixes German/English  
**Fix**: Translate to English or create separate German doc

### 3. Broken Cross-Reference
**File**: `02-cpp-interface.md` line 130  
**Issue**: Links to non-existent `02d-layer4-transport-refactoring.md`  
**Fix**: Remove link or create document

### 4. Position Parent Pointer Ambiguity
**File**: `02-cpp-interface.md` lines 468-474  
**Issue**: Comment says "only from_steps needs parent" but get_steps() uses parent  
**Fix**: Clarify when parent is required, document null pointer behavior

---

## Should Fix Before Alpha ⚠️

### 5. Error Recovery Strategy
**File**: `02b-layer2-stepper-engine.md`  
**Missing**: What happens to queue during errors? Recovery paths?  
**Fix**: Add "Error Recovery Flows" section

### 6. Timeout Guidelines
**Files**: Multiple locations  
**Missing**: Default timeout values, how to choose them  
**Fix**: Add "Timeout Guidelines" section with recommended values

### 7. Callback Type Safety
**File**: `02c-layer3-command-queue.md` lines 46-50  
**Issue**: `void(bool)` callback doesn't convey failure reason  
**Fix**: Use enum or separate callbacks for success/timeout/error

### 8. Thread Safety Documentation
**File**: `02c-layer3-command-queue.md` lines 42-84  
**Issue**: No mention of thread safety guarantees  
**Fix**: Document single-threaded assumptions or add mutex

---

## Nice to Have for Production ✨

### Consistency
- Normalize Mode/OperatingMode terminology (issue 2.1)
- Clean up "0_Mode/No_Limit" references (issue 2.3)

### Documentation
- Add position carry/borrow code example (issue 4.3)
- Define command priority system (issue 4.4)
- Improve document navigation (issue 5.1)
- Clarify audience (issue 5.2)

### Validation
- Document working_current validation order (issue 6.1)
- Add unit conversion edge case guarantees (issue 6.2)
- Handle uncalibrated microstepping values (issue 6.3)

---

## Not Blocking But Consider 💭

### Implementation Complexity
- Position sync between internal and base class (issue 3.2)
- Speed hardware compensation API exposure (issue 3.4)

### Base Class Integration
- Overload vs override confusion (issue 1.4)
- ESPHome stepper integration complexity

---

## Summary Statistics

- **Total Issues**: 16
- **Critical**: 4 (must fix before coding)
- **Important**: 4 (fix before alpha)
- **Nice to Have**: 8 (polish for production)

**Overall Grade**: ⭐⭐⭐⭐ (4/5)

**Bottom Line**: Solid specification with manageable issues. Fix the 4 critical items and proceed with implementation. Address important items before first release.

---

**Full Reports**:
- 📄 `VALIDATION-REPORT.md` - Complete analysis (25KB, English)
- 📄 `VALIDIERUNGSBERICHT.md` - Executive summary (11KB, German)

**Generated**: 2025-10-30
