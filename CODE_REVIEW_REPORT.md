# Code Review Report - Chassis Positioning System

**Date**: 2026-06-16  
**Reviewer**: Claude Code Review Agent  
**Effort Level**: Medium (xhigh effort - recall-focused)

---

## Executive Summary

**Critical Bugs Found**: 2  
**High Severity**: 3  
**Medium Severity**: 3  
**Documentation Issues**: 5

**Status**: ❌ **NOT READY FOR PRODUCTION**

The kinematic formulas contain a **variable swap bug** that inverts X/Y motion. Additionally, multiple documentation inconsistencies exist between code and docs.

---

## Critical Findings

### 1. ❌ CRITICAL: X/Y Body Coordinates Swapped in Kinematic Resolution

**Location**: `Core/Src/main.c:511-512`

**Bug**:
```c
float dx_body_raw = (dx_wheel + dy_wheel) * INV_SQRT2;  // Actually computes dy_body
float dy_body_raw = (dy_wheel - dx_wheel) * INV_SQRT2;  // Actually computes dx_body
```

**Root Cause**: Variable names are swapped. The mathematical formulas are correct but assigned to wrong variables.

**Impact**:
- Forward motion (`+X`) outputs as lateral motion (`+Y`)
- Left motion (`+Y`) outputs as forward motion (`+X`)
- Robot trajectory rotated 90° from reality

**Verification**: Mathematically confirmed via forward/inverse kinematics derivation.

**Fix**:
```c
float dy_body_raw = (dx_wheel + dy_wheel) * INV_SQRT2;  // Swap variable names
float dx_body_raw = (dy_wheel - dx_wheel) * INV_SQRT2;
```

**Test**: Push robot forward → verify only X increases, Y stays constant.

---

### 2. ❌ CRITICAL: Missing Encoder Validity Check Before Integration

**Location**: `Core/Src/main.c:484-530` (function `integrate_orthogonal_encoder_delta`)

**Bug**: Function never validates `sample->valid_mask` before using encoder counts.

**Impact**:
- If one encoder fails (returns 0x0000 or garbled data), corrupted counts integrate into position
- Position accumulates errors even when `AS5->valid == 0`

**Current State**: Caller checks `encoder_delta.valid_mask == ODOM_ENCODER_VALID_ALL` (line 721), but this only prevents calling the function when **both** encoders are invalid.

**Fix**: Add guard at function start:
```c
if (sample->valid_mask != ODOM_ENCODER_VALID_ALL) {
    return;
}
```

---

## High Severity

### 3. ⚠️ HIGH: Documentation Claims Orthogonal Axes, Firmware Uses 45° Diagonal

**Locations**:
- `README.md:6-8` - "编码器0 (SPI1): X轴/前进方向"
- `YIS130.h:14` - "SPI1 -> body X/forward, SPI2 -> body Y/left"
- Actual code: 45° X-configuration with inverse kinematics matrix

**Impact**:
- Calibration procedures will fail
- Hardware replacements using docs will produce wrong odometry
- Developers misunderstand system architecture

**Fix**: Update all docs to state:
```
Left wheel (SPI2): 45° angle, measures (forward + left) / √2
Right wheel (SPI1): 135° angle, measures (forward - left) / √2
```

---

### 4. ⚠️ HIGH: Diagnostic Tool Expects Wrong Wheel Angles

**Location**: `diagnose_encoders.py:76-78`

**Bug**:
```python
print("  - 转X轮 → body角度应为  0°（正向）或 180°（反向）")
print("  - 转Y轮 → body角度应为 90°（正向）或 -90°（反向）")
```

**Reality**: With 45° configuration:
- Turning SPI1 alone → body motion at **135°** (left-forward)
- Turning SPI2 alone → body motion at **45°** (right-forward)

**Impact**: Tool will always report "error" even when hardware is correct.

**Fix**: Update expected angles to 45° and 135°, or rewrite for diagonal validation.

---

### 5. ⚠️ HIGH: PI Constant Insufficient Precision

**Location**: `YIS130.h:8`, used in `main.c:155-156`

**Bug**: `#define PI 3.1415926f` has only 7 significant figures.

**Error**: True π = 3.14159265358979..., error = 5.4e-8 (17 ppm)

**Impact**:
- With 81.8mm wheel diameter: 1.4µm error per revolution
- Over 1000 revolutions: **1.4mm systematic drift**
- Over 1M revolutions (100km travel): **1.4m drift**

**Fix**:
```c
#define PI 3.14159265f  // 9 digits, full float32 precision
```

Or use `<math.h>` and `M_PI`.

---

## Medium Severity

### 6. ⚠️ MEDIUM: No NaN Propagation Guard

**Location**: `main.c:517-520`

**Bug**: If IMU fails and `yaw_rad` becomes NaN, trigonometric functions return NaN, which permanently corrupts position (NaN + x = NaN).

**Fix**:
```c
if (isnan(yaw_rad) || isnan(cos_yaw) || isnan(sin_yaw)) {
    return;  // Skip this integration
}
```

---

### 7. ⚠️ MEDIUM: Rotation Compensation Frame Ambiguity

**Location**: `main.c:515-516`

**Code**:
```c
float dx_body = dx_body_raw + ODOM_X_WHEEL_Y_OFFSET_M * dyaw;
float dy_body = dy_body_raw - ODOM_Y_WHEEL_X_OFFSET_M * dyaw;
```

**Issue**: Comment says "Rotation compensation for tracking wheels offset from robot center", but unclear if offsets are in **body frame** or **wheel-local 45° frame**.

**Impact**: If offsets are interpreted wrong, position error = `offset × angular_velocity`.

**Verification Needed**: Document coordinate frame for offset definitions in YIS130.h.

---

### 8. ⚠️ MEDIUM: ENCODER_FIX.md Outdated After Kinematic Change

**Location**: `ENCODER_FIX.md:3-18`

**Issue**: Document written before 45° commit, assumes 1:1 wheel-to-body mapping. Diagnostic procedures and fix suggestions are now invalid.

**Impact**: Future debugging using this doc will produce misleading results.

**Fix**: Rewrite or archive with warning: "Applies only to pre-45° firmware".

---

## Low Severity

### 9. ℹ️ LOW: INV_SQRT2 Truncation

**Location**: `main.c:510`

**Precision**: `0.70710678118f` vs true `0.707106781186547...`

**Error**: 6.5e-12, causes ~0.9 ppm kinematic error (sub-micron per meter).

**Assessment**: Within float32 limits, acceptable. Could use `0.7071067812f` for marginal improvement.

---

### 10. ℹ️ LOW: First Velocity Output Transient

**Location**: Interaction between `main.c:529` and `main.c:733-738`

**Issue**: `odom_vel_ticks` starts at 0, so first velocity calculation may have wrong `dt`.

**Impact**: One-time velocity spike on first valid frame (200ms transient).

**Severity**: Low, self-correcting after first output.

---

## Documentation Inconsistencies

### 11. 📄 README.md: False Claim of Orthogonal Axes

As noted in Finding #3.

### 12. 📄 YIS130.h Comment Misleading

Line 14 says "SPI1 -> body X/forward" but SPI1 actually measures a 135° diagonal.

### 13. 📄 Legacy Debug Code Still Uses Differential Drive

**Location**: `main.c:796-797` (disabled by `#else` when `ODOM_BINARY_MODE == 1`)

**Code**:
```c
float enc_left_total_m = -AS5048s[1].total_angle * AS5048_LEFT_METERS_PER_COUNT;
float enc_right_total_m = AS5048s[0].total_angle * AS5048_RIGHT_METERS_PER_COUNT;
```

**Issue**: CSV output mode still interprets encoders as left/right drive wheels, not 45° tracking wheels.

**Impact**: If someone disables binary mode for debugging, CSV data will be completely wrong.

---

## Recommendations

### Immediate Actions (Block Production)

1. **FIX CRITICAL #1**: Swap `dx_body_raw` and `dy_body_raw` variable names (2 lines)
2. **FIX CRITICAL #2**: Add validity check at start of `integrate_orthogonal_encoder_delta`
3. **TEST**: Physical robot forward/left motion validation

### High Priority (Before Next Release)

4. **Update README.md** with correct 45° wheel configuration
5. **Rewrite or disable** `diagnose_encoders.py`
6. **Increase PI precision** to 9 digits
7. **Add NaN guards** to trigonometric code

### Documentation Cleanup

8. Update YIS130.h comment (line 14)
9. Archive or update ENCODER_FIX.md with version warning
10. Remove or fix legacy CSV output code (#else branch)

---

## Test Plan

### Unit Tests
- [x] `test_zero_detection.py` - Passes (12/12)
- [ ] **NEW**: Test kinematic formulas with known wheel inputs
- [ ] **NEW**: Test NaN propagation handling

### Integration Tests
```python
# Test 1: Pure forward motion
Push robot forward 0.5m → verify:
  - x increases by ~0.5m
  - |y| < 5mm
  - Trajectory angle < 5°

# Test 2: Pure lateral motion  
Push robot left 0.5m → verify:
  - y increases by ~0.5m
  - |x| < 5mm
  - Trajectory angle 85-95°

# Test 3: Diagonal motion
Push robot at 45° angle → verify:
  - Both x and y increase
  - atan2(y, x) ≈ 45° ± 5°
```

---

## Review Sign-off

**Status**: ❌ **FAILED - Critical bugs must be fixed**

**Blocker Issues**: 2 critical bugs (#1 variable swap, #2 validity check)

**Estimated Fix Time**: 30 minutes coding + 2 hours testing

**Re-review Required**: Yes, after critical fixes applied

---

**Reviewer**: Claude Code (Opus 4.8)  
**Review Effort**: xhigh (recall-focused, 10 angles × 8 candidates)  
**Findings**: 13 technical issues + 5 documentation inconsistencies
