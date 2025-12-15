# PID Auto-Tuning Implementation Guide

## Overview

This document describes the automatic PID tuning feature for the current loop controllers in the BLDC motor FOC system. The auto-tuning algorithm uses identified motor parameters (resistance R and inductance L) to calculate optimal PI controller gains based on control theory and industry-standard practices.

**Last Updated:** 2025-12-15  
**Status:** Implemented and ready for testing

---

## Background

### Current State of PI Controllers

The FOC system uses cascaded PI control structure:
- **Outer Loop:** Speed controller (regulates motor speed)
- **Inner Loop:** Current controllers (regulate Id and Iq independently)

Current loop PI parameters (as of v1.3.0):
- **D-axis:** Kp = 2.199, Ki = 1282.8, Kb = 15.0
- **Q-axis:** Kp = 2.199, Ki = 1282.8, Kb = 15.0

These values appear to be manually tuned or theoretically calculated. There is no documentation on their derivation or whether they are optimal for the specific motor being used.

### Problem Statement

Manual PID tuning is:
1. **Time-consuming:** Requires iterative testing and adjustment
2. **Motor-specific:** Gains need retuning when motor changes
3. **Load-dependent:** Optimal gains may vary with operating conditions
4. **Expertise-required:** Requires deep understanding of control theory

### Solution: Model-Based Auto-Tuning

The implemented auto-tuning algorithm leverages the existing online parameter identification (RLS) to automatically calculate optimal PI gains based on identified motor electrical parameters.

---

## Algorithm Description

### Theoretical Foundation

The current loop transfer function for a BLDC motor is:

```
G(s) = 1 / (L*s + R)
```

Where:
- L = stator inductance (H)
- R = stator resistance (Ω)
- s = Laplace variable

For optimal PI control, the controller zero should cancel the motor pole:

```
Controller: C(s) = Kp * (1 + Ki/(Kp*s)) = Kp + Ki/s
```

### Gain Calculation Formulas

Based on TI InstaSPIN and control theory literature:

1. **Proportional Gain:**
   ```
   Kp = L × ωc
   ```

2. **Integral Gain:**
   ```
   Ki = R × ωc
   ```

3. **Anti-Windup Gain (empirical):**
   ```
   Kb = ωc / 10
   ```

Where:
- **ωc** = desired closed-loop bandwidth (rad/s)
- **fc** = bandwidth in Hz, where ωc = 2π × fc

### Bandwidth Selection

The bandwidth determines how fast the current loop responds:

- **Typical Range:** 1/10 to 1/5 of PWM frequency
- **For 10 kHz PWM:** 1000-2000 Hz bandwidth
- **Default:** 1000 Hz (conservative for stability)

Higher bandwidth:
- ✅ Faster current response
- ✅ Better transient performance
- ❌ More sensitive to noise
- ❌ Less stable margins

Lower bandwidth:
- ✅ More stable and robust
- ✅ Better noise rejection
- ❌ Slower response
- ❌ Reduced performance in transients

---

## State Machine Design

The auto-tuning process follows a state machine with the following states:

### 1. AUTOTUNE_IDLE
- **Description:** Auto-tune not active
- **Entry Condition:** Initial state or after completion/abort
- **Actions:** None
- **Exit Condition:** User triggers auto-tune start

### 2. AUTOTUNE_WAIT_STABLE
- **Description:** Waiting for motor to reach stable state
- **Entry Condition:** Auto-tune started
- **Actions:** 
  - Save original PI gains (backup)
  - Wait for specified time (default 2 seconds)
- **Exit Condition:** Stability timeout reached
- **Purpose:** Ensure motor is in steady state before parameter identification

### 3. AUTOTUNE_IDENTIFY_PARAMS
- **Description:** Monitoring parameter identification convergence
- **Entry Condition:** Stability wait complete
- **Actions:**
  - Read identified R and L from RLS algorithms
  - Validate parameters are in reasonable range
  - Check convergence (relative change < 5%)
- **Exit Conditions:**
  - **Success:** Parameters converged → move to calculate gains
  - **Failure:** Timeout or invalid parameters → abort
- **Convergence Criteria:**
  ```c
  |R_new - R_old| / R_old < 5%  AND
  |L_new - L_old| / L_old < 5%
  ```

### 4. AUTOTUNE_CALCULATE_GAINS
- **Description:** Calculate new PI gains
- **Entry Condition:** Parameters identified and converged
- **Actions:**
  - Apply model-based formulas
  - Apply safety margin (default 80%)
  - Clamp bandwidth to safe range
  - Sanity check calculated gains
- **Exit Condition:** Gains calculated → move to apply
- **Safety Checks:**
  - Minimum Kp: 0.1
  - Minimum Ki: 1.0
  - Minimum Kb: 1.0

### 5. AUTOTUNE_APPLY_GAINS
- **Description:** Apply calculated gains to controllers
- **Entry Condition:** Gains calculated
- **Actions:**
  - Update D-axis PI gains
  - Update Q-axis PI gains (same as D-axis for typical BLDC/SPMSM)
  - Wait for settling (500 ms)
- **Exit Condition:** Settling time complete → success

### 6. AUTOTUNE_COMPLETE
- **Description:** Auto-tune completed successfully
- **Actions:** New gains are active
- **Result:** System uses calculated optimal gains

### 7. AUTOTUNE_FAILED
- **Description:** Auto-tune failed
- **Entry Conditions:**
  - Timeout (> 10 seconds)
  - Invalid parameters
  - Convergence failure
- **Actions:** Restore original gains (rollback)
- **Result:** System reverts to pre-tune gains

---

## Motor Type Considerations

### D-axis and Q-axis Inductance

The current implementation assumes **Ld = Lq** (d-axis inductance equals q-axis inductance), which is appropriate for:

1. **BLDC Motors** (trapezoidal back-EMF)
   - Typically use surface-mounted permanent magnets
   - Ld ≈ Lq due to symmetric magnetic structure
   - **This is the primary target for this firmware**

2. **Surface-Mounted PMSM (SPMSM)** (sinusoidal back-EMF)
   - Magnets mounted on rotor surface
   - Non-salient pole design (no magnetic asymmetry)
   - Ld = Lq (equal inductance in both axes)

**Note for Interior PMSMs (IPMSM):**
- Interior permanent magnet motors have **Ld ≠ Lq** (typically Ld < Lq)
- Magnets embedded inside rotor create salient pole effect
- Different magnetic paths for d-axis (through magnets) and q-axis (through iron)
- For IPMSMs, this auto-tuning feature should be **disabled** and D/Q axes tuned independently

The auto-tune applies the same calculated gains to both D-axis and Q-axis controllers, which is correct for typical BLDC motors and SPMSM but may not be optimal for IPMSM applications.

---

## Configuration Parameters

All parameters are configurable in `motor/foc_define_parameter.h`:

```c
/* Enable feature */
#define ENABLE_PID_AUTOTUNE

/* Target bandwidth (Hz) */
#define PID_AUTOTUNE_TARGET_BANDWIDTH_HZ   1000.0f

/* Bandwidth limits (Hz) */
#define PID_AUTOTUNE_MIN_BANDWIDTH_HZ      500.0f
#define PID_AUTOTUNE_MAX_BANDWIDTH_HZ      2000.0f

/* Safety margin (0.5-0.9) */
#define PID_AUTOTUNE_SAFETY_MARGIN         0.8f

/* Convergence threshold (0.01-0.1) */
#define PID_AUTOTUNE_CONVERGENCE_THRESHOLD 0.05f

/* Timing (milliseconds) */
#define PID_AUTOTUNE_STABLE_TIME_MS        2000
#define PID_AUTOTUNE_MAX_TUNE_TIME_MS      10000
```

### Parameter Tuning Guide

| Parameter | Recommended Range | Effect |
|-----------|-------------------|--------|
| TARGET_BANDWIDTH_HZ | 800-1500 Hz | Higher = faster response, lower = more stable |
| SAFETY_MARGIN | 0.7-0.9 | Lower = more conservative gains |
| CONVERGENCE_THRESHOLD | 0.03-0.10 | Lower = tighter convergence, longer wait |
| STABLE_TIME_MS | 1000-3000 ms | System stabilization time |

---

## Usage

### Enabling the Feature

1. Uncomment in `motor/foc_define_parameter.h`:
   ```c
   #define ENABLE_PID_AUTOTUNE
   ```

2. Build and flash firmware

3. Feature is now compiled in and ready to use

### Starting Auto-Tune

**Prerequisites:**
- Motor must be running
- Low to medium speed (< 100 rad/s recommended)
- No load or light load
- System in steady state (not accelerating/decelerating)

**Method 1: Via Code**
```c
#ifdef ENABLE_PID_AUTOTUNE
if (foc_start_pid_autotune()) {
    /* Auto-tune started successfully */
} else {
    /* Conditions not met */
}
#endif
```

**Method 2: Via PC Tool (if integrated)**
- Send start command via UART
- Monitor telemetry for progress

### Monitoring Progress

Check auto-tune state:
```c
uint8_t state = foc_get_autotune_status();
/* 0=IDLE, 1=WAIT_STABLE, 2=IDENTIFY, 3=CALCULATE, 4=APPLY, 5=COMPLETE, 6=FAILED */
```

Monitor telemetry variables (if configured):
- `float_test6`: Kp_calculated
- `float_test7`: Ki_calculated  
- `float_test8`: Kb_calculated

### Stopping Auto-Tune

Abort and restore original gains:
```c
foc_stop_pid_autotune();
```

### Retrieving Results

After successful completion:
```c
real32_T Kp, Ki, Kb;
if (pid_autotune_get_gains(&Kp, &Ki, &Kb)) {
    /* Gains are valid and can be saved to non-volatile memory */
    printf("Optimal gains: Kp=%.3f, Ki=%.3f, Kb=%.3f\n", Kp, Ki, Kb);
}
```

---

## Safety Considerations

### Pre-Conditions
- ✅ Motor running at stable low speed
- ✅ Light or no load
- ✅ Current loop already functional
- ✅ Parameter identification converged

### During Auto-Tune
- ⚠️ Gains are being updated in real-time
- ⚠️ Brief transients may occur
- ⚠️ Monitor motor for abnormal behavior

### Abort Conditions
The auto-tune will automatically abort if:
- Total time exceeds 10 seconds
- Parameters outside valid range
- Parameters fail to converge
- System becomes unstable

### Recovery
- Original gains are restored on abort
- No permanent changes until completion
- Safe to retry after fixing conditions

---

## Testing and Validation

### Test Procedure

1. **Baseline Test**
   - Run motor with current manual gains
   - Record step response (Iq reference → Iq actual)
   - Measure rise time, overshoot, settling time

2. **Run Auto-Tune**
   - Start motor at 50-100 rad/s
   - No load or light load
   - Trigger auto-tune via code or PC tool
   - Wait for completion (5-10 seconds)

3. **Compare Performance**
   - Run same step response test
   - Compare metrics with baseline
   - Expected improvements:
     - Faster rise time if bandwidth increased
     - Similar or better overshoot
     - Stable steady-state tracking

4. **Load Test**
   - Apply load variations
   - Verify stable operation
   - Check current tracking accuracy

5. **Save Gains**
   - If performance improved, save to configuration
   - Update `D_PI_P`, `D_PI_I`, `D_PI_KB` in code
   - Disable auto-tune for production

### Expected Results

For typical BLDC motor parameters:
- **R ≈ 0.1-1.0 Ω**
- **L ≈ 0.0005-0.005 H**
- **Bandwidth = 1000 Hz**

Calculated gains:
```
Kp = L × 2π × 1000 × 0.8
   ≈ 0.0005 × 6283 × 0.8
   ≈ 2.5

Ki = R × 2π × 1000 × 0.8
   ≈ 0.1 × 6283 × 0.8
   ≈ 502

Kb = (2π × 1000 × 0.8) / 10
   ≈ 502
```

Compare with current gains:
- Current Kp: 2.199 ✓ (similar)
- Current Ki: 1282.8 ⚠️ (higher, may be for different motor)

### Validation Metrics

| Metric | Baseline | After Auto-Tune | Target |
|--------|----------|-----------------|--------|
| Rise Time (ms) | ? | ? | < 2 ms |
| Overshoot (%) | ? | ? | < 10% |
| Settling Time (ms) | ? | ? | < 5 ms |
| Steady-State Error | ? | ? | < 1% |
| Stability | ? | ? | No oscillations |

---

## Troubleshooting

### Auto-Tune Fails to Start
**Symptoms:** `foc_start_pid_autotune()` returns 0

**Possible Causes:**
- Auto-tune already running
- Feature not enabled (#define missing)

**Solutions:**
- Check if auto-tune already active
- Verify `ENABLE_PID_AUTOTUNE` is defined

### Parameters Don't Converge
**Symptoms:** State stuck in AUTOTUNE_IDENTIFY_PARAMS

**Possible Causes:**
- Motor speed varying
- Load changing
- Parameter identification not working
- Convergence threshold too tight

**Solutions:**
- Ensure constant speed
- Remove load variations
- Check RLS algorithms are active
- Increase CONVERGENCE_THRESHOLD

### Invalid Parameters Detected
**Symptoms:** Auto-tune fails with parameter validation error

**Possible Causes:**
- RLS not converged yet
- Extreme motor parameters
- Measurement noise

**Solutions:**
- Wait longer before triggering auto-tune
- Check motor parameter ranges in code
- Verify current sensing is accurate

### Calculated Gains Too High/Low
**Symptoms:** Motor unstable or too slow after auto-tune

**Possible Causes:**
- Bandwidth setting inappropriate
- Safety margin too aggressive
- Motor parameters incorrect

**Solutions:**
- Adjust TARGET_BANDWIDTH_HZ
- Increase SAFETY_MARGIN (more conservative)
- Verify identified R and L are correct

### Motor Becomes Unstable
**Symptoms:** Oscillations, current spikes after applying gains

**Immediate Action:**
- Call `foc_stop_pid_autotune()` to restore original gains
- Reduce motor speed
- Emergency stop if necessary

**Root Causes:**
- Bandwidth too high
- Anti-windup gain incorrect
- Current sensing issues

**Solutions:**
- Reduce MAX_BANDWIDTH_HZ
- Increase SAFETY_MARGIN
- Verify current sensors and offsets

---

## Integration with Existing Code

### Files Modified
1. `motor/foc_define_parameter.h` - Added feature flags and config
2. `motor/foc_algorithm.h` - Added API declarations
3. `motor/foc_algorithm.c` - Added auto-tune calls and wrappers
4. `user/pc_communication_init.c` - Added telemetry (optional)

### Files Added
1. `motor/pid_autotune.h` - Auto-tune API and structures
2. `motor/pid_autotune.c` - Auto-tune implementation
3. `docs/pid_autotune_implementation.md` - This document

### Build System
- Add `motor/pid_autotune.c` to Keil project if using auto-tune
- No changes needed if feature disabled (conditional compilation)

---

## References

### Industry Standards
1. **Texas Instruments InstaSPIN-FOC**
   - Automatic motor identification and PI tuning
   - Model-based gain calculation
   - [TI MotorWare Documentation](https://www.ti.com/tool/MOTORWARE)

2. **SimpleFOC Library**
   - Open-source FOC implementation
   - Parameter estimation and tuning guidelines
   - [SimpleFOC Docs](https://docs.simplefoc.com/)

3. **STMicroelectronics Motor Control SDK**
   - Commercial FOC implementation
   - Auto-tuning examples

### Academic References
1. Bose, B.K., "Modern Power Electronics and AC Drives", 2001
   - Chapter on FOC and PI controller design

2. Åström, K.J. and Murray, R.M., "Feedback Systems", 2008
   - PI controller theory and tuning methods

3. Texas Instruments Application Notes:
   - SPRA588: "Field Orientated Control of 3-Phase AC-Motors"
   - SPRT647: "InstaSPIN-FOC Technology"

### Formula Derivation
The gain formulas come from pole-zero cancellation and bandwidth specification:

1. Motor transfer function: `G(s) = 1/(Ls + R) = (1/L) / (s + R/L)`
2. Motor pole: `p = -R/L`
3. PI controller: `C(s) = Kp(1 + Ki/(Kp·s))`
4. Controller zero: `z = -Ki/Kp`
5. For pole-zero cancellation: `Ki/Kp = R/L`, thus `Ki = Kp·R/L`
6. Closed-loop bandwidth: `ωc = Kp/L`, thus `Kp = L·ωc`
7. Combining: `Ki = (L·ωc)·(R/L) = R·ωc`

---

## Future Enhancements

### Possible Improvements
1. **Adaptive Tuning**
   - Continuously adjust gains based on operating conditions
   - Load-dependent gain scheduling

2. **Relay Feedback Method**
   - Alternative auto-tune using controlled oscillations
   - Ziegler-Nichols tuning rules

3. **Multi-Axis Independence**
   - Separate tuning for D and Q axes
   - Useful for IPMSM (different L_d and L_q)

4. **Speed Loop Auto-Tune**
   - Extend to outer speed control loop
   - Coordinate with current loop bandwidth

5. **Persistent Storage**
   - Save tuned gains to EEPROM/Flash
   - Auto-load on startup

6. **GUI Integration**
   - PC tool interface for auto-tune
   - Real-time visualization
   - Parameter export/import

---

## Conclusion

The PID auto-tuning feature provides:
- ✅ Automatic optimization of current loop PI gains
- ✅ Model-based approach using identified motor parameters
- ✅ Safe operation with abort and rollback
- ✅ Minimal code overhead when disabled
- ✅ Industry-standard algorithms (TI, SimpleFOC)

**Status:** Implementation complete and ready for hardware testing.

**Next Steps:**
1. Enable feature in configuration
2. Run initial tests with known motor
3. Validate calculated gains
4. Compare with manual tuning
5. Document optimal settings per motor type

---

**Document Version:** 1.0.0  
**Last Updated:** 2025-12-15  
**Author:** GitHub Copilot Agent  
**Review Status:** Ready for review
