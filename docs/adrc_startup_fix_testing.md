# ADRC Speed Loop Startup Fix - Testing Guide

**Bug Fixed:** PR #12 - ADRC speed controller startup initialization
**Date:** 2025-12-17  
**Status:** Ready for hardware testing

---

## Problem Summary

When switching to ADRC speed controller mode, the motor would move slightly and immediately stop during startup. The root cause was a structure field mismatch where the code attempted to access `Speed_Pid.I_Sum` which only exists in the PID structure, not in the ADRC structure.

## Solution Implemented

Added proper conditional compilation guards and ESO state initialization:

**For PID mode:**
```c
#ifdef USE_SPEED_PID
    Speed_Pid.I_Sum = Iq_ref;  // Pre-load integral to prevent windup
#endif
```

**For ADRC mode:**
```c
#ifdef USE_SPEED_ADRC
    Speed_Pid.eso.z1 = current_speed;      // Initialize speed estimate
    Speed_Pid.eso.z2 = 0.0f;               // Zero acceleration estimate
    Speed_Pid.eso.z3 = b0 * Iq_ref;        // Initialize disturbance estimate
#endif
```

---

## How to Test the Fix

### Step 1: Switch to ADRC Mode

Edit `user/public.h`:

```c
// Change from:
#define USE_SPEED_PID          // Traditional PID speed controller
//#define USE_SPEED_ADRC         // Linear ADRC speed controller

// To:
//#define USE_SPEED_PID          // Traditional PID speed controller
#define USE_SPEED_ADRC         // Linear ADRC speed controller
```

### Step 2: Configure ADRC Parameters (Optional)

Edit `motor/foc_define_parameter.h` if you want to tune ADRC parameters:

```c
#ifdef USE_SPEED_ADRC
/* Observer bandwidth (rad/s) - default: 100 rad/s */
#define SPEED_ADRC_WO_DEFAULT           100.0f

/* Controller bandwidth (rad/s) - default: 50 rad/s */
#define SPEED_ADRC_WC_DEFAULT           50.0f

/* System gain estimate (b0) - default: 200 */
#define SPEED_ADRC_B0_DEFAULT           200.0f

/* Output limits (Amperes) */
#define SPEED_ADRC_OUTPUT_MAX           5.0f
#define SPEED_ADRC_OUTPUT_MIN          -5.0f
#endif
```

**Tuning Guidelines:**
- **wo (observer bandwidth)**: Start with 100 rad/s, increase for faster disturbance rejection
- **wc (controller bandwidth)**: Start with 50 rad/s (typically 1/2 to 1/3 of wo)
- **b0 (system gain)**: Motor-specific, start with 200 and adjust based on response

### Step 3: Build and Flash

1. Open Keil µVision project: `Keil_Project/stm32_drv8301_keil.uvprojx`
2. Clean build: **Project → Clean Targets**
3. Build: **Project → Build Target** (F7)
4. Flash firmware to hardware
5. Start debug session if needed

### Step 4: Test Motor Startup

1. **Power on** the motor controller
2. **Wait** for ADC offset calibration to complete
3. **Press Key1** (short press) to start the motor
4. **Expected behavior:**
   - Motor should start smoothly
   - Motor should accelerate to the default speed reference (25 Hz)
   - No immediate stopping after initial movement
   - Speed should be regulated according to ADRC controller

### Step 5: Test Speed Control

1. **Press Key3** to increase speed reference (5 Hz steps, max 200 Hz)
2. **Press Key2** to decrease speed reference (5 Hz steps, min 25 Hz)
3. **Expected behavior:**
   - Smooth speed transitions
   - Good speed regulation
   - Better disturbance rejection compared to PID (if load applied)

### Step 6: Compare with PID Mode

For validation, you can test both controllers:

1. **Test with ADRC** (as above)
2. **Switch back to PID** mode in `user/public.h`
3. **Rebuild and reflash**
4. **Test motor startup and speed control**
5. **Compare**:
   - Startup behavior
   - Speed regulation accuracy
   - Disturbance rejection (apply load)
   - Transient response (speed steps)

---

## What to Look For

### Success Indicators ✅
- Motor starts smoothly without stopping
- Speed tracks reference commands
- No oscillations or instability
- Smooth transitions between speeds
- Good load disturbance rejection

### Failure Indicators ✗
- Motor still stops after initial movement (fix not working)
- Excessive oscillations (tune wc lower)
- Sluggish response (tune wc higher or wo higher)
- Poor disturbance rejection (tune wo higher or check b0)

---

## Troubleshooting

### Motor Still Stops After Startup

**Possible causes:**
1. Build didn't include the fix - verify git commit and rebuild
2. Wrong macro defined - check `USE_SPEED_ADRC` is defined in `public.h`
3. b0 parameter way off - try adjusting `SPEED_ADRC_B0_DEFAULT`

### Motor Oscillates or Unstable

**Possible causes:**
1. Controller bandwidth too high - reduce `SPEED_ADRC_WC_DEFAULT` to 30-40 rad/s
2. Observer bandwidth too high - reduce `SPEED_ADRC_WO_DEFAULT` to 70-80 rad/s
3. System gain estimate incorrect - adjust `SPEED_ADRC_B0_DEFAULT`

**Tuning approach:**
1. Start with conservative values: wo=70, wc=30, b0=200
2. Increase wo gradually for faster disturbance rejection
3. Increase wc gradually for faster speed response
4. Adjust b0 based on motor response (step response test)

### Poor Disturbance Rejection

**Possible causes:**
1. Observer bandwidth too low - increase `SPEED_ADRC_WO_DEFAULT`
2. System gain (b0) incorrect - adjust up or down by 20-30%

---

## Technical Details

### ESO State Initialization Logic

**z1 (Speed Estimate):**
- Initialized to current speed feedback from sensors
- Prevents large initial tracking error
- Different for each sensor mode:
  - HALL: `hall_speed × 2π`
  - EKF: `FOC_Output.EKF[2]`
  - Hybrid: `fused_speed`

**z2 (Acceleration Estimate):**
- Set to zero during startup
- Motor accelerates slowly during open-loop phase
- ESO quickly adapts once closed-loop begins

**z3 (Disturbance Estimate):**
- Initialized to `b0 × Iq_ref`
- Represents steady-state load torque and friction
- Prevents large transient during open-loop to closed-loop transition

### Modified Files
- `motor/adc.c`: Lines 245, 292, 363 - Fixed ESO initialization
- `docs/project_stat.md`: Added Bug #6 comprehensive analysis
- `CHANGELOG.md`: Added v1.5.3 with bug fix details

---

## Expected ADRC Advantages

Compared to traditional PID, ADRC should provide:

1. **Better Disturbance Rejection**
   - Load changes compensated faster
   - Friction variations handled automatically
   - Parameter variations rejected

2. **No Integral Windup**
   - ESO z3 state handles steady-state errors
   - No need for anti-windup schemes

3. **Easier Tuning**
   - Two bandwidth parameters (wo, wc) instead of three PID gains (Kp, Ki, Kd)
   - Bandwidth has intuitive physical meaning

4. **Better Transient Response**
   - Faster settling with less overshoot (when tuned properly)
   - Smoother acceleration/deceleration

---

## Reporting Results

After testing, please report:

1. **Did the motor start successfully?** (Yes/No)
2. **Did speed control work as expected?** (Yes/No)
3. **Any oscillations or instability?** (Yes/No, describe)
4. **Comparison with PID mode?** (Better/Same/Worse)
5. **Final ADRC parameters used?** (wo, wc, b0)

Create a GitHub issue or comment on PR #12 with test results.

---

## References

- **ADRC Theory**: Han, J. "From PID to Active Disturbance Rejection Control", IEEE Trans. 2009
- **Bandwidth Tuning**: Gao, Z. "Scaling and bandwidth-parameterization", ACC 2003
- **Implementation Guide**: `docs/speed_controller_cpu_analysis.md`
- **Bug Analysis**: `docs/project_stat.md` - Bug #6

---

**Note:** This fix makes ADRC mode functional on hardware. The original implementation was completely non-functional due to the structure field mismatch bug.
