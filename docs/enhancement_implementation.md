# FOC Algorithm Enhancement Implementation

## Overview

This document describes the implementation of three advanced FOC (Field-Oriented Control) enhancements based on recommendations in the FOC Control Loop Analysis Report.

## Implemented Features

### 1. Dead-Time Compensation

**Purpose:** Compensates for voltage error introduced by gate driver dead-time, improving low-speed torque and reducing current distortion.

**Implementation Location:** `motor/foc_algorithm.c` (after Inverse Park Transform, before SVPWM)

**Algorithm:**
```c
V_comp = sign(I) * DEADTIME_COMPENSATION_GAIN
```

**Configuration:**
```c
#define ENABLE_DEADTIME_COMPENSATION  // Comment out to disable
#define DEADTIME_COMPENSATION_GAIN   0.02f  // Adjustable compensation factor
```

**Key Features:**
- Applies compensation based on current direction (sign function)
- Dead zone of ±0.01A to avoid noise at zero crossing
- Compensation voltage added to alpha-beta voltage commands
- Improves torque linearity at low speeds

---

### 2. Field-Weakening Control

**Purpose:** Extends motor speed range beyond base speed by injecting negative Id current to weaken rotor flux.

**Implementation Location:** `motor/adc.c` (in both HALL_FOC_SELECT and SENSORLESS_FOC_SELECT modes)

**Algorithm:**
```c
if (|speed| > BASE_SPEED) {
    Id_fw = -K_fw * (|speed| - BASE_SPEED)
    Id_fw = max(Id_fw, MAX_NEG_ID)  // Limit to prevent demagnetization
}
```

**Configuration:**
```c
#define ENABLE_FIELD_WEAKENING           // Comment out to disable
#define FIELD_WEAKENING_BASE_SPEED   150.0f  // rad/s electrical
#define FIELD_WEAKENING_MAX_NEG_ID   -2.0f   // Maximum negative Id (A)
#define FIELD_WEAKENING_GAIN         0.01f   // Gain factor
```

**Key Features:**
- Activates only in closed-loop speed control mode
- Works with both Hall sensor and sensorless (EKF) modes
- Progressive Id reduction proportional to speed above base
- Protection against excessive flux weakening (demagnetization limit)
- Typical speed range extension: 30-50%

---

### 3. Bus Voltage Filtering

**Purpose:** Reduces SVPWM calculation errors from DC link capacitor ripple, improving voltage utilization and control accuracy.

**Implementation Location:** `motor/adc.c` (in motor_run function)

**Algorithm:**
```c
Vbus_filtered = α * Vbus_new + (1-α) * Vbus_old
```

**Configuration:**
```c
#define ENABLE_VBUS_FILTERING        // Comment out to disable
#define VBUS_FILTER_ALPHA        0.1f  // Filter coefficient (0 to 1)
```

**Key Features:**
- First-order IIR low-pass filter
- Filter coefficient α = 0.1 provides ~160Hz cutoff at 10kHz sampling
- Initialized with first measurement to avoid startup transient
- Lower α = more filtering but slower response
- Reduces errors in SVPWM duty cycle calculations

---

## Usage Instructions

### Enabling/Disabling Features

All features are controlled via preprocessor macros in `motor/foc_define_parameter.h`:

1. **To enable a feature:** Ensure the corresponding `#define` is uncommented
2. **To disable a feature:** Comment out the corresponding `#define`

Example:
```c
// All features enabled (default)
#define ENABLE_DEADTIME_COMPENSATION
#define ENABLE_FIELD_WEAKENING
#define ENABLE_VBUS_FILTERING

// Only dead-time compensation enabled
#define ENABLE_DEADTIME_COMPENSATION
//#define ENABLE_FIELD_WEAKENING      // Disabled
//#define ENABLE_VBUS_FILTERING        // Disabled
```

### Parameter Tuning

Each feature has tunable parameters that can be adjusted based on specific motor and application requirements:

**Dead-Time Compensation:**
- `DEADTIME_COMPENSATION_GAIN`: Start with 0.02f, increase if low-speed torque ripple persists
- Monitor current THD (Total Harmonic Distortion) to verify improvement

**Field-Weakening:**
- `FIELD_WEAKENING_BASE_SPEED`: Set to rated speed of motor
- `FIELD_WEAKENING_MAX_NEG_ID`: Typically -10% to -30% of rated current
- `FIELD_WEAKENING_GAIN`: Tune for smooth speed transition, typical range 0.005-0.02

**Bus Voltage Filtering:**
- `VBUS_FILTER_ALPHA`: Lower for more filtering (0.05-0.2 typical range)
- Trade-off between noise rejection and transient response

---

## Testing and Validation

### Recommended Test Procedures

1. **Dead-Time Compensation:**
   - Measure phase currents at low speed (< 10% rated)
   - Compare THD with and without compensation
   - Verify torque ripple reduction
   - Check zero-crossing current smoothness

2. **Field-Weakening:**
   - Gradually increase speed reference above base speed
   - Monitor Id current (should become negative above base speed)
   - Verify smooth acceleration without current spikes
   - Measure maximum achievable speed with/without field-weakening

3. **Bus Voltage Filtering:**
   - Observe Vbus measurement noise with oscilloscope
   - Compare SVPWM duty cycle stability
   - Verify no degradation in dynamic response
   - Test under varying load conditions

### Safety Considerations

- **Field-Weakening:** Excessive negative Id can cause permanent magnet demagnetization
- **Dead-Time Compensation:** Over-compensation can cause shoot-through at high current
- **Bus Voltage Filtering:** Excessive filtering may mask actual voltage transients

Always test incrementally with conservative parameter values.

---

## Code Structure

### Modified Files

1. **motor/foc_define_parameter.h**
   - Added feature enable/disable macros
   - Added configuration parameters for each feature
   - Comprehensive documentation for each parameter

2. **motor/foc_algorithm.c**
   - Implemented dead-time compensation in `foc_algorithm_step()`
   - Inserted after Inverse Park Transform, before SVPWM
   - Guards with `#ifdef ENABLE_DEADTIME_COMPENSATION`

3. **motor/adc.c**
   - Implemented bus voltage filtering in `motor_run()`
   - Implemented field-weakening control for both sensor modes
   - Guards with appropriate `#ifdef` directives

### Design Principles

- **Minimal Changes:** All enhancements are additive, no existing code modified
- **Conditional Compilation:** Each feature independently controlled via `#ifdef`
- **Backward Compatibility:** Disabling all features restores original behavior
- **Clear Documentation:** Every parameter and logic block documented
- **Safety First:** Built-in limits and protections for each feature

---

## References

Based on recommendations in:
- **Document:** `docs/FOC_Control_Loop_Analysis.md`
- **Section:** "Recommendations for Future Enhancement"
- **Priority:** HIGH (items 1-3)

Industry standards referenced:
- STMicroelectronics Motor Control SDK
- Texas Instruments MotorWare
- SimpleFOC Library

---

## Conclusion

All three high-priority FOC enhancements have been successfully implemented with:
- ✅ Proper conditional compilation guards (`#ifdef`)
- ✅ Comprehensive parameter documentation
- ✅ Industry-standard algorithms
- ✅ Backward compatibility maintained
- ✅ Ready for hardware testing and validation

Each feature can be independently enabled/disabled for debugging and incremental deployment.
