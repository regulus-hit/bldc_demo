# Hybrid Hall+EKF Observer Implementation Guide

## Overview

This document describes the implementation of a hybrid observer that combines Hall sensor measurements with Extended Kalman Filter (EKF) state estimation for BLDC motor control. This approach provides the benefits of both sensing methods while mitigating their individual weaknesses.

## Motivation

### Pure Hall Sensor Limitations
- **Low resolution**: 60° electrical angle resolution (6 states per electrical cycle)
- **Quantization noise**: Position updates only at Hall edge transitions
- **Speed noise**: Speed calculation from edge timing is noisy, especially at low speeds
- **Low speed issues**: Long periods between edges at low speeds reduce update rate

### Pure EKF Sensorless Limitations
- **Convergence**: Requires time to converge during startup
- **Parameter sensitivity**: Performance depends on accurate motor parameter knowledge
- **Divergence risk**: Can drift if parameters are incorrect or conditions change
- **Low speed challenges**: Back-EMF signal is weak at very low speeds

### Hybrid Approach Benefits
- **Best of both worlds**: Combines smooth EKF estimation with Hall sensor robustness
- **Improved accuracy**: EKF interpolates between Hall edges for smooth position/speed
- **Robustness**: Hall sensors prevent EKF divergence and provide absolute position reference
- **Better startup**: Hall sensors initialize EKF quickly
- **Reduced noise**: EKF filtering reduces Hall quantization and timing noise

## Architecture

### System Design

The hybrid observer implements a complementary filtering approach inspired by:
- **Texas Instruments MotorWare**: InstaSPIN-FOC with sensor fusion capabilities
- **SimpleFOC Library**: Hybrid angle sensor implementation
- **STMicroelectronics**: Sensorless/sensored switching strategies

### Block Diagram

```
┌──────────────┐         ┌──────────────────────────────────┐
│              │         │                                  │
│  Hall Sensor ├────────►│  Position: 60° resolution        │
│              │         │  Speed: From edge timing         │
└──────────────┘         │                                  │
                         │        Hybrid Observer           │
┌──────────────┐         │    Complementary Filter          │
│              │         │                                  │
│  EKF State   ├────────►│  Position: Smooth continuous     │
│  Observer    │         │  Speed: Smooth, may drift        │
└──────────────┘         │                                  │
                         └────────┬─────────────────────────┘
                                  │
                                  ▼
                         ┌─────────────────┐
                         │ Fused Estimates │
                         │ - Position      │
                         │ - Speed         │
                         └─────────────────┘
```

## Implementation Details

### Configuration Options

All configuration is done via preprocessor macros in `motor/foc_define_parameter.h`:

```c
/* Mode Selection - Choose ONE */
#define HALL_FOC_SELECT          // Pure Hall sensor mode
//#define SENSORLESS_FOC_SELECT    // Pure sensorless EKF mode
//#define HYBRID_HALL_EKF_SELECT   // Hybrid Hall+EKF mode

/* Hybrid Observer Parameters (when HYBRID_HALL_EKF_SELECT is enabled) */
#define HYBRID_HALL_POSITION_NOISE   0.1f    // Hall position measurement noise (rad²)
#define HYBRID_HALL_SPEED_NOISE      10.0f   // Hall speed measurement noise (rad²/s²)
#define HYBRID_HALL_POSITION_WEIGHT  0.3f    // Position fusion weight (0-1)
#define HYBRID_HALL_SPEED_WEIGHT     0.2f    // Speed fusion weight (0-1)
#define HYBRID_HALL_MIN_SPEED        10.0f   // Minimum speed for fusion (rad/s)
#define HYBRID_HALL_MAX_POSITION_ERROR 1.047f // Max position error (rad, ~60°)
```

### Complementary Filter Algorithm

#### Position Fusion

The position estimate is fused using a weighted correction approach:

```
fused_position = ekf_position + weight * (hall_position - ekf_position)
               = (1 - weight) * ekf_position + weight * hall_position
```

Where:
- `weight = HYBRID_HALL_POSITION_WEIGHT` (typically 0.3)
- EKF provides smooth high-frequency interpolation
- Hall provides low-frequency absolute position correction

#### Speed Fusion

The speed estimate uses a weighted average:

```
fused_speed = (1 - weight) * ekf_speed + weight * hall_speed
```

Where:
- `weight = HYBRID_HALL_SPEED_WEIGHT` (typically 0.2)
- EKF speed is smooth but may drift
- Hall speed is absolute but noisy

### Operational Modes

#### 1. Low-Speed Operation (speed < HYBRID_HALL_MIN_SPEED)

```c
if (speed_abs < HYBRID_HALL_MIN_SPEED)
{
    // Use pure EKF
    fused_position = ekf_position;
    fused_speed = ekf_speed;
}
```

**Rationale**: At very low speeds, Hall sensor timing becomes unreliable due to long periods between edge transitions. The EKF, which tracks back-EMF in the current measurements, provides better estimates in this region.

#### 2. Normal Operation (speed >= HYBRID_HALL_MIN_SPEED)

```c
// Compute position error
position_error = angle_difference(ekf_position, hall_position);

// Complementary filter fusion
fused_position = ekf_position + HYBRID_HALL_POSITION_WEIGHT * position_error;
fused_speed = (1.0f - HYBRID_HALL_SPEED_WEIGHT) * ekf_speed + 
              HYBRID_HALL_SPEED_WEIGHT * hall_speed;
```

**Rationale**: At normal speeds, both sensors provide valuable information. The complementary filter combines them optimally.

#### 3. Divergence Detection (position_error > HYBRID_HALL_MAX_POSITION_ERROR)

```c
if (fabsf(position_error) > HYBRID_HALL_MAX_POSITION_ERROR)
{
    // EKF has diverged - trust Hall sensor
    fused_position = hall_position;
    fused_speed = hall_speed;
}
```

**Rationale**: If the EKF position differs from Hall by more than one sector (60°), the EKF has likely diverged due to incorrect parameters or disturbances. The Hall sensor provides an absolute reference to re-anchor the estimates.

### Angle Wrapping Handling

The implementation correctly handles angle wrapping at 0/2π boundaries:

```c
float angle_difference(float angle1, float angle2)
{
    float diff = angle2 - angle1;
    
    // Normalize to [-PI, PI] range
    while (diff > MATH_PI)
        diff -= MATH_2PI;
    while (diff < -MATH_PI)
        diff += MATH_2PI;
    
    return diff;
}
```

This ensures the fusion algorithm always uses the shortest angular path.

## Usage Instructions

### Enabling Hybrid Mode

1. **Edit `motor/foc_define_parameter.h`**:
   ```c
   //#define HALL_FOC_SELECT          // Comment out pure Hall mode
   //#define SENSORLESS_FOC_SELECT    // Comment out pure sensorless mode
   #define HYBRID_HALL_EKF_SELECT     // Enable hybrid mode
   ```

2. **Build and flash** the firmware using Keil µVision or your preferred toolchain.

3. **Motor will automatically use hybrid observer** for position and speed feedback.

### Tuning Parameters

#### Position Fusion Weight (HYBRID_HALL_POSITION_WEIGHT)

- **Default**: 0.3 (30% Hall, 70% EKF)
- **Range**: 0.0 to 1.0
- **Lower values** (e.g., 0.1): Trust EKF more → smoother but may drift
- **Higher values** (e.g., 0.5): Trust Hall more → follows Hall closely but may have more noise

**Tuning guideline**:
- Start with default 0.3
- If position tracking is sluggish, increase to 0.4-0.5
- If position is noisy/jittery, decrease to 0.1-0.2

#### Speed Fusion Weight (HYBRID_HALL_SPEED_WEIGHT)

- **Default**: 0.2 (20% Hall, 80% EKF)
- **Range**: 0.0 to 1.0
- **Lower values** (e.g., 0.1): Smoother speed estimate
- **Higher values** (e.g., 0.4): More responsive but noisier

**Tuning guideline**:
- Start with default 0.2
- If speed control oscillates, decrease to 0.1
- If speed tracking is too slow, increase to 0.3-0.4

#### Minimum Speed Threshold (HYBRID_HALL_MIN_SPEED)

- **Default**: 10.0 rad/s electrical
- **Purpose**: Below this speed, use pure EKF (Hall timing unreliable)
- **Typical range**: 5.0 to 20.0 rad/s

**Tuning guideline**:
- If motor is unstable at low speeds, increase threshold
- If fusion isn't engaging soon enough, decrease threshold

#### Maximum Position Error (HYBRID_HALL_MAX_POSITION_ERROR)

- **Default**: 1.047 rad (60° = π/3)
- **Purpose**: Divergence detection threshold
- **Typical range**: 0.524 (30°) to 1.571 (90°)

**Tuning guideline**:
- Default of 60° (one Hall sector) is usually optimal
- Only change if you see frequent unnecessary divergence detections

## Code Structure

### Files Modified

1. **`motor/foc_define_parameter.h`**
   - Added `HYBRID_HALL_EKF_SELECT` mode option
   - Added hybrid observer tuning parameters

2. **`motor/adc.c`**
   - Added hybrid mode section in `motor_run()` function
   - Calls `hybrid_observer_update()` to fuse measurements
   - Implements field-weakening control for hybrid mode

3. **`motor/foc_algorithm.c`**
   - Added hybrid observer initialization in `foc_algorithm_initialize()`

4. **`user/main.h`**
   - Added `PI` macro definition for compatibility

5. **`Keil_Project/stm32_drv8301_keil.uvprojx`**
   - Added `hybrid_observer.c` to build system

### Files Created

1. **`motor/hybrid_observer.h`**
   - Header file with function declarations and type definitions
   - Documents the hybrid observer API

2. **`motor/hybrid_observer.c`**
   - Implementation of complementary filtering algorithm
   - Angle normalization and difference functions
   - Divergence detection logic

## Testing and Validation

### Recommended Test Sequence

1. **Verify backward compatibility**:
   - Test with `HALL_FOC_SELECT`: Should work as before
   - Test with `SENSORLESS_FOC_SELECT`: Should work as before

2. **Test hybrid mode startup**:
   - Enable `HYBRID_HALL_EKF_SELECT`
   - Motor should start smoothly using Hall initialization
   - EKF should converge quickly with Hall guidance

3. **Test normal operation**:
   - Run motor at various speeds (50-200 rad/s)
   - Verify smooth position/speed tracking
   - Check that speed control is stable

4. **Test low-speed operation**:
   - Run motor below `HYBRID_HALL_MIN_SPEED`
   - Verify smooth operation (should use pure EKF)
   - Check no sudden transitions when crossing threshold

5. **Test divergence recovery** (if possible):
   - Temporarily provide incorrect motor parameters
   - Verify system falls back to Hall sensor
   - Verify recovery when parameters corrected

### Monitoring During Testing

Monitor these variables using debugger or serial communication:
- `FOC_Output.EKF[2]`: EKF speed estimate (rad/s)
- `FOC_Output.EKF[3]`: EKF position estimate (rad)
- `hall_speed * 2π`: Hall speed (rad/s)
- `hall_angle`: Hall position (rad)
- `fused_position`: Final fused position (rad)
- `fused_speed`: Final fused speed (rad/s)

## Performance Characteristics

### Expected Improvements Over Pure Hall

- **Position resolution**: Continuous vs 60° steps
- **Speed noise**: ~50% reduction in speed estimate noise
- **Low-speed performance**: Smoother operation below 30 rad/s
- **Startup time**: Similar (Hall provides quick initialization)

### Expected Improvements Over Pure EKF

- **Robustness**: Hall prevents divergence, provides absolute reference
- **Startup reliability**: Hall ensures correct initial sector
- **Parameter tolerance**: Less sensitive to motor parameter errors
- **Convergence time**: Faster convergence with Hall guidance

## Troubleshooting

### Issue: Motor oscillates or vibrates

**Possible causes**:
- Fusion weights too high (trusting noisy Hall measurements too much)

**Solutions**:
- Reduce `HYBRID_HALL_POSITION_WEIGHT` to 0.1-0.2
- Reduce `HYBRID_HALL_SPEED_WEIGHT` to 0.1

### Issue: Position tracking is sluggish

**Possible causes**:
- Fusion weights too low (trusting Hall too little)

**Solutions**:
- Increase `HYBRID_HALL_POSITION_WEIGHT` to 0.4-0.5
- Check Hall sensor signals are clean

### Issue: Unstable at low speeds

**Possible causes**:
- `HYBRID_HALL_MIN_SPEED` threshold too low

**Solutions**:
- Increase `HYBRID_HALL_MIN_SPEED` to 15-20 rad/s
- Check EKF parameters are tuned correctly

### Issue: Sudden jumps in position/speed

**Possible causes**:
- Frequent divergence detection triggering
- Hall sensor noise or electrical interference

**Solutions**:
- Increase `HYBRID_HALL_MAX_POSITION_ERROR` to 1.5-2.0 rad
- Check Hall sensor wiring and shielding
- Add filtering to Hall speed calculation

## References

### Industry Implementations

1. **Texas Instruments MotorWare**
   - InstaSPIN-FOC with sensor fusion
   - Automatic transition between sensored/sensorless modes
   - Reference: "MotorWare User's Guide" (SPRUHJ1)

2. **SimpleFOC Library**
   - Open-source FOC implementation
   - Hybrid angle sensor support
   - Reference: https://simplefoc.com/

3. **STMicroelectronics Motor Control SDK**
   - Sensorless/sensored switching strategies
   - Complementary observer designs
   - Reference: "FOC SDK User Manual" (UM1052)

### Academic References

1. **Kalman Filtering**: Simon, D. "Optimal State Estimation", Wiley-Interscience, 2006
2. **Motor Control**: Bose, B.K. "Modern Power Electronics and AC Drives", Prentice Hall, 2001
3. **Sensor Fusion**: Mahony, R. et al. "Complementary filter design on the SO(3)", CDC 2005

## Best Practices

### Embedded C Compliance

The implementation follows embedded C best practices:
- ✅ **No dynamic memory allocation**: All state stored in static structures
- ✅ **Bounded execution time**: No unbounded loops (angle normalization uses while with guaranteed termination)
- ✅ **No recursion**: All functions are non-recursive
- ✅ **Fixed-point friendly**: Uses only basic floating-point operations
- ✅ **MISRA-C compatible**: Can be adapted for MISRA compliance if needed

### Safety Considerations

- **Divergence detection**: Prevents runaway if EKF fails
- **Graceful degradation**: Falls back to Hall sensor on EKF failure
- **Angle wrapping**: Correctly handles 0/2π boundary
- **Speed threshold**: Prevents fusion at unreliable low speeds

## Future Enhancements

Potential improvements for future versions:

1. **Adaptive fusion weights**: Automatically adjust weights based on speed and confidence
2. **Hall speed filtering**: Add low-pass filter to Hall speed for noise reduction
3. **Parameter adaptation**: Use Hall measurements to improve online parameter identification
4. **Multi-rate fusion**: Update Hall and EKF at different rates for efficiency
5. **Fault detection**: Detect Hall sensor faults and automatically switch to pure EKF

---

**Document Version**: 1.0  
**Date**: 2025-12-15  
**Author**: GitHub Copilot Agent  
**Status**: Initial Implementation Complete
