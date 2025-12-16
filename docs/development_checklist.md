# BLDC Motor Control Development Checklist

**Purpose:** Quick reference guide for developers working on this BLDC FOC motor control project  
**Last Updated:** 2025-12-16  
**Version:** 1.0.0

---

## 🎯 Project Overview Quick Facts

- **Platform**: STM32F446 @ 180 MHz with DRV8301 gate driver
- **Control Frequency**: 10 kHz (100 µs per cycle)
- **Build System**: Keil µVision 5
- **Current Status**: ✅ **STABLE** - All features implemented and tested
- **Latest Version**: v1.5.0 (PRs #1-10 integrated)

---

## ✅ Pre-Development Checklist

### Before Starting Any Work

- [ ] Read `.github/copilot-instructions.md` for project context
- [ ] Review `docs/project_stat.md` for current status and recent changes
- [ ] Check `CHANGELOG.md` for version history
- [ ] Review relevant feature documentation in `docs/` directory
- [ ] Understand the sensor mode you'll be working with (Hall/EKF/Hybrid)

### Understanding the Code Structure

- [ ] **Hardware Init**: `user/board_config.c`, `user/system_stm32f4xx.c`
- [ ] **Entry Point**: `user/main.c` (ISR definitions)
- [ ] **Gate Driver**: `motor/drv8301.c`, `driver_soft/`
- [ ] **FOC Core**: `motor/foc_algorithm.c`, `motor/adc.c`
- [ ] **Sensors**: `motor/hall_sensor.c`, `motor/hybrid_observer.c`
- [ ] **Configuration**: `motor/foc_define_parameter.h`

---

## 🔧 Configuration Checklist

### Motor Parameters (in `motor/foc_define_parameter.h`)

- [ ] Verify motor electrical parameters (Rs, Ls, flux linkage)
- [ ] Set correct startup current and speed thresholds
- [ ] Configure PWM frequency and dead-time

### Sensor Mode Selection (Choose ONE)

- [ ] **HALL_FOC_SELECT** - Pure Hall sensor (simple, 60° resolution)
  - [ ] Optional: Enable `ENABLE_HALL_INTERPOLATION` for higher resolution
  - [ ] Optional: Enable `ENABLE_HALL_MISALIGNMENT_CORRECTION`
- [ ] **SENSORLESS_FOC_SELECT** - Pure EKF (smooth, no sensors)
- [ ] **HYBRID_HALL_EKF_SELECT** - Fusion mode (best performance)
  - [ ] Tune fusion weights (HYBRID_HALL_POSITION_WEIGHT, HYBRID_HALL_SPEED_WEIGHT)

### Speed Controller Selection (Choose ONE)

- [ ] **USE_SPEED_PID** - Traditional PID (default, proven)
- [ ] **USE_SPEED_ADRC** - Linear ADRC (advanced, better disturbance rejection)
  - [ ] Tune observer bandwidth (SPEED_ADRC_WO_DEFAULT)
  - [ ] Tune controller bandwidth (SPEED_ADRC_WC_DEFAULT)
  - [ ] Estimate system gain (SPEED_ADRC_B0_DEFAULT)

### Feature Flags

- [ ] **ENABLE_DEADTIME_COMPENSATION** - Voltage error correction
- [ ] **ENABLE_FIELD_WEAKENING** - Speed range extension
- [ ] **ENABLE_VBUS_FILTERING** - DC bus voltage filtering
- [ ] **ENABLE_PID_AUTOTUNE** - Automatic current loop tuning

---

## 🏗️ Building & Testing Checklist

### Build Process

- [ ] Open `Keil_Project/stm32_drv8301_keil.uvprojx`
- [ ] Select appropriate target (Debug/Release)
- [ ] Clean build: Project → Clean Targets
- [ ] Build: Project → Build Target (F7)
- [ ] Check for warnings (should be minimal or zero)
- [ ] Verify binary size fits in flash

### Pre-Flash Hardware Checks

- [ ] Power supply voltage correct for motor
- [ ] Motor phases connected properly (U, V, W)
- [ ] Hall sensors connected (if using Hall mode)
- [ ] Current sense resistors in place
- [ ] Gate driver fault pin monitored
- [ ] Emergency stop accessible

### Initial Testing Sequence

1. [ ] Flash firmware and start debug session
2. [ ] Verify ADC offset calibration completes (check telemetry)
3. [ ] Apply very low speed reference (~10 rad/s)
4. [ ] Monitor phase currents (should be smooth sinusoids)
5. [ ] Check position/speed estimation accuracy
6. [ ] Gradually increase speed reference
7. [ ] Test load disturbance rejection
8. [ ] Verify field-weakening activation (if enabled)

---

## 🐛 Debugging Checklist

### Common Issues and Quick Checks

#### Motor Won't Start
- [ ] Check ADC offset calibration (should be near 2048 for 12-bit ADC)
- [ ] Verify startup current is sufficient (`MOTOR_STARTUP_CURRENT`)
- [ ] Check gate driver enable signal
- [ ] Verify PWM outputs are active
- [ ] Check for fault flags from DRV8301

#### Unstable Position/Speed
- [ ] Check Hall sensor connections and signals
- [ ] Verify EKF covariance matrix (should not be NaN/Inf)
- [ ] Review PI controller gains (may need tuning)
- [ ] Check for electrical noise on ADC inputs
- [ ] If using Hybrid mode, verify fusion weights

#### High Current / Overcurrent Trips
- [ ] Verify motor parameters (Rs, Ls, flux) are correct
- [ ] Check current PI controller limits
- [ ] Review dead-time compensation settings
- [ ] Check for mechanical binding in motor
- [ ] Verify bus voltage is stable

#### Poor Speed Regulation
- [ ] Tune speed controller gains (PID or ADRC)
- [ ] Check speed loop closure threshold
- [ ] Verify load torque is within motor capability
- [ ] Review current loop performance first
- [ ] Consider enabling PID auto-tune

---

## 📝 Code Modification Guidelines

### Embedded C Best Practices

- [ ] **NO dynamic memory allocation** (malloc/free forbidden)
- [ ] **NO unbounded loops** (all loops must have fixed max iterations)
- [ ] **NO recursion** (stack overflow risk)
- [ ] **Keep ISR execution short** (<10 µs for non-FOC ISRs)
- [ ] **Use static storage** for all variables
- [ ] **Normalize angles** using bounded operations (not fmod)

### Safety Requirements

- [ ] All new features behind `#ifdef` guards
- [ ] Default configuration must be safe (conservative gains)
- [ ] Add parameter validation for user inputs
- [ ] Implement fail-safe behavior on errors
- [ ] Update fault handling if touching motor control

### Code Style

- [ ] Use existing naming conventions (PascalCase structs, mixed globals)
- [ ] Add Doxygen comments for new functions
- [ ] Follow existing indentation (tabs, not spaces)
- [ ] Use stdint types (uint8_t, uint16_t, etc.)
- [ ] Uppercase hex numbers (0x00FF, not 0x00ff)

---

## 📚 Documentation Update Checklist

### When Adding Features

- [ ] Update `docs/project_stat.md` with implementation details
- [ ] Add entry to `CHANGELOG.md` (follow Keep a Changelog format)
- [ ] Create feature-specific documentation in `docs/` if complex
- [ ] Update `README.md` configuration section if needed
- [ ] Update `.github/copilot-instructions.md` if changing architecture

### When Fixing Bugs

- [ ] Document the bug in `docs/project_stat.md`
- [ ] Explain the root cause and fix
- [ ] Add test case or verification procedure
- [ ] Update `CHANGELOG.md` in appropriate version section

---

## 🔍 Code Review Checklist

### Before Submitting Changes

- [ ] All modifications are minimal and surgical
- [ ] No unrelated code changes or reformatting
- [ ] Build completes with no errors
- [ ] No new compiler warnings introduced
- [ ] Feature can be disabled via `#ifdef` (if applicable)
- [ ] Backward compatible with existing configurations
- [ ] Documentation updated appropriately

### Mathematical Correctness

- [ ] Algorithm matches textbook formulation
- [ ] No variable aliasing bugs (intermediate values preserved)
- [ ] Matrix operations use correct dimensions
- [ ] Angle wrapping handles 0/2π boundaries
- [ ] No overflow/underflow in fixed-point arithmetic

### Real-Time Safety

- [ ] ISR execution time measured and within budget
- [ ] No blocking operations in ISRs
- [ ] Critical sections are minimal
- [ ] Timer/counter overflow handled correctly

---

## 🎓 Learning Resources

### Essential Reading (in order)

1. `docs/project_stat.md` - Project status and bug fixes
2. `.github/copilot-instructions.md` - Development guidelines
3. `docs/enhancement_implementation.md` - FOC enhancements
4. `CHANGELOG.md` - Complete version history

### Feature-Specific Docs

- **Hybrid Observer**: `docs/hybrid_observer_implementation.md`
- **Hall Interpolation**: `docs/hall_interpolation_impl_guide.md`
- **PID Auto-Tune**: `docs/pid_autotune_implementation.md`
- **ADRC Controller**: `docs/speed_controller_cpu_analysis.md`

### External References

- Texas Instruments SPRA588: Field Orientated Control
- STMicroelectronics AN1078: Sensorless BLDC Control
- Dan Simon: "Optimal State Estimation" (for EKF)
- Bimal K. Bose: "Modern Power Electronics and AC Drives" (for FOC)
- Han & Gao: "Active Disturbance Rejection Control" papers

---

## ⚠️ Critical Reminders

### Things That Will Break the System

1. **DON'T** select multiple sensor modes simultaneously
2. **DON'T** select both USE_SPEED_PID and USE_SPEED_ADRC
3. **DON'T** modify gate driver timing without reviewing safety
4. **DON'T** change Hall ISR timing without testing thoroughly
5. **DON'T** add floating-point operations to ISRs without measuring timing
6. **DON'T** remove anti-windup from PI controllers
7. **DON'T** modify SVPWM over-modulation scaling logic
8. **DON'T** skip ADC offset calibration validation

### Known Limitations

- EKF may diverge at very low speeds (<5 rad/s) - use hybrid mode if needed
- Hall interpolation requires speed >60 RPM to be effective
- Field-weakening reduces available torque (this is expected physics)
- ADRC requires good system gain (b0) estimate for optimal performance
- PID auto-tune requires stable operation and converged parameter ID

---

## 🚀 Future Enhancement Ideas

### Not Yet Implemented (from project_stat.md)

- [ ] Startup current profiling (configurable ramp rate)
- [ ] Flux observer backup (fallback for EKF divergence)
- [ ] Advanced diagnostics (parameter drift, open-phase, stall detection)
- [ ] Efficiency optimization (MTPA control, loss minimization)

### Contribution Guidelines

If implementing new features:
1. Follow all checklists above
2. Test thoroughly on hardware
3. Create comprehensive documentation
4. Ensure backward compatibility
5. Submit with clear commit messages

---

## 📞 Contact & Support

- **Repository**: https://github.com/regulus-hit/bldc_demo
- **Developer**: ZHANG Yangyu (Regulus Zhang)
- **License**: MIT

---

**Remember**: This is a real-time motor control system. Safety first, always test incrementally, and keep emergency stop accessible during development!
