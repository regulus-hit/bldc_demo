# BLDC Motor Field-Oriented Control (FOC) Demo

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-STM32F4-green.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
[![Build](https://img.shields.io/badge/IDE-Keil%20µVision-orange.svg)](https://www.keil.com/)

A production-ready BLDC (Brushless DC) motor control system implementing Field-Oriented Control (FOC) on STM32F4 microcontroller with DRV8301 gate driver. Features multiple sensor modes, advanced control algorithms, and comprehensive enhancements for optimal motor performance.

## Table of Contents

- [Features](#features)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Getting Started](#getting-started)
- [Configuration](#configuration)
- [Sensor Modes](#sensor-modes)
- [Advanced Features](#advanced-features)
- [Documentation](#documentation)
- [Project Structure](#project-structure)
- [Building](#building)
- [Debugging](#debugging)
- [Contributing](#contributing)
- [License](#license)
- [References](#references)

## Features

### Core FOC Implementation
- **Field-Oriented Control (FOC)** algorithm at 10 kHz control frequency
  - Clarke Transform (abc → αβ)
  - Park Transform (αβ → dq)
  - Current PI Controllers with anti-windup
  - Inverse Park Transform (dq → αβ)
  - Space Vector PWM (SVPWM) with over-modulation handling
- **Three Sensor Modes** (selectable via configuration):
  - Pure Hall sensor (60° resolution, simple and reliable)
  - Pure sensorless EKF observer (smooth, high resolution)
  - **Hybrid Hall+EKF** (complementary fusion, best of both)
- **Hall Sensor Interpolation** for improved position resolution in Hall mode
- **Extended Kalman Filter (EKF)** for sensorless rotor position and speed estimation
- **Online Parameter Identification** using Recursive Least Squares (RLS)
  - Motor resistance (Rs)
  - Motor inductance (Ls)
  - Flux linkage estimation

### Advanced Control Enhancements
- **Dead-Time Compensation**: Compensates voltage error from gate driver dead-time
  - Improves low-speed torque linearity
  - Reduces current harmonic distortion
- **Field-Weakening Control**: Extends speed range by 30-50% beyond base speed
  - Progressive negative Id injection
  - Demagnetization protection
- **Bus Voltage Filtering**: Low-pass filters DC bus voltage for improved SVPWM accuracy
- **ADC Offset Validation**: Automatic calibration retry with bounds checking

### Code Quality & Reliability
- ✅ **Zero dynamic memory allocation** (embedded-friendly)
- ✅ **Bounded execution time** (deterministic, real-time safe)
- ✅ **5 critical bugs fixed** (EKF, PI controller, SVPWM)
- ✅ **Industry-standard algorithms** (verified against ST, TI, SimpleFOC)
- ✅ **Comprehensive documentation** with Doxygen comments
- ✅ **Backward compatible** with independent feature flags

## System Architecture

### Hardware Architecture
```
STM32F4 MCU
    ├── TIM1/8: Center-aligned PWM @ 10 kHz
    ├── ADC: 12-bit current/voltage sampling (injected channels)
    ├── TIM2/3/4: Hall sensor edge capture
    ├── USART2: PC communication for tuning
    └── SPI/I2C: OLED display

DRV8301 Gate Driver
    ├── 3-phase gate driver with dead-time insertion
    ├── Current shunt amplifiers (gain configurable)
    ├── Overcurrent protection
    └── Fault detection
```

### Software Architecture
```
ISR (10 kHz)
    ├── ADC sampling → Current measurement
    ├── FOC algorithm execution
    │   ├── Clarke Transform
    │   ├── Park Transform
    │   ├── Current PI Controllers
    │   ├── Inverse Park Transform
    │   └── SVPWM duty cycle generation
    ├── State Observer (Hall/EKF/Hybrid)
    └── PWM update

Main Loop
    ├── Speed control (cascaded with current control)
    ├── Parameter identification (RLS)
    ├── Communication handling
    └── OLED display update
```

### Control Loop Flow
```
PWM ISR (10kHz) → ADC Sample → motor_run() → foc_algorithm_step() → PWM Update
     ↓                                               ↓
  100µs cycle                       EKF Observer + Parameter ID + Enhancements
```

## Hardware Requirements

- **Microcontroller**: STM32F4 series (tested on STM32F407)
- **Gate Driver**: DRV8301 or compatible
- **Motor**: 3-phase BLDC/PMSM with Hall sensors (optional for sensorless mode)
- **Power Supply**: Appropriate DC voltage for motor (typically 12V-48V)
- **Debug Interface**: J-Link or ST-Link for programming and debugging
- **Optional**: OLED display for status monitoring

## Software Requirements

- **IDE**: Keil µVision 5 or later
- **Compiler**: ARM Compiler 5 (ARMCC) or ARM Compiler 6
- **Libraries**: 
  - CMSIS (included in `Libraries/CMSIS/`)
  - STM32F4xx Standard Peripheral Library (included in `Libraries/STM32F4xx_StdPeriph_Driver/`)
- **Debugger**: J-Link software or ST-Link utilities

## Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/regulus-hit/bldc_demo.git
cd bldc_demo
```

### 2. Hardware Setup
1. Connect STM32F4 board to DRV8301 gate driver
2. Connect 3-phase motor to gate driver outputs
3. Connect Hall sensors (if using Hall or Hybrid mode)
4. Connect power supply
5. Connect J-Link/ST-Link debugger

### 3. Software Configuration
See [Configuration](#configuration) section below.

### 4. Build and Flash
1. Open `Keil_Project/stm32_drv8301_keil.uvprojx` in Keil µVision
2. Select build configuration (Debug or Release)
3. Build project (F7)
4. Flash to target (F8)

## Configuration

### Sensor Mode Selection
Edit `motor/foc_define_parameter.h` to select ONE sensor mode:

```c
// Pure Hall sensor mode (simple, 60° resolution)
#define HALL_FOC_SELECT

// Pure sensorless EKF mode (smooth, no sensors required)
//#define SENSORLESS_FOC_SELECT

// Hybrid Hall+EKF mode (best performance, combines both)
//#define HYBRID_HALL_EKF_SELECT
```

### Motor Parameters
Configure motor electrical parameters in `motor/foc_define_parameter.h`:

```c
#define RS_PARAMETER     0.1f      // Stator resistance (Ohms)
#define LS_PARAMETER     0.0005f   // Stator inductance (H)
#define FLUX_PARAMETER   0.005f    // Flux linkage (Wb)
```

### Control Parameters
```c
#define MOTOR_STARTUP_CURRENT   1.0f   // Startup current (A)
#define SPEED_LOOP_CLOSE_RAD_S  50.0f  // Speed loop engagement (rad/s)
```

### Feature Flags
Enable/disable advanced features in `motor/foc_define_parameter.h`:

```c
// Dead-time compensation
#define ENABLE_DEADTIME_COMPENSATION
#define DEADTIME_COMPENSATION_GAIN  0.02f

// Field-weakening control
#define ENABLE_FIELD_WEAKENING
#define FIELD_WEAKENING_BASE_SPEED   150.0f  // rad/s
#define FIELD_WEAKENING_MAX_NEG_ID    -2.0f  // A

// Bus voltage filtering
#define ENABLE_VBUS_FILTERING
#define VBUS_FILTER_ALPHA  0.1f

// Hall interpolation (for HALL_FOC_SELECT mode)
#define ENABLE_HALL_INTERPOLATION
#define HALL_INTERPOLATION_MIN_SPEED  20.0f  // rad/s
```

## Sensor Modes

### 1. Pure Hall Sensor Mode (`HALL_FOC_SELECT`)
- Uses Hall sensors for rotor position feedback
- 60° electrical angle resolution
- Simple and reliable
- Optional: Enable `ENABLE_HALL_INTERPOLATION` for higher resolution
  - Velocity-based linear interpolation between Hall edges
  - Automatic misalignment correction
  - Smooth torque control at medium/high speeds

**Use Case**: Cost-sensitive applications, reliable position feedback

### 2. Pure Sensorless EKF Mode (`SENSORLESS_FOC_SELECT`)
- Extended Kalman Filter estimates position and speed from current measurements
- Smooth, continuous position estimation
- No Hall sensors required
- May require startup sequence for initial position detection

**Use Case**: Applications without Hall sensors, cost optimization

### 3. Hybrid Hall+EKF Mode (`HYBRID_HALL_EKF_SELECT`)
- Complementary filtering fuses Hall sensor and EKF estimates
- Best of both worlds: Hall's absolute reference + EKF's smoothness
- Configurable fusion weights
- Automatic divergence detection and recovery
- Eliminates Hall's 60° quantization noise

**Use Case**: High-performance applications requiring optimal position/speed accuracy

**Configuration Parameters:**
```c
#define HYBRID_HALL_POSITION_WEIGHT    0.3f   // Position fusion weight
#define HYBRID_HALL_SPEED_WEIGHT       0.2f   // Speed fusion weight
#define HYBRID_HALL_MIN_SPEED          10.0f  // Min speed for fusion (rad/s)
#define HYBRID_HALL_MAX_POSITION_ERROR 1.047f // Divergence threshold (rad)
```

## Advanced Features

### Dead-Time Compensation
Compensates for voltage error introduced by gate driver dead-time:
- Applied between Inverse Park Transform and SVPWM
- Improves low-speed torque linearity
- Reduces current harmonic distortion
- Tunable compensation gain

### Field-Weakening Control
Extends motor speed range beyond base speed:
- Progressive negative Id injection when speed > base speed
- 30-50% speed range extension typical
- Demagnetization protection (Id limited to -2.0A default)
- Smooth transition into field-weakening region

### Bus Voltage Filtering
First-order IIR low-pass filter for DC bus voltage:
- Reduces SVPWM calculation errors from DC link ripple
- Filter coefficient α = 0.1 (160 Hz cutoff @ 10 kHz sampling)
- Improves voltage utilization

### Hall Sensor Interpolation
Available in `HALL_FOC_SELECT` mode:
- Velocity-based linear interpolation between 60° Hall edges
- Automatic misalignment correction for sensor installation errors
- Threshold: Activates above 60 RPM (20 rad/s)
- ~10-15 µs computational overhead per cycle
- No EKF required

## Documentation

Comprehensive documentation available in `docs/` directory:

- **[docs/project_stat.md](docs/project_stat.md)**: Project status, bug fixes, and analysis
  - 5 critical bugs fixed with detailed explanations
  - Mathematical verification against textbooks
  - Control loop timing analysis
  - Comparison with industry standards (ST, TI, SimpleFOC)

- **[docs/enhancement_implementation.md](docs/enhancement_implementation.md)**: FOC enhancement guide
  - Dead-time compensation implementation
  - Field-weakening control details
  - Bus voltage filtering
  - Configuration and tuning guidelines

- **[docs/hybrid_observer_implementation.md](docs/hybrid_observer_implementation.md)**: Hybrid observer design
  - Complementary filtering algorithm
  - Fusion weight selection
  - Divergence detection logic
  - Architecture and best practices

- **[docs/hall_interpolation_impl_guide.md](docs/hall_interpolation_impl_guide.md)**: Hall interpolation guide
  - Velocity-based interpolation algorithm
  - Misalignment correction details
  - Configuration and tuning
  - Performance characteristics

- **[docs/hall_interpolation_impl.md](docs/hall_interpolation_impl.md)**: Hall interpolation technical details

- **[CHANGELOG.md](CHANGELOG.md)**: Complete version history with all changes

## Project Structure

```
bldc_demo/
├── motor/                          # Motor control core
│   ├── foc_algorithm.c/h          # FOC algorithm (Clarke, Park, SVPWM)
│   ├── adc.c/h                    # ADC sampling and motor_run() entry
│   ├── hall_sensor.c/h            # Hall sensor handling + interpolation
│   ├── hybrid_observer.c/h        # Hybrid Hall+EKF observer
│   ├── stm32_ekf_wrapper.c        # Extended Kalman Filter
│   ├── speed_pid.c/h              # Speed PI controller
│   ├── drv8301.c/h                # DRV8301 gate driver interface
│   ├── low_task.c/h               # Low-frequency tasks
│   ├── foc_define_parameter.h     # Configuration parameters
│   ├── R_flux_identification_wrapper.c  # Rs & flux RLS
│   └── L_identification_wrapper.c       # Ls RLS
├── user/                           # User application code
│   ├── main.c/h                   # Main entry and ISR definitions
│   ├── board_config.c/h           # Hardware initialization (TIM, ADC, PWM)
│   ├── exti.c/h                   # External interrupts (Hall edges)
│   ├── oled.c/h                   # OLED display driver
│   ├── oled_display.c/h           # OLED display logic
│   ├── USART2.c/h                 # UART communication
│   ├── pc_communication/          # PC tool protocol
│   ├── pc_communication_init.c/h  # Telemetry registration
│   ├── stm32f4xx_it.c/h           # System interrupt handlers
│   └── system_stm32f4xx.c         # System clock configuration
├── driver_soft/                    # Software driver components
├── Libraries/                      # CMSIS and StdPeriph libraries
│   ├── CMSIS/                     # ARM CMSIS headers
│   └── STM32F4xx_StdPeriph_Driver/  # STM32 peripheral drivers
├── Keil_Project/                   # Keil µVision project files
│   └── stm32_drv8301_keil.uvprojx # Main project file
├── settings/                       # Debug settings (J-Link, ST-Link)
├── Debug/                          # Debug build artifacts
├── Release/                        # Release build artifacts
├── docs/                           # Documentation
│   ├── project_stat.md            # Project status and bug fixes
│   ├── enhancement_implementation.md  # Enhancement guide
│   ├── hybrid_observer_implementation.md  # Hybrid observer docs
│   ├── hall_interpolation_impl_guide.md   # Hall interpolation guide
│   └── hall_interpolation_impl.md         # Hall interpolation details
├── .github/
│   └── copilot-instructions.md    # AI agent context
├── README.md                       # This file
└── CHANGELOG.md                    # Version history
```

## Building

### Using Keil µVision

1. Open project: `Keil_Project/stm32_drv8301_keil.uvprojx`
2. Select target configuration:
   - **Debug**: Full symbols, optimized for debugging (-O0)
   - **Release**: Optimized for size/speed (-O2/-O3)
3. Build: `Project → Build Target` (F7)
4. Output files generated in `Debug/` or `Release/` directory

### Build Configurations

- **Debug**: Enables debug symbols, no optimization, JTAG/SWD enabled
- **Release**: Optimized build, minimal debug info, smaller code size

## Debugging

### Using J-Link

1. Connect J-Link to STM32F4 SWD/JTAG interface
2. Configure J-Link in Keil: `Options → Debug → Use J-Link`
3. Start debug session: `Debug → Start/Stop Debug Session` (Ctrl+F5)
4. Use Keil debugger features:
   - Breakpoints
   - Variable watch windows
   - Memory browser
   - Peripheral register viewer

### Using ST-Link

1. Connect ST-Link to STM32F4 SWD interface
2. Configure ST-Link in Keil: `Options → Debug → Use ST-Link`
3. Follow same debug procedures as J-Link

### Live Tuning via PC Tool

- UART communication on USART2 enables real-time parameter tuning
- Variables exposed in `user/pc_communication_init.c`
- Use compatible PC tool for monitoring and tuning (protocol in `user/UpperComputer.lib`)

## Contributing

Contributions welcome! Please follow embedded C best practices:

- **No dynamic memory allocation** (malloc/free)
- **Bounded execution time** (no unbounded loops)
- **Deterministic behavior** (critical for real-time control)
- **Proper angle wrapping** (0-2π boundaries)
- **Comprehensive comments** (especially for math-heavy code)
- **Update documentation** when changing algorithms

Before submitting:
1. Test on hardware
2. Update `docs/project_stat.md` with changes
3. Add entries to `CHANGELOG.md`
4. Ensure backward compatibility

## License

This project is released under the MIT License. See `LICENSE` file for details.

## References

### Industry Standards & Application Notes
- Texas Instruments SPRA588: "Field Orientated Control of 3-Phase AC-Motors"
- STMicroelectronics AN1078: "Sensorless BLDC Motor Control and BEMF Sampling Methods"
- Microchip AN1162: "Sensored BLDC Motor Control Using dsPIC30F2010"
- Microchip AN4413: "BLDC Motor Control with Hall Sensors"

### Textbooks
- Dan Simon, "Optimal State Estimation", Wiley-Interscience, 2006
- Grewal & Andrews, "Kalman Filtering: Theory and Practice Using MATLAB", Wiley, 2008
- Bimal K. Bose, "Modern Power Electronics and AC Drives", Prentice Hall, 2001
- Åström & Murray, "Feedback Systems: An Introduction for Scientists and Engineers", Princeton, 2008

### Open Source Projects
- [SimpleFOC Library](https://simplefoc.com/) - Open-source FOC implementation
- [STMicroelectronics Motor Control SDK](https://www.st.com/en/embedded-software/x-cube-mcsdk.html)
- [Texas Instruments MotorWare](https://www.ti.com/tool/MOTORWARE)

---

**Developed by**: ZHANG Yangyu (Regulus Zhang)  
**Repository**: https://github.com/regulus-hit/bldc_demo  
**Last Updated**: 2025-12-15
