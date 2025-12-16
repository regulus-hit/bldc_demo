# Copilot / AI agent instructions for this repository

Purpose: give an AI coding agent immediate, actionable context to be productive editing this BLDC motor firmware.

- **For Start**: Always read [docs/development_checklist.md](docs/development_checklist.md) first for project overview, recent changes, and style notes.

- **Big picture**: STM32F446 BLDC FOC firmware (Keil µVision). Organized by functional layers:
  - **Hardware init**: [user/hardware/board_config.c](user/hardware/board_config.c), [user/hardware/system_stm32f4xx.c](user/hardware/system_stm32f4xx.c)
  - **Application entry**: [user/app/main.c](user/app/main.c), [user/app/exti.c](user/app/exti.c)
  - **User interface**: [user/interface/](user/interface) (OLED, UART, UpperComputer)
  - **Motor drivers**: [motor/drivers/drv8301.c](motor/drivers/drv8301.c), [motor/drivers/adc.c](motor/drivers/adc.c), [driver_soft/](driver_soft)
  - **FOC control**: [motor/control/foc_algorithm.c](motor/control/foc_algorithm.c), [motor/control/speed_pid.c](motor/control/speed_pid.c), [motor/control/speed_adrc.c](motor/control/speed_adrc.c)
  - **Sensors**: [motor/sensors/hall_sensor.c](motor/sensors/hall_sensor.c), [motor/sensors/hybrid_observer.c](motor/sensors/hybrid_observer.c)
  - **Parameter ID**: [motor/identification/](motor/identification) (EKF, RLS wrappers)
  - **Enhancements**: [motor/enhancement/pid_autotune.c](motor/enhancement/pid_autotune.c)

- **Build & debug**: Build with Keil project [Keil_Project/stm32_drv8301_keil.uvprojx](Keil_Project/stm32_drv8301_keil.uvprojx) (includes CMSIS + StdPeriph paths). Debug/flash via Keil + J-Link configs in [Keil_Project/](Keil_Project) and [settings/](settings). Artifacts under [Debug/](Debug) and [Release/](Release).

- **Runtime model**: FOC runs in ADC ISR (`ADC_IRQHandler` → `adc_c_adc_sub()` → FOC) at ~10 kHz; keep ISR code bounded, no malloc, no blocking, no heavy math. TIM/ADC/PWM setup in [user/hardware/board_config.c](user/hardware/board_config.c); ISRs wired in [user/app/main.c](user/app/main.c#L1-L120).

- **Feature flags & modes**: Modes selected in [motor/foc_define_parameter.h](motor/foc_define_parameter.h): `HALL_FOC_SELECT`, `SENSORLESS_FOC_SELECT`, `HYBRID_HALL_EKF_SELECT`. Enhancements are macro-guarded here: dead-time comp, field-weakening, Vbus filtering, Hall interpolation, hybrid fusion weights. Keep new features behind `#define` with sane defaults.

- **Hall interpolation (docs-driven)**: See [docs/hall_interpolation_impl_guide.md](docs/hall_interpolation_impl_guide.md#L1-L180) and [docs/hall_interpolation_impl.md](docs/hall_interpolation_impl.md#L1-L180). In HALL_FOC_SELECT, use `ENABLE_HALL_INTERPOLATION` (+ optional misalignment correction) in foc_define_parameter.h; interpolation runs per control tick, Hall edge ISR supplies timestamps; expect ~10% CPU. Low speed falls back to raw Hall; misalignment limited ±20°.

- **Observer fusion**: Hybrid Hall+EKF logic in [motor/sensors/hybrid_observer.c](motor/sensors/hybrid_observer.c#L1-L220); design notes in [docs/hybrid_observer_implementation.md](docs/hybrid_observer_implementation.md#L1-L120). Fusion weights and thresholds live in foc_define_parameter.h. Keep angles normalized, use shortest-path diffs, avoid expensive fmod in ISR (bounded while/conditionals instead).

- **FOC core patterns**: PI current/speed controllers use anti-windup and must keep P/I gains independent (see PI fix in [docs/project_stat.md](docs/project_stat.md#L40-L140)). SVPWM over-modulation: scale Tx/Ty together when `Tx+Ty > Tpwm` (guarded in motor/control/foc_algorithm.c). Preserve intermediates before reuse to avoid overwrite bugs (Kalman gain/covariance fixes documented in project_stat.md).

- **Telemetry & tuning**: PC tool variables registered in [user/interface/UpperComputer.c](user/interface/UpperComputer.c); add new telemetry via `float_data_upload_init`/`int_data_upload_init`, add tunables via `float_data_download_init`, and wire references (e.g., `speed_ref_data_download_init`).

- **Generated code caution**: Files named `*_wrapper.c` and `rtw_*` headers are Simulink/RTW outputs (e.g., EKF, parameter ID). In this project it is acceptable to edit wrappers directly when math correctness is reviewed carefully—preserve interfaces and verify algebra (see Kalman gain/covariance fixes in docs/project_stat.md). Avoid changing struct layouts unless coordinated.

- **Style & safety expectations** (from recent fixes/docs):
  - Embedded C best practices: static storage, no dynamic allocation, deterministic loops, short ISR paths, normalized angles via bounded ops.
  - Preserve originals before multi-use math to prevent aliasing bugs in matrix ops (see Kalman gain/covariance fixes in project_stat.md).
  - Keep comments concise C-style block headers above non-trivial routines; follow existing naming (PascalCase structs, mixed-case globals, `_sub` ISR helpers).
  - When changing DRV8301 enable/fault handling or PWM timing, review gate safety and hall timing in [user/hardware/board_config.c](user/hardware/board_config.c#L1-L200) and [user/app/main.c](user/app/main.c#L1-L200).

- **Docs discipline**: When fixing bugs or adding features, update [docs/project_stat.md](docs/project_stat.md) and related guides (hall interpolation, hybrid observer, enhancements). Keep usage notes and tunables in docs plus foc_define_parameter.h comments.

- **Ask maintainers when**: altering Simulink/RTW-generated behavior, changing gate-driver safety, or adding new build/CI paths (no top-level Make/CMake). For J-Link/flash automation details, confirm desired flow.

If you want expansions (Keil build steps, J-Link script example, safety checklist), ask to add them here.
