/**********************************
      
**********************************/
#ifndef __FOC_DEFINE_PARAMETER_H__
#define __FOC_DEFINE_PARAMETER_H__

#define MOTOR_STARTUP_CURRENT   1.0f   //电机启动电流，根据自己实际负载设置 1.0
#define SPEED_LOOP_CLOSE_RAD_S  50.0f  //速度环切入闭环的速度  单位: rad/s

/*******************************************************************************
 * FOC Sensor Mode Selection
 * Select ONE of the following three modes:
 * - HALL_FOC_SELECT: Pure Hall sensor feedback (no EKF)
 * - SENSORLESS_FOC_SELECT: Pure sensorless EKF observer (no Hall sensors)
 * - HYBRID_HALL_EKF_SELECT: Hybrid observer using Hall sensors to enhance EKF
 ******************************************************************************/
#define HALL_FOC_SELECT          // Pure Hall sensor mode
//#define SENSORLESS_FOC_SELECT    // Pure sensorless EKF mode
//#define HYBRID_HALL_EKF_SELECT   // Hybrid Hall+EKF mode (Hall enhances EKF)


////配套信浓电机参数配置（电阻，电感，磁链）
//#define RS_PARAMETER     1.0f           //电阻2.5
//#define LS_PARAMETER     0.002f          //电感0.001  002
//#define FLUX_PARAMETER   0.03f        //磁链 0.02050f  03
////#define FLUX_PARAMETER   0.0538f        //磁链 0.02050f  03

////原版57BL55S06电机参数配置（电阻，电感，磁链）
//#define RS_PARAMETER     0.59f           //电阻
//#define LS_PARAMETER     0.001f          //电感
//#define FLUX_PARAMETER   0.01150f        //磁链

#define RS_PARAMETER     0.1f           //电阻
#define LS_PARAMETER     0.0005f          //电感
#define FLUX_PARAMETER   0.005000f        //磁链

/*******************************************************************************
 * Speed Controller Selection
 * Select ONE of the following speed controllers:
 * - USE_SPEED_PID: Traditional PID controller (default, proven)
 * - USE_SPEED_ADRC: Linear Active Disturbance Rejection Control (advanced)
 * 
 * ADRC provides better disturbance rejection and faster response but requires
 * more computational resources. PID is simpler and well-tested.
 ******************************************************************************/
#define USE_SPEED_PID          // Traditional PID speed controller
//#define USE_SPEED_ADRC         // Linear ADRC speed controller

/*******************************************************************************
 * Advanced FOC Enhancement Features
 * Uncomment to enable each feature independently for debugging and testing
 ******************************************************************************/

/* Dead-Time Compensation: Compensates voltage error from gate driver dead-time
 * Improves low-speed torque and reduces current distortion */
//#define ENABLE_DEADTIME_COMPENSATION
#undef ENABLE_DEADTIME_COMPENSATION

/* Field-Weakening Control: Extends speed range beyond base speed
 * Enables operation at higher speeds by injecting negative Id current */
#define ENABLE_FIELD_WEAKENING
//#undef ENABLE_FIELD_WEAKENING

/* Bus Voltage Filtering: Low-pass filters DC bus voltage measurement
 * Reduces SVPWM errors from DC link ripple, improves voltage utilization */
#define ENABLE_VBUS_FILTERING
//#undef ENABLE_VBUS_FILTERING

/* Hall Sensor Position Interpolation: Interpolates position between Hall edges
 * Provides higher resolution position feedback for HALL_FOC_SELECT mode
 * Only active when HALL_FOC_SELECT mode is enabled */
//#define ENABLE_HALL_INTERPOLATION
#undef ENABLE_HALL_INTERPOLATION

/* PID Auto-Tuning: Automatically optimizes current loop PI controller gains
 * Uses identified motor parameters (R, L) to calculate optimal Kp/Ki values
 * Based on TI InstaSPIN and model-based tuning approach */
//#define ENABLE_PID_AUTOTUNE
#undef ENABLE_PID_AUTOTUNE

/*******************************************************************************
 * PID Auto-Tuning Parameters
 ******************************************************************************/
#ifdef ENABLE_PID_AUTOTUNE
/* Target bandwidth for current loop (Hz)
 * Typical: 1/10 to 1/5 of PWM frequency
 * For 10 kHz PWM: 1000-2000 Hz is typical
 * Higher bandwidth = faster response but less stable */
#define PID_AUTOTUNE_TARGET_BANDWIDTH_HZ   1000.0f

/* Minimum safe bandwidth (Hz)
 * Sets lower limit to prevent too-slow response */
#define PID_AUTOTUNE_MIN_BANDWIDTH_HZ      500.0f

/* Maximum safe bandwidth (Hz)
 * Sets upper limit to prevent instability
 * Should be less than 1/5 of PWM frequency */
#define PID_AUTOTUNE_MAX_BANDWIDTH_HZ      2000.0f

/* Safety margin factor (0.5-0.9)
 * Reduces calculated bandwidth by this factor for extra stability
 * 0.8 = use 80% of theoretical bandwidth */
#define PID_AUTOTUNE_SAFETY_MARGIN         0.8f

/* Parameter convergence threshold (0.01-0.1)
 * Relative change below this triggers convergence
 * 0.05 = 5% change threshold */
#define PID_AUTOTUNE_CONVERGENCE_THRESHOLD 0.05f

/* Stability wait time (milliseconds)
 * Time to wait for motor stabilization before tuning
 * Typical: 1000-3000 ms */
#define PID_AUTOTUNE_STABLE_TIME_MS        2000

/* Maximum tuning time (milliseconds)
 * Auto-tune will abort after this time
 * Typical: 5000-15000 ms */
#define PID_AUTOTUNE_MAX_TUNE_TIME_MS      10000
#endif  /* ENABLE_PID_AUTOTUNE */

/*******************************************************************************
 * Dead-Time Compensation Parameters
 ******************************************************************************/
#ifdef ENABLE_DEADTIME_COMPENSATION
/* Dead-time in microseconds (typical 1-3 us for MOSFETs) */
#define DEADTIME_US              2.0f

/* Dead-time voltage compensation (V_dead = Dead_time * Vbus / T_pwm)
 * This compensates for the voltage error introduced by gate driver dead-time */
#define DEADTIME_COMPENSATION_GAIN   0.02f  /* Adjustable compensation factor */
#endif

/*******************************************************************************
 * Field-Weakening Control Parameters
 ******************************************************************************/
#ifdef ENABLE_FIELD_WEAKENING
/* Base speed for field weakening activation (rad/s electrical)
 * Above this speed, negative Id current is injected to weaken flux */
#define FIELD_WEAKENING_BASE_SPEED   150.0f

/* Maximum negative Id current for field weakening (A)
 * Limits flux weakening to prevent demagnetization */
#define FIELD_WEAKENING_MAX_NEG_ID   -2.0f

/* Field weakening gain: Id_fw = -K_fw * (speed - base_speed)
 * Determines how aggressively Id is reduced above base speed */
#define FIELD_WEAKENING_GAIN         0.01f
#endif

/*******************************************************************************
 * Bus Voltage Filtering Parameters
 ******************************************************************************/
#ifdef ENABLE_VBUS_FILTERING
/* Low-pass filter coefficient (0 to 1)
 * Vbus_filtered = alpha * Vbus_new + (1-alpha) * Vbus_old
 * Lower values = more filtering, slower response
 * Typical: 0.1 for 10kHz sampling gives ~160Hz cutoff */
#define VBUS_FILTER_ALPHA            0.1f
#endif

/*******************************************************************************
 * Hybrid Hall+EKF Observer Parameters
 * Used when HYBRID_HALL_EKF_SELECT is enabled
 ******************************************************************************/

/* Hall sensor measurement noise covariance (rad²)
 * Represents uncertainty in Hall sensor position measurement
 * Hall sensors have ~60° resolution, so variance reflects this */
#define HYBRID_HALL_POSITION_NOISE   0.1f

/* Hall sensor speed measurement noise covariance (rad²/s²)
 * Represents uncertainty in speed calculated from Hall edge timing */
#define HYBRID_HALL_SPEED_NOISE      10.0f

/* Complementary filter weight for Hall position correction (0 to 1)
 * Higher values trust Hall sensors more for position
 * Lower values trust EKF prediction more
 * Typical: 0.3 gives 70% EKF, 30% Hall for smooth operation */
#define HYBRID_HALL_POSITION_WEIGHT  0.3f

/* Complementary filter weight for Hall speed correction (0 to 1)
 * Higher values trust Hall sensors more for speed
 * Lower values trust EKF prediction more
 * Typical: 0.2 gives 80% EKF, 20% Hall for smooth speed estimation */
#define HYBRID_HALL_SPEED_WEIGHT     0.2f

/* Minimum speed for Hall sensor fusion (rad/s)
 * Below this speed, Hall timing becomes unreliable due to long periods
 * Use pure EKF below this threshold */
#define HYBRID_HALL_MIN_SPEED        10.0f

/* Maximum Hall position error for fusion (rad)
 * If EKF position differs from Hall by more than this, use Hall directly
 * This handles EKF divergence scenarios
 * Typical: PI/3 (60 degrees) - one Hall sector */
#define HYBRID_HALL_MAX_POSITION_ERROR  1.047f  /* PI/3 radians */

/*******************************************************************************
 * Hall Sensor Interpolation Parameters (HALL_FOC_SELECT mode only)
 * Used when ENABLE_HALL_INTERPOLATION is defined
 ******************************************************************************/
#ifdef ENABLE_HALL_INTERPOLATION

/* Minimum speed for Hall interpolation activation (rad/s electrical)
 * Below this speed, use pure Hall sensor position (no interpolation)
 * 60 RPM mechanical = 6.28 rad/s mechanical
 * For typical BLDC motor with 3 pole pairs: 60 RPM mechanical = 18.84 rad/s electrical
 * Set to ~20 rad/s electrical for safety margin */
#define HALL_INTERPOLATION_MIN_SPEED     20.0f

/* Enable automatic misalignment offset detection and correction
 * Hall sensors may be misaligned to UVW motor phases by a small offset
 * When enabled, automatically detects and compensates for this offset */
#define ENABLE_HALL_MISALIGNMENT_CORRECTION

/* Initial misalignment offset (radians)
 * This is the initial guess for Hall sensor misalignment relative to motor axes
 * Will be automatically adjusted if ENABLE_HALL_MISALIGNMENT_CORRECTION is enabled
 * Typical range: -0.2 to +0.2 radians (-11° to +11°) */
#define HALL_MISALIGNMENT_OFFSET_INITIAL  0.0f

/* Misalignment correction filter coefficient (0 to 1)
 * Higher values = faster adaptation, lower values = more stable
 * Typical: 0.001 for slow stable adaptation */
#define HALL_MISALIGNMENT_FILTER_COEFF    0.001f

/* Maximum allowed misalignment correction (radians)
 * Limits the automatic correction to prevent runaway
 * Typical: ±0.35 radians (±20°) */
#define HALL_MISALIGNMENT_MAX_CORRECTION  0.35f

#endif  /* ENABLE_HALL_INTERPOLATION */

/*******************************************************************************
 * Linear ADRC Speed Controller Parameters
 * Used when USE_SPEED_ADRC is enabled
 ******************************************************************************/
#ifdef USE_SPEED_ADRC

/* Observer bandwidth (rad/s)
 * Determines how fast the ESO tracks disturbances
 * Typical: 50-200 rad/s (higher = faster disturbance estimation, more sensitive to noise)
 * Start with 100 rad/s and tune based on motor response */
#define SPEED_ADRC_WO_DEFAULT           100.0f

/* Controller bandwidth (rad/s)
 * Determines closed-loop response speed
 * Typical: 20-100 rad/s (higher = faster response, potential overshoot)
 * Generally set to 1/2 to 1/3 of observer bandwidth
 * Start with 50 rad/s and tune for desired response */
#define SPEED_ADRC_WC_DEFAULT           50.0f

/* System gain estimate (b0)
 * Physical meaning: Kt/J (torque constant / rotor inertia)
 * For BLDC: relates Iq current to angular acceleration
 * Rough estimate: b0 = Kt/J ≈ 100-500 for small BLDC motors
 * Can be identified experimentally or from motor datasheet
 * Start with 200 and adjust based on system response */
#define SPEED_ADRC_B0_DEFAULT           200.0f

/* Output limits (Amperes)
 * Maximum and minimum Iq current reference
 * Should match motor current rating and power supply capability */
#define SPEED_ADRC_OUTPUT_MAX           5.0f
#define SPEED_ADRC_OUTPUT_MIN          -5.0f

#endif  /* USE_SPEED_ADRC */

#endif  /* __FOC_DEFINE_PARAMETER_H__ */
