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
 * Advanced FOC Enhancement Features
 * Uncomment to enable each feature independently for debugging and testing
 ******************************************************************************/

/* Dead-Time Compensation: Compensates voltage error from gate driver dead-time
 * Improves low-speed torque and reduces current distortion */
#define ENABLE_DEADTIME_COMPENSATION

/* Field-Weakening Control: Extends speed range beyond base speed
 * Enables operation at higher speeds by injecting negative Id current */
#define ENABLE_FIELD_WEAKENING

/* Bus Voltage Filtering: Low-pass filters DC bus voltage measurement
 * Reduces SVPWM errors from DC link ripple, improves voltage utilization */
#define ENABLE_VBUS_FILTERING

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
#ifdef HYBRID_HALL_EKF_SELECT
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

#endif

#endif
