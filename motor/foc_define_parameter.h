/**********************************
      
**********************************/
#ifndef __FOC_DEFINE_PARAMETER_H__
#define __FOC_DEFINE_PARAMETER_H__

#define MOTOR_STARTUP_CURRENT   1.0f   //电机启动电流，根据自己实际负载设置 1.0
#define SPEED_LOOP_CLOSE_RAD_S  50.0f  //速度环切入闭环的速度  单位: rad/s

//有感FOC 或 无感FOC选择，总得注释掉其中一个
#define HALL_FOC_SELECT          //此行注释掉就不使用有感FOC运行
//#define SENSORLESS_FOC_SELECT    //此行注释掉就使用有感FOC运行


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

#endif
