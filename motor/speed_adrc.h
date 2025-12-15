/**********************************
 * Speed Linear ADRC Controller Implementation
 * Active Disturbance Rejection Control for BLDC motor speed loop
 * 
 * Linear ADRC provides better disturbance rejection than PID by:
 * 1. Estimating and compensating total disturbances via Extended State Observer (ESO)
 * 2. Using state error feedback control law for improved robustness
 * 
 * References:
 * - Han, J. "From PID to Active Disturbance Rejection Control", IEEE Trans. Industrial Electronics, 2009
 * - Gao, Z. "Scaling and bandwidth-parameterization based controller tuning", ACC 2003
 * - SimpleFOC: Modern ADRC implementations for motor control
 **********************************/

#ifndef RTW_HEADER_speed_adrc_h_
#define RTW_HEADER_speed_adrc_h_

#include <stddef.h>
#ifndef speed_adrc_COMMON_INCLUDES_
# define speed_adrc_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "MW_target_hardware_resources.h"

/**
 * @brief Linear ADRC Extended State Observer (ESO) Structure
 * 
 * Three-state observer estimates:
 * - z1: Speed state estimate (tracks actual speed)
 * - z2: Speed derivative estimate (acceleration)
 * - z3: Extended state estimate (total disturbance including load torque, friction, etc.)
 */
typedef struct
{
    real32_T z1;        /* State 1: Speed estimate (rad/s) */
    real32_T z2;        /* State 2: Speed derivative estimate (rad/s^2) */
    real32_T z3;        /* State 3: Extended state (total disturbance) */
    real32_T beta1;     /* Observer gain 1 */
    real32_T beta2;     /* Observer gain 2 */
    real32_T beta3;     /* Observer gain 3 */
    real32_T b0;        /* System gain estimate (for BLDC: relates Iq to acceleration) */
} SPEED_ADRC_ESO;

/**
 * @brief Linear ADRC Controller Structure
 * 
 * Complete Linear ADRC controller for speed regulation.
 * Outer loop controller that generates torque (Iq) reference.
 */
typedef struct
{
    SPEED_ADRC_ESO eso;     /* Extended State Observer */
    real32_T kp;            /* Proportional gain for error feedback */
    real32_T kd;            /* Derivative gain for error feedback */
    real32_T Max_Output;    /* Upper output limit (max Iq) */
    real32_T Min_Output;    /* Lower output limit (min Iq) */
} SPEED_ADRC_DEF;

/* Global ADRC controller variables */
extern real32_T Speed_Ref_ADRC;      /* Speed reference in Hz */
extern real32_T Speed_Fdk_ADRC;      /* Speed feedback in rad/s */
extern real32_T Speed_Adrc_Out;      /* ADRC output -> Iq reference for torque control */

extern SPEED_ADRC_DEF Speed_Adrc;

/* ADRC controller parameters (tunable) */
extern real32_T SPEED_ADRC_WO;           /* Observer bandwidth (rad/s) */
extern real32_T SPEED_ADRC_WC;           /* Controller bandwidth (rad/s) */
extern real32_T SPEED_ADRC_B0;           /* System gain estimate */
extern real32_T SPEED_ADRC_LOW_LIMIT;    /* Lower output limit */
extern real32_T SPEED_ADRC_UP_LIMIT;     /* Upper output limit */

/**
 * @brief Initialize Speed Linear ADRC Controller
 * 
 * Sets up ADRC gains and limits based on bandwidth parameters.
 * Must be called before motor startup.
 * 
 * Bandwidth-parameterization approach (Gao, 2003):
 * - Observer gains: beta1 = 3*wo, beta2 = 3*wo^2, beta3 = wo^3
 * - Controller gains: kp = wc^2, kd = 2*wc
 * 
 * where wo is observer bandwidth and wc is controller bandwidth.
 */
extern void speed_adrc_initialize(void);

/**
 * @brief Speed Linear ADRC Controller Calculation
 * 
 * Two-stage control:
 * 1. ESO estimates speed, acceleration, and total disturbance
 * 2. Control law generates Iq reference based on error and disturbance compensation
 * 
 * Control law: u = (kp*e + kd*(0 - z2) - z3) / b0
 * where e = ref - z1, and z3 is the estimated total disturbance
 * 
 * Advantages over PID:
 * - Active disturbance rejection via z3 compensation
 * - No integral windup issues
 * - Better load disturbance rejection
 * - Faster transient response with less overshoot
 * 
 * @param ref_temp Speed reference in Hz
 * @param fdb_temp Speed feedback in rad/s (from encoder or observer)
 * @param out_temp Output: Iq current reference in Amperes
 * @param current_adrc_temp ADRC controller state structure
 */
extern void Speed_Adrc_Calc(real32_T ref_temp, real32_T fdb_temp, real32_T* out_temp, SPEED_ADRC_DEF* current_adrc_temp);

#endif  /* RTW_HEADER_speed_adrc_h_ */
