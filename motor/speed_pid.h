
#ifndef RTW_HEADER_speed_pid_h_
#define RTW_HEADER_speed_pid_h_
#include <stddef.h>
#ifndef speed_pid_COMMON_INCLUDES_
# define speed_pid_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                               

#include "MW_target_hardware_resources.h"


#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#define speed_pid_M                    (rtM)



extern real32_T Speed_Ref;		/* Speed reference in Hz */
extern real32_T Speed_Fdk;		/* Speed feedback in rad/s */
extern real32_T Speed_Pid_Out;	/* Speed PI output (Iq reference) */

/**
 * @brief Speed PI Controller Structure
 * 
 * PI controller for speed regulation in FOC control.
 * Outer loop controller that generates torque (Iq) reference.
 */
typedef struct
{
	real32_T P_Gain;		/* Proportional gain */
	real32_T I_Gain;		/* Integral gain */
	real32_T D_Gain;		/* Derivative gain (unused) */
	real32_T B_Gain;		/* Anti-windup back-calculation gain */
	real32_T Max_Output;	/* Upper output limit */
	real32_T Min_Output;	/* Lower output limit */
	real32_T I_Sum;			/* Integral accumulator */
} SPEED_PID_DEF;

extern SPEED_PID_DEF Speed_Pid;

/**
 * @brief Initialize Speed Controller
 * 
 * Sets up controller gains and limits, clears state variables.
 * Generic initialization function for speed control (PID implementation).
 */
extern void speed_controller_init(void);

/**
 * @brief Speed Controller Calculation
 * 
 * Outer loop controller for speed regulation. Generates torque current
 * reference (Iq_ref) based on speed error. Typically runs at lower
 * frequency than current loop (e.g., 1kHz vs 10kHz).
 * 
 * Generic interface for speed control (PID implementation).
 * 
 * @param ref_temp Speed reference in Hz
 * @param fdb_temp Speed feedback in rad/s
 * @param out_temp Output: Iq current reference
 * @param current_pid_temp PI controller state structure
 */
extern void speed_controller_calc(real32_T ref_temp, real32_T fdb_temp, real32_T* out_temp, SPEED_PID_DEF* current_pid_temp);

extern real32_T SPEED_PI_I;
extern real32_T SPEED_PI_KB;
extern real32_T SPEED_PI_LOW_LIMIT;
extern real32_T SPEED_PI_P;
extern real32_T SPEED_PI_UP_LIMIT;


#endif                            
