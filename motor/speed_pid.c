/**********************************
 * Speed PI Controller Implementation
 * Outer loop speed control for BLDC motor
 **********************************/
#include "speed_pid.h"

#define SPEED_PID_PERIOD 0.001F		/* Speed loop period: 1ms (1kHz) */

/* Speed PI controller parameters */
real32_T SPEED_PI_I = 5.0F;
real32_T SPEED_PI_KB = 0.015F;
real32_T SPEED_PI_LOW_LIMIT = -5.0F;
real32_T SPEED_PI_P = 0.003F;
real32_T SPEED_PI_UP_LIMIT = 5.0F;

real32_T Speed_Ref;			/* Speed reference in Hz */
real32_T Speed_Fdk;			/* Speed feedback in rad/s */
real32_T Speed_Pid_Out;		/* Speed PID output -> Iq reference for torque control */

SPEED_PID_DEF Speed_Pid;

/**
 * @brief Speed PI Controller Calculation
 * 
 * Cascaded outer loop controller for speed regulation.
 * Converts speed error to torque current reference (Iq).
 * 
 * Key Features:
 * - Automatic unit conversion (Hz to rad/s)
 * - Output limiting to prevent current saturation
 * - Anti-windup via back-calculation method
 * 
 * Control Structure:
 * Speed_Ref (Hz) -> [PI Controller] -> Iq_Ref (A) -> Current Loop
 * 
 * @param ref_temp Speed reference in Hz
 * @param fdb_temp Speed feedback in rad/s (from encoder or observer)
 * @param out_temp Output: Iq current reference in Amperes
 * @param current_pid_temp PI controller state structure
 */
void Speed_Pid_Calc(real32_T ref_temp, real32_T fdb_temp, real32_T* out_temp, SPEED_PID_DEF* current_pid_temp)
{
	real32_T error;
	real32_T temp;

	/* Calculate error with unit conversion: Hz -> rad/s (multiply by 2*PI) */
	error = 6.28318548F * ref_temp - fdb_temp;

	/* PI control law: P + I (standard form) */
	temp = current_pid_temp->P_Gain * error + current_pid_temp->I_Sum;

	/* Output limiting */
	if (temp > current_pid_temp->Max_Output) {
		*out_temp = current_pid_temp->Max_Output;
	} else if (temp < current_pid_temp->Min_Output) {
		*out_temp = current_pid_temp->Min_Output;
	} else {
		*out_temp = temp;
	}

	/* Integral update with back-calculation anti-windup */
	current_pid_temp->I_Sum += ((*out_temp - temp) * current_pid_temp->B_Gain + current_pid_temp->I_Gain * error) * SPEED_PID_PERIOD;
}

/**
 * @brief Initialize Speed PI Controller
 * 
 * Loads PI gains and limits, resets integral accumulator.
 * Must be called before motor startup.
 */
void speed_pid_initialize(void)
{
	Speed_Pid.P_Gain = SPEED_PI_P;
	Speed_Pid.I_Gain = SPEED_PI_I;
	Speed_Pid.B_Gain = SPEED_PI_KB;
	Speed_Pid.Max_Output = SPEED_PI_UP_LIMIT;
	Speed_Pid.Min_Output = SPEED_PI_LOW_LIMIT;
	Speed_Pid.I_Sum = 0.0f;
}


