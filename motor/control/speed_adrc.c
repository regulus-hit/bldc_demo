/**********************************
 * Speed Linear ADRC Controller Implementation
 * Active Disturbance Rejection Control for BLDC motor speed loop
 * 
 * Linear ADRC consists of:
 * 1. Linear Extended State Observer (LESO) - estimates disturbances
 * 2. Linear State Error Feedback (LSEF) - generates control based on error
 * 
 * Advantages over traditional PID:
 * - Superior disturbance rejection (load changes, friction, etc.)
 * - No integral windup
 * - Easier tuning via bandwidth parameters
 * - Better transient response
 * 
 * This file is compiled only when USE_SPEED_ADRC is defined
 **********************************/
#include "../foc_define_parameter.h"

#ifdef USE_SPEED_ADRC

#include "speed_adrc.h"
#include "../../user/main.h"  /* For MATH_2PI constant */

#define SPEED_ADRC_PERIOD 0.001F    /* ADRC loop period: 1ms (1kHz) */

/* Speed ADRC controller parameters */
real32_T SPEED_ADRC_WO = 100.0F;         /* Observer bandwidth: 100 rad/s (typical: 50-200) */
real32_T SPEED_ADRC_WC = 50.0F;          /* Controller bandwidth: 50 rad/s (typical: 20-100) */
real32_T SPEED_ADRC_B0 = 200.0F;         /* System gain estimate (Kt/J, motor-dependent) */
real32_T SPEED_ADRC_LOW_LIMIT = -5.0F;   /* Lower output limit (Iq) */
real32_T SPEED_ADRC_UP_LIMIT = 5.0F;     /* Upper output limit (Iq) */

/* Reuse PID variable names since only one controller is active at a time */
real32_T Speed_Ref;             /* Speed reference in Hz (shared with PID) */
real32_T Speed_Fdk;             /* Speed feedback in rad/s (shared with PID) */
real32_T Speed_Pid_Out;         /* Controller output -> Iq reference (shared with PID) */

SPEED_PID_DEF Speed_Pid;        /* ADRC controller instance (shared name with PID) */

/**
 * @brief Initialize Speed Controller (ADRC Implementation)
 * 
 * Calculates observer and controller gains based on bandwidth parameters.
 * Uses standard bandwidth-parameterization approach (Gao, 2003).
 * 
 * ESO gain selection:
 * - beta1 = 3*wo (for fast state tracking)
 * - beta2 = 3*wo^2 (for acceleration estimation)
 * - beta3 = wo^3 (for disturbance estimation)
 * 
 * Controller gain selection:
 * - kp = wc^2 (proportional gain)
 * - kd = 2*wc (derivative gain)
 * 
 * These formulas ensure:
 * - Observer poles at -wo (fast disturbance estimation)
 * - Controller poles optimally placed for step response
 */
void speed_controller_init(void)
{
	real32_T wo = SPEED_ADRC_WO;
	real32_T wc = SPEED_ADRC_WC;
	
	/* ESO gains based on observer bandwidth */
	Speed_Pid.eso.beta1 = 3.0f * wo;
	Speed_Pid.eso.beta2 = 3.0f * wo * wo;
	Speed_Pid.eso.beta3 = wo * wo * wo;
	
	/* System gain estimate */
	Speed_Pid.eso.b0 = SPEED_ADRC_B0;
	
	/* Controller gains based on controller bandwidth */
	Speed_Pid.kp = wc * wc;
	Speed_Pid.kd = 2.0f * wc;
	
	/* Output limits */
	Speed_Pid.Max_Output = SPEED_ADRC_UP_LIMIT;
	Speed_Pid.Min_Output = SPEED_ADRC_LOW_LIMIT;
	
	/* Initialize ESO states */
	Speed_Pid.eso.z1 = 0.0f;
	Speed_Pid.eso.z2 = 0.0f;
	Speed_Pid.eso.z3 = 0.0f;
}

/**
 * @brief Speed Controller Calculation (ADRC Implementation)
 * 
 * Implements complete Linear ADRC algorithm:
 * 
 * Step 1: Extended State Observer (ESO)
 * Observes three states:
 *   z1: speed estimate
 *   z2: acceleration estimate
 *   z3: total disturbance estimate (load, friction, model uncertainty)
 * 
 * ESO dynamics:
 *   e = z1 - y  (observation error)
 *   dz1/dt = z2 - beta1*e
 *   dz2/dt = z3 - beta2*e + b0*u
 *   dz3/dt = -beta3*e
 * 
 * Step 2: Control Law
 * Error feedback with disturbance compensation:
 *   e_speed = ref - z1
 *   e_accel = 0 - z2  (desired acceleration is 0 at steady state)
 *   u = (kp*e_speed + kd*e_accel - z3) / b0
 * 
 * The z3 term actively rejects disturbances, key advantage over PID.
 * 
 * @param ref_temp Speed reference in Hz
 * @param fdb_temp Speed feedback in rad/s (from encoder or observer)
 * @param out_temp Output: Iq current reference in Amperes
 * @param current_adrc_temp ADRC controller state structure
 */
void speed_controller_calc(real32_T ref_temp, real32_T fdb_temp, real32_T* out_temp, SPEED_PID_DEF* current_adrc_temp)
{
	real32_T speed_ref_rad;
	real32_T e;             /* Observation error */
	real32_T e_speed;       /* Speed tracking error */
	real32_T e_accel;       /* Acceleration error */
	real32_T u_raw;         /* Raw control output before limiting */
	real32_T u_limited;     /* Limited control output */
	real32_T dt = SPEED_ADRC_PERIOD;
	
	/* Convert speed reference from Hz to rad/s */
	speed_ref_rad = MATH_2PI * ref_temp;
	
	/***************************************************************************
	 * Extended State Observer (ESO) Update
	 ***************************************************************************/
	
	/* Observation error: difference between estimated and measured speed */
	e = current_adrc_temp->eso.z1 - fdb_temp;
	
	/* Update z1: speed state estimate
	 * Tracks the actual speed with correction based on observation error */
	current_adrc_temp->eso.z1 += (current_adrc_temp->eso.z2 - current_adrc_temp->eso.beta1 * e) * dt;
	
	/* Update z2: acceleration estimate
	 * Incorporates control input (u) and corrects for observation error
	 * Note: Uses previous sample's control output (*out_temp from last call).
	 * This is standard for discrete-time ADRC implementation where ESO runs
	 * before control law to avoid algebraic loop. */
	current_adrc_temp->eso.z2 += (current_adrc_temp->eso.z3 - current_adrc_temp->eso.beta2 * e + 
								  current_adrc_temp->eso.b0 * (*out_temp)) * dt;
	
	/* Update z3: extended state (total disturbance) estimate
	 * This is the key to ADRC - estimates all disturbances for rejection */
	current_adrc_temp->eso.z3 += (-current_adrc_temp->eso.beta3 * e) * dt;
	
	/***************************************************************************
	 * Control Law: State Error Feedback with Disturbance Compensation
	 ***************************************************************************/
	
	/* Speed tracking error */
	e_speed = speed_ref_rad - current_adrc_temp->eso.z1;
	
	/* Acceleration error (desired acceleration = 0 at steady state) */
	e_accel = 0.0f - current_adrc_temp->eso.z2;
	
	/* Control law with disturbance compensation:
	 * u = (kp*e_speed + kd*e_accel - z3) / b0
	 * 
	 * The "-z3" term actively cancels the estimated total disturbance.
	 * This provides superior disturbance rejection compared to PID. */
	u_raw = (current_adrc_temp->kp * e_speed + 
			 current_adrc_temp->kd * e_accel - 
			 current_adrc_temp->eso.z3) / current_adrc_temp->eso.b0;
	
	/* Output limiting */
	if (u_raw > current_adrc_temp->Max_Output)
	{
		u_limited = current_adrc_temp->Max_Output;
	}
	else if (u_raw < current_adrc_temp->Min_Output)
	{
		u_limited = current_adrc_temp->Min_Output;
	}
	else
	{
		u_limited = u_raw;
	}
	
	/* Update output */
	*out_temp = u_limited;
	
	/* Note: Unlike PID, ADRC does not have integral windup issues.
	 * The ESO automatically adapts to steady-state disturbances via z3,
	 * eliminating the need for anti-windup schemes. */
}

#endif  /* USE_SPEED_ADRC */
