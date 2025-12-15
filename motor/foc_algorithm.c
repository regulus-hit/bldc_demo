/**********************************
 * FOC Algorithm Implementation
 * Field Oriented Control for BLDC/PMSM motors
 * Includes coordinate transforms, PI controllers, and SVPWM
 **********************************/
#include "main.h"
#include "foc_algorithm.h"

/* D-axis current PI controller parameters */
real32_T D_PI_I = 1282.8F;
real32_T D_PI_KB = 15.0F;
real32_T D_PI_LOW_LIMIT = -24.0F;
real32_T D_PI_P = 2.199F;
real32_T D_PI_UP_LIMIT = 24.0F;

/* Q-axis current PI controller parameters */
real32_T Q_PI_I = 1282.8F;
real32_T Q_PI_KB = 15.0F;
real32_T Q_PI_LOW_LIMIT = -24.0F;
real32_T Q_PI_P = 2.199F;
real32_T Q_PI_UP_LIMIT = 24.0F;

FOC_INTERFACE_STATES_DEF FOC_Interface_states;
FOC_INPUT_DEF FOC_Input;
FOC_OUTPUT_DEF FOC_Output;

RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

#ifdef __cplusplus
extern "C" {
#endif
extern void stm32_ekf_Start_wrapper(real_T *xD);
extern void stm32_ekf_Outputs_wrapper(const real32_T *u, real32_T *y, const real_T *xD);
extern void stm32_ekf_Update_wrapper(const real32_T *u, real32_T *y, real_T *xD);
extern void stm32_ekf_Terminate_wrapper(real_T *xD);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif
extern void L_identification_Start_wrapper(real_T *xD);
extern void L_identification_Outputs_wrapper(const real32_T *u, real32_T *y, const real_T *xD);
extern void L_identification_Update_wrapper(const real32_T *u, real32_T *y, real_T *xD);
extern void L_identification_Terminate_wrapper(real_T *xD);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif
extern void R_flux_identification_Start_wrapper(real_T *xD);
extern void R_flux_identification_Outputs_wrapper(const real32_T *u, real32_T *y, const real_T *xD);
extern void R_flux_identification_Update_wrapper(const real32_T *u, real32_T *y, real_T *xD);
extern void R_flux_identification_Terminate_wrapper(real_T *xD);
#ifdef __cplusplus
}
#endif

extern float float_test1;
extern float float_test2;

CURRENT_ABC_DEF Current_Iabc;
CURRENT_ALPHA_BETA_DEF Current_Ialpha_beta;
VOLTAGE_ALPHA_BETA_DEF Voltage_Alpha_Beta;
TRANSF_COS_SIN_DEF Transf_Cos_Sin;
CURRENT_DQ_DEF Current_Idq; 
VOLTAGE_DQ_DEF Voltage_DQ;
CURRENT_PID_DEF Current_D_PID;
CURRENT_PID_DEF Current_Q_PID;

/**
 * @brief Clarke Transform (abc -> alpha-beta)
 * 
 * Transforms three-phase currents (120 degrees apart) to two-phase
 * orthogonal currents (90 degrees apart) in the stationary reference frame.
 * This is the first step in FOC to simplify the control problem.
 * 
 * @param Current_abc_temp Three-phase currents (Ia, Ib, Ic)
 * @param Current_alpha_beta_temp Output: Two-phase currents (Ialpha, Ibeta)
 */
void Clarke_Transf(CURRENT_ABC_DEF Current_abc_temp, CURRENT_ALPHA_BETA_DEF* Current_alpha_beta_temp)
{
	Current_alpha_beta_temp->Ialpha = (Current_abc_temp.Ia - (Current_abc_temp.Ib + Current_abc_temp.Ic) * MATH_cos_60) * 2.0F / 3.0F;
	Current_alpha_beta_temp->Ibeta = ((Current_abc_temp.Ib - Current_abc_temp.Ic) * MATH_cos_30) * 2.0F / 3.0F;
}

/**
 * @brief Space Vector PWM Calculation
 * 
 * Calculates three-phase PWM duty cycles from alpha-beta voltage reference.
 * SVPWM provides better DC bus utilization (15% more voltage) compared to
 * sinusoidal PWM and reduces current harmonics.
 * 
 * Algorithm:
 * 1. Determine voltage vector sector (1-6)
 * 2. Calculate switching times Tx, Ty for active vectors
 * 3. Distribute zero vectors symmetrically (Ta, Tb, Tc)
 * 4. Apply over-modulation limiting if needed
 * 
 * @param v_alpha_beta_temp Alpha-beta voltage reference
 * @param Udc_temp DC bus voltage
 * @param Tpwm_temp PWM timer period
 */
void SVPWM_Calc(VOLTAGE_ALPHA_BETA_DEF v_alpha_beta_temp, real32_T Udc_temp, real32_T Tpwm_temp)
{
	int32_T sector;
	real32_T Tcmp1, Tcmp2, Tcmp3, Tx, Ty, f_temp, Ta, Tb, Tc;
	sector = 0;
	Tcmp1 = 0.0F;
	Tcmp2 = 0.0F;
	Tcmp3 = 0.0F;

	/* Determine sector (1-6) based on voltage vector angle */
	if (v_alpha_beta_temp.Vbeta > 0.0F)
	{
		sector = 1;
	}

	if ((MATH_sqrt_3 * v_alpha_beta_temp.Valpha - v_alpha_beta_temp.Vbeta) / 2.0F > 0.0F)
	{
		sector += 2;
	}

	if ((-MATH_sqrt_3 * v_alpha_beta_temp.Valpha - v_alpha_beta_temp.Vbeta) / 2.0F > 0.0F)
	{
		sector += 4;
	}

	/* Calculate active vector times for each sector */
	switch (sector)
	{
		case 1:
			Tx = (-1.5F * v_alpha_beta_temp.Valpha + MATH_cos_30 * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
			Ty = (1.5F * v_alpha_beta_temp.Valpha + MATH_cos_30 * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
			break;

		case 2:
			Tx = (1.5F * v_alpha_beta_temp.Valpha + MATH_cos_30 * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
			Ty = -(MATH_sqrt_3 * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp);
			break;

		case 3:
			Tx = -((-1.5F * v_alpha_beta_temp.Valpha + MATH_cos_30 * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
			Ty = MATH_sqrt_3 * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp;
			break;

		case 4:
			Tx = -(MATH_sqrt_3 * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp);
			Ty = (-1.5F * v_alpha_beta_temp.Valpha + MATH_cos_30 * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
			break;

		case 5:
			Tx = MATH_sqrt_3 * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp;
			Ty = -((1.5F * v_alpha_beta_temp.Valpha + MATH_cos_30 * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
			break;

		case 6:
			Tx = -((1.5F * v_alpha_beta_temp.Valpha + MATH_cos_30 * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
			Ty = -((-1.5F * v_alpha_beta_temp.Valpha + MATH_cos_30 * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
			break;

		default:
			break;
	}

	/* Over-modulation protection - scale down if total time exceeds period */
	f_temp = Tx + Ty;
	if (f_temp > Tpwm_temp)
	{
		/* Scale both vectors proportionally to fit within PWM period */
		Tx = Tx * Tpwm_temp / f_temp;
		Ty = Ty * Tpwm_temp / f_temp;
	}

	/* Calculate zero vector distribution for center-aligned PWM */
	Ta = (Tpwm_temp - (Tx + Ty)) / 4.0F;
	Tb = Tx / 2.0F + Ta;
	Tc = Ty / 2.0F + Tb;

	/* Map switching times to three phases based on sector */
	switch (sector)
	{
		case 1:
			Tcmp1 = Tb;
			Tcmp2 = Ta;
			Tcmp3 = Tc;
			break;

		case 2:
			Tcmp1 = Ta;
			Tcmp2 = Tc;
			Tcmp3 = Tb;
			break;

		case 3:
			Tcmp1 = Ta;
			Tcmp2 = Tb;
			Tcmp3 = Tc;
			break;

		case 4:
			Tcmp1 = Tc;
			Tcmp2 = Tb;
			Tcmp3 = Ta;
			break;

		case 5:
			Tcmp1 = Tc;
			Tcmp2 = Ta;
			Tcmp3 = Tb;
			break;

		case 6:
			Tcmp1 = Tb;
			Tcmp2 = Tc;
			Tcmp3 = Ta;
			break;

		default:
			break;
	}

	FOC_Output.Tcmp1 = Tcmp1;
	FOC_Output.Tcmp2 = Tcmp2;
	FOC_Output.Tcmp3 = Tcmp3;
}

/**
 * @brief Calculate sine and cosine for Park/Inverse Park transforms
 * 
 * Computes trigonometric functions needed for coordinate transforms.
 * Uses ARM CMSIS-DSP optimized functions for fast computation.
 * 
 * @param angle_temp Rotor angle in radians
 * @param cos_sin_temp Output: Cosine and sine values
 */
void Angle_To_Cos_Sin(real32_T angle_temp, TRANSF_COS_SIN_DEF* cos_sin_temp)
{
	cos_sin_temp->Cos = arm_cos_f32(angle_temp);
	cos_sin_temp->Sin = arm_sin_f32(angle_temp);
}
/**
 * @brief Park Transform (alpha-beta -> dq)
 * 
 * Transforms currents from stationary frame to rotating synchronous frame.
 * In the dq frame aligned with rotor flux, AC currents become DC values
 * which are easy to control with PI controllers.
 * 
 * - Id: Direct-axis current (flux-producing, kept at 0 for SPMSM)
 * - Iq: Quadrature-axis current (torque-producing)
 * 
 * @param current_alpha_beta_temp Stationary frame currents
 * @param cos_sin_temp Sine and cosine of rotor angle
 * @param current_dq_temp Output: Rotating frame currents
 */
void Park_Transf(CURRENT_ALPHA_BETA_DEF current_alpha_beta_temp, TRANSF_COS_SIN_DEF cos_sin_temp, CURRENT_DQ_DEF* current_dq_temp)
{
	current_dq_temp->Id = current_alpha_beta_temp.Ialpha * cos_sin_temp.Cos + current_alpha_beta_temp.Ibeta * cos_sin_temp.Sin;
	current_dq_temp->Iq = -current_alpha_beta_temp.Ialpha * cos_sin_temp.Sin + current_alpha_beta_temp.Ibeta * cos_sin_temp.Cos;
}
/**
 * @brief Inverse Park Transform (dq -> alpha-beta)
 * 
 * Transforms voltage references from rotating frame back to stationary frame.
 * Converts DC voltage commands from PI controllers to AC voltages for SVPWM.
 * 
 * @param v_dq_temp Rotating frame voltage commands from PI controllers
 * @param cos_sin_temp Sine and cosine of rotor angle
 * @param v_alpha_beta_temp Output: Stationary frame voltage references
 */
void Rev_Park_Transf(VOLTAGE_DQ_DEF v_dq_temp, TRANSF_COS_SIN_DEF cos_sin_temp, VOLTAGE_ALPHA_BETA_DEF* v_alpha_beta_temp)
{
	v_alpha_beta_temp->Valpha = cos_sin_temp.Cos * v_dq_temp.Vd - cos_sin_temp.Sin * v_dq_temp.Vq;
	v_alpha_beta_temp->Vbeta  = cos_sin_temp.Sin * v_dq_temp.Vd + cos_sin_temp.Cos * v_dq_temp.Vq;
}

/**
 * @brief Current Loop PI Controller with Anti-Windup
 * 
 * PI controller for d-axis and q-axis current regulation.
 * Implements back-calculation anti-windup to prevent integral saturation.
 * 
 * Control law:
 * - Output = Kp * error + integral
 * - Integral update includes back-calculation term for anti-windup
 * 
 * This is critical for FOC performance:
 * - Fast current response (bandwidth ~1kHz typical)
 * - Prevents current overshoot during transients
 * - Maintains torque control accuracy
 * 
 * @param ref_temp Current reference (Id_ref or Iq_ref)
 * @param fdb_temp Current feedback (Id or Iq)
 * @param out_temp Output: Voltage command (Vd or Vq)
 * @param current_pid_temp PI controller state structure
 */
void Current_PID_Calc(real32_T ref_temp, real32_T fdb_temp, real32_T* out_temp, CURRENT_PID_DEF* current_pid_temp)
{
	real32_T error;
	real32_T temp;
	
	error = ref_temp - fdb_temp;
	temp = current_pid_temp->P_Gain * error + current_pid_temp->I_Sum;
	
	/* Output limiting */
	if (temp > current_pid_temp->Max_Output)
	{
		*out_temp = current_pid_temp->Max_Output;
	}
	else if (temp < current_pid_temp->Min_Output)
	{
		*out_temp = current_pid_temp->Min_Output;
	}
	else
	{
		*out_temp = temp;
	}
	
	/* Integral update with back-calculation anti-windup */
	current_pid_temp->I_Sum += ((*out_temp - temp) * current_pid_temp->B_Gain + current_pid_temp->I_Gain * error) * 0.0001f;
}

/**
 * @brief Execute one complete FOC control cycle
 * 
 * This function implements the complete Field Oriented Control algorithm:
 * 
 * FOC Control Flow:
 * 1. Clarke Transform: Convert 3-phase currents (abc) to 2-phase (alpha-beta)
 * 2. Park Transform: Convert stationary frame to rotating frame (dq)
 * 3. Current PI Control: Regulate Id and Iq independently
 * 4. Inverse Park: Convert voltage commands back to stationary frame
 * 5. SVPWM: Generate three-phase PWM duty cycles
 * 6. State Observer: Estimate rotor position and speed (EKF)
 * 7. Parameter ID: Identify motor parameters online
 * 
 * Called at PWM frequency for real-time motor control.
 */
void foc_algorithm_step(void)
{
	/* Get three-phase current measurements */
	Current_Iabc.Ia = FOC_Input.ia;
	Current_Iabc.Ib = FOC_Input.ib;
	Current_Iabc.Ic = FOC_Input.ic;

	/* Step 1: Clarke Transform - Convert to two-phase stationary frame */
	Clarke_Transf(Current_Iabc, &Current_Ialpha_beta);

	/* Calculate sin/cos for coordinate transforms */
	Angle_To_Cos_Sin(FOC_Input.theta, &Transf_Cos_Sin);

	/* Step 2: Park Transform - Convert to rotating dq frame */
	Park_Transf(Current_Ialpha_beta, Transf_Cos_Sin, &Current_Idq);

	/* Step 3: Current PI Controllers - Regulate d and q axis currents */
	Current_PID_Calc(FOC_Input.Id_ref, Current_Idq.Id, &Voltage_DQ.Vd, &Current_D_PID);
	Current_PID_Calc(FOC_Input.Iq_ref, Current_Idq.Iq, &Voltage_DQ.Vq, &Current_Q_PID);

	/* Step 4: Inverse Park Transform - Convert voltage commands to stationary frame */
	Rev_Park_Transf(Voltage_DQ, Transf_Cos_Sin, &Voltage_Alpha_Beta);

#ifdef ENABLE_DEADTIME_COMPENSATION
	/**
	 * Step 5: Dead-Time Compensation
	 * Compensates for voltage error due to gate driver dead-time
	 * Formula: V_comp = sign(I) * V_dead * gain
	 * This improves low-speed torque and reduces current distortion
	 */
	{
		float V_comp_alpha, V_comp_beta;
		
		/* Calculate compensation voltage based on current direction
		 * sign(I_alpha) * dead-time compensation voltage */
		if (Current_Ialpha_beta.Ialpha > 0.01f)
			V_comp_alpha = DEADTIME_COMPENSATION_GAIN;
		else if (Current_Ialpha_beta.Ialpha < -0.01f)
			V_comp_alpha = -DEADTIME_COMPENSATION_GAIN;
		else
			V_comp_alpha = 0.0f;
		
		if (Current_Ialpha_beta.Ibeta > 0.01f)
			V_comp_beta = DEADTIME_COMPENSATION_GAIN;
		else if (Current_Ialpha_beta.Ibeta < -0.01f)
			V_comp_beta = -DEADTIME_COMPENSATION_GAIN;
		else
			V_comp_beta = 0.0f;
		
		/* Apply compensation to voltage commands */
		Voltage_Alpha_Beta.Valpha += V_comp_alpha;
		Voltage_Alpha_Beta.Vbeta += V_comp_beta;
	}
#endif

	/**
	 * Step 6: Extended Kalman Filter (EKF) State Observer
	 * Estimates rotor position and speed for sensorless control
	 * Uses voltage and current measurements plus motor parameters
	 */
	FOC_Interface_states.EKF_Interface[0] = Voltage_Alpha_Beta.Valpha;
	FOC_Interface_states.EKF_Interface[1] = Voltage_Alpha_Beta.Vbeta;
	FOC_Interface_states.EKF_Interface[2] = Current_Ialpha_beta.Ialpha;
	FOC_Interface_states.EKF_Interface[3] = Current_Ialpha_beta.Ibeta;
	FOC_Interface_states.EKF_Interface[4] = FOC_Input.Rs;
	FOC_Interface_states.EKF_Interface[5] = FOC_Input.Ls;
	FOC_Interface_states.EKF_Interface[6] = FOC_Input.flux;

	/* EKF output calculation */
	stm32_ekf_Outputs_wrapper(&FOC_Interface_states.EKF_Interface[0], &FOC_Output.EKF[0], &FOC_Interface_states.EKF_States[0]);

	/**
	 * Step 7: Motor Parameter Identification
	 * Online estimation of resistance, inductance, and flux linkage
	 */
	
	/* Prepare inputs for resistance and flux identification */
	FOC_Interface_states.R_flux_Ident_Interface[0] = Current_Idq.Iq;
	FOC_Interface_states.R_flux_Ident_Interface[1] = FOC_Input.speed_fdk;
	FOC_Interface_states.R_flux_Ident_Interface[2] = Voltage_DQ.Vq;

	/* Prepare inputs for inductance identification */
	FOC_Interface_states.L_Ident_Interface[0] = -(Current_Idq.Iq * FOC_Input.speed_fdk);
	FOC_Interface_states.L_Ident_Interface[1] = Voltage_DQ.Vd;

	/* Execute parameter identification outputs */
	L_identification_Outputs_wrapper(&FOC_Interface_states.L_Ident_Interface[0], &FOC_Interface_states.L_Ident_Output, &FOC_Interface_states.L_Ident_States);

	R_flux_identification_Outputs_wrapper(&FOC_Interface_states.R_flux_Ident_Interface[0], &FOC_Interface_states.R_flux_Ident_Output[0], &FOC_Interface_states.R_flux_Ident_States);

	/* Step 8: Space Vector PWM - Generate three-phase PWM duty cycles */
	SVPWM_Calc(Voltage_Alpha_Beta, FOC_Input.Udc, FOC_Input.Tpwm);

	/* Update EKF state observer */
	stm32_ekf_Update_wrapper(&FOC_Interface_states.EKF_Interface[0], &FOC_Output.EKF[0], &FOC_Interface_states.EKF_States[0]);

	/* Update parameter identification states */
	L_identification_Update_wrapper(&FOC_Interface_states.L_Ident_Interface[0], &FOC_Interface_states.L_Ident_Output, &FOC_Interface_states.L_Ident_States);

	R_flux_identification_Update_wrapper(&FOC_Interface_states.R_flux_Ident_Interface[0], &FOC_Interface_states.R_flux_Ident_Output[0], &FOC_Interface_states.R_flux_Ident_States);

	/* Pack identified motor parameters for output */
	FOC_Output.L_RF[0] = FOC_Interface_states.L_Ident_Output;
	FOC_Output.L_RF[1] = FOC_Interface_states.R_flux_Ident_Output[0];
	FOC_Output.L_RF[2] = FOC_Interface_states.R_flux_Ident_Output[1];
}


/**
 * @brief Initialize FOC Algorithm and Sub-Components
 * 
 * Performs complete initialization of FOC control system:
 * - Current loop PI controllers (D and Q axis)
 * - Speed loop PI controller
 * - Extended Kalman Filter for sensorless control
 * - Motor parameter identification algorithms
 * - All state variables
 * 
 * Must be called once before starting motor operation.
 */
void foc_algorithm_initialize(void)
{
	/* Initialize D-axis current PI controller */
	{
		Current_D_PID.P_Gain = D_PI_P;
		Current_D_PID.I_Gain = D_PI_I;
		Current_D_PID.B_Gain = D_PI_KB;
		Current_D_PID.Max_Output = D_PI_UP_LIMIT;
		Current_D_PID.Min_Output = D_PI_LOW_LIMIT;
		Current_D_PID.I_Sum = 0.0f;

		/* Initialize Q-axis current PI controller */
		Current_Q_PID.P_Gain = Q_PI_P;
		Current_Q_PID.I_Gain = Q_PI_I;
		Current_Q_PID.B_Gain = Q_PI_KB;
		Current_Q_PID.Max_Output = Q_PI_UP_LIMIT;
		Current_Q_PID.Min_Output = Q_PI_LOW_LIMIT;
		Current_Q_PID.I_Sum = 0.0f;
	}

	/* Initialize speed loop PI controller */
	speed_pid_initialize();

	/* Initialize Extended Kalman Filter for position/speed estimation */
	stm32_ekf_Start_wrapper(&FOC_Interface_states.EKF_States[0]);

	/* Initialize motor parameter identification algorithms */
	L_identification_Start_wrapper(&FOC_Interface_states.L_Ident_States);
	R_flux_identification_Start_wrapper(&FOC_Interface_states.R_flux_Ident_States);

	/* Initialize EKF state variables */
	{
		real_T initVector[4] = { 0, 0, 0, 0 };
		{
			int_T i1;
			real_T *dw_DSTATE = &FOC_Interface_states.EKF_States[0];
			for (i1 = 0; i1 < 4; i1++)
			{
				dw_DSTATE[i1] = initVector[i1];
			}
		}
	}

	/* Initialize inductance identification state */
	{
		real_T initVector[1] = { 0 };
		{
			int_T i1;
			for (i1 = 0; i1 < 1; i1++)
			{
				FOC_Interface_states.L_Ident_States = initVector[0];
			}
		}
	}

	/* Initialize resistance/flux identification state */
	{
		real_T initVector[1] = { 0 };
		{
			int_T i1;
			for (i1 = 0; i1 < 1; i1++)
			{
				FOC_Interface_states.R_flux_Ident_States = initVector[0];
			}
		}
	}
}

