/**********************************
 * Inductance Identification Wrapper
 * Recursive Least Squares (RLS) algorithm for online motor inductance estimation
 * Auto-generated wrapper for Simulink-generated identification code
 **********************************/
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

#include <math.h>

/* RLS algorithm variables for inductance identification */
float l_h;        /* Observation coefficient */
float l_Pn0;      /* Covariance matrix element (previous) */
float l_Pn1;      /* Covariance matrix element (current) */
float l_x1;       /* State variable: estimated inductance */
float l_x2;       /* State variable: covariance */
float l_K;        /* RLS gain */
float l_theta0;   /* Parameter estimate (previous) */
float l_theta1;   /* Parameter estimate (current) */
float l_Ud;       /* Input: measured voltage */

#define u_width 2  /* Input vector width */
#define y_width 1  /* Output vector width */

/**
 * @brief Initialize Inductance Identification RLS Algorithm
 * 
 * Initializes Recursive Least Squares parameters for online inductance estimation:
 * - Initial inductance estimate: 0.01 H (10 mH, typical for small PMSM)
 * - Initial covariance: Large value for fast convergence
 * 
 * The RLS algorithm adaptively estimates motor inductance Ls from voltage
 * and current measurements during operation.
 * 
 * @param xD Discrete state vector storage
 */
void L_identification_Start_wrapper(real_T *xD)
{
	l_theta0 = 0.01f;           /* Initial inductance estimate: 10 mH */
	l_Pn0 = 0.0008f * 2.0f;     /* Initial covariance (large for fast adaptation) */

	l_x1 = l_theta0;  /* Initialize state with initial estimate */
	l_x2 = l_Pn0;     /* Initialize covariance state */
}

/**
 * @brief Output Inductance Identification Result
 * 
 * Returns the current estimated motor inductance.
 * 
 * @param u Input vector (not used in output stage)
 * @param y Output vector: [estimated_inductance]
 * @param xD Discrete state vector
 */
void L_identification_Outputs_wrapper(const real32_T *u, real32_T *y, const real_T *xD)
{
	l_theta1 = l_x1;  /* Get current estimate */
	y[0] = l_theta1;  /* Output estimated inductance (Henries) */
}

/**
 * @brief Update Inductance Identification RLS Algorithm
 * 
 * Implements Recursive Least Squares update for online inductance estimation:
 * 1. Compute RLS gain K based on current covariance
 * 2. Update covariance matrix
 * 3. Compute prediction error and update parameter estimate
 * 4. Store updated states
 * 
 * Uses forgetting factor 0.99 to allow tracking of parameter changes.
 * 
 * Input u[2]: [observation_coefficient, measured_voltage]
 * State: Estimated inductance Ls
 * 
 * @param u Input measurements for RLS update
 * @param y Output estimates (updated by Outputs_wrapper)
 * @param xD Discrete state vector (updated in-place)
 */
void L_identification_Update_wrapper(const real32_T *u, real32_T *y, real_T *xD)
{
	l_h = u[0];      /* Observation coefficient from current measurement */
	l_Pn0 = l_x2;    /* Get previous covariance */
	
	/* Calculate RLS gain */
	l_K = l_Pn0 * l_h / (1 + l_h * l_Pn0 * l_h);
	
	/* Update covariance */
	l_Pn1 = (l_Pn0 - l_K * l_h * l_Pn0);
	
	/* Update parameter estimate */
	l_theta0 = l_x1;  /* Get previous estimate */
	l_Ud = u[1];      /* Get measured voltage */
	l_theta1 = l_theta0 + l_K * (l_Ud - l_h * 0.99f * l_theta0);  /* RLS update with forgetting factor */
	
	/* Store updated states */
	l_x1 = l_theta1;  /* Store new estimate */
	l_x2 = l_Pn1;     /* Store new covariance */
}

