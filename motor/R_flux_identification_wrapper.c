/**********************************
 * Resistance and Flux Linkage Identification Wrapper
 * Multi-parameter RLS algorithm for simultaneous estimation of Rs and flux
 * Auto-generated wrapper for Simulink-generated identification code
 **********************************/
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

#include <math.h>

/* RLS algorithm variables for 2-parameter identification (Rs and flux) */
float theta0_1_1;   /* Parameter estimate: Resistance Rs (previous) */
float theta0_2_1;   /* Parameter estimate: Flux linkage (previous) */
float Pn0_1_1;      /* Covariance matrix element [1,1] (previous) */
float Pn0_1_2;      /* Covariance matrix element [1,2] (previous) */
float Pn0_2_1;      /* Covariance matrix element [2,1] (previous) */
float Pn0_2_2;      /* Covariance matrix element [2,2] (previous) */
float x_1_1;        /* State variable: estimated Rs */
float x_1_2;        /* State variable: covariance [1,1] */
float x_1_3;        /* State variable: covariance [1,2] */
float x_2_1;        /* State variable: estimated flux */
float x_2_2;        /* State variable: covariance [2,1] */
float x_2_3;        /* State variable: covariance [2,2] */
float h_1_1;        /* Observation coefficient for Rs */
float h_2_1;        /* Observation coefficient for flux */
float r_K_1_1;      /* RLS gain element for Rs */
float r_K_2_1;      /* RLS gain element for flux */
float Pn1_1_1;      /* Covariance matrix element [1,1] (current) */
float Pn1_1_2;      /* Covariance matrix element [1,2] (current) */
float Pn1_2_1;      /* Covariance matrix element [2,1] (current) */
float Pn1_2_2;      /* Covariance matrix element [2,2] (current) */
float theta1_1_1;   /* Parameter estimate: Rs (current) */
float theta1_2_1;   /* Parameter estimate: flux (current) */
float r_temp_1_1;   /* Temporary calculation variable */
float r_temp_1_2;   /* Temporary calculation variable */
float r_temp_2_1;   /* Temporary calculation variable */
float r_temp_2_2;   /* Temporary calculation variable */
float r_Uq;         /* Input: measured q-axis voltage */

#define u_width 3  /* Input vector width */
#define y_width 1  /* Output vector width */

/**
 * @brief Initialize Resistance and Flux Identification RLS Algorithm
 * 
 * Initializes multi-parameter Recursive Least Squares for simultaneous estimation
 * of stator resistance Rs and permanent magnet flux linkage:
 * - Initial Rs estimate: 0.05 Ω (typical for small PMSM)
 * - Initial flux estimate: 0.01 Wb (typical for small PMSM)
 * - Initial covariance: Diagonal matrix with large values for fast convergence
 * 
 * This dual-parameter identification is more complex than single-parameter RLS
 * as it estimates both Rs and flux from voltage/current measurements.
 * 
 * @param xD Discrete state vector storage
 */
void R_flux_identification_Start_wrapper(real_T *xD)
{
	/* Initial parameter estimates */
	theta0_1_1 = 0.05f;  /* Initial resistance: 50 mΩ */
	theta0_2_1 = 0.01f;  /* Initial flux linkage: 10 mWb */

	/* Initial covariance matrix (2x2, large diagonal for fast adaptation) */
	Pn0_1_1 = 0.0008f * 2.0f;  /* Variance for Rs */
	Pn0_1_2 = 0.0008f * 0.0f;  /* Covariance Rs-flux (uncorrelated) */
	Pn0_2_1 = 0.0008f * 0.0f;  /* Covariance flux-Rs (uncorrelated) */
	Pn0_2_2 = 0.0008f * 2.0f;  /* Variance for flux */

	/* Initialize state variables */
	x_1_1 = theta0_1_1;  /* Rs estimate */
	x_2_1 = theta0_2_1;  /* Flux estimate */
	x_1_2 = Pn0_1_1;     /* Covariance [1,1] */
	x_1_3 = Pn0_1_2;     /* Covariance [1,2] */
	x_2_2 = Pn0_2_1;     /* Covariance [2,1] */
	x_2_3 = Pn0_2_2;     /* Covariance [2,2] */
}

/**
 * @brief Output Resistance and Flux Identification Results
 * 
 * Returns the current estimated motor parameters.
 * 
 * @param u Input vector (not used in output stage)
 * @param y Output vector: [estimated_Rs, estimated_flux]
 * @param xD Discrete state vector
 */
void R_flux_identification_Outputs_wrapper(const real32_T *u, real32_T *y, const real_T *xD)
{
	y[0] = x_1_1;  /* Output estimated resistance Rs (Ohms) */
	y[1] = x_2_1;  /* Output estimated flux linkage (Webers) */
}


/**
 * @brief Update Resistance and Flux Identification RLS Algorithm
 * 
 * Implements multi-parameter Recursive Least Squares for simultaneous estimation
 * of Rs and flux:
 * 1. Extract observation coefficients and measurements from inputs
 * 2. Compute 2x2 Kalman gain matrix
 * 3. Update 2x2 covariance matrix
 * 4. Compute prediction errors for both parameters
 * 5. Update both parameter estimates (Rs and flux)
 * 6. Store updated states
 * 
 * Uses forgetting factor 0.99 to track parameter changes over time.
 * The algorithm solves for both Rs and flux by observing voltage-current
 * relationships in the q-axis.
 * 
 * Input u[3]: [observation_coef_Rs, observation_coef_flux, measured_Uq]
 * States: [Rs_estimate, flux_estimate, covariance_matrix]
 * 
 * @param u Input measurements for RLS update
 * @param y Output estimates (updated by Outputs_wrapper)
 * @param xD Discrete state vector (updated in-place)
 */
void R_flux_identification_Update_wrapper(const real32_T *u, real32_T *y, real_T *xD)
{
	/* Extract inputs */
	h_1_1 = u[0];  /* Observation coefficient for Rs */
	h_2_1 = u[1];  /* Observation coefficient for flux */
	r_Uq = u[2];   /* Measured q-axis voltage */
	
	/* Get previous covariance matrix */
	Pn0_1_1 = x_1_2;
	Pn0_1_2 = x_1_3;
	Pn0_2_1 = x_2_2;
	Pn0_2_2 = x_2_3;

	r_temp_1_1 = 1.0f + (((h_1_1*Pn0_1_1 + h_2_1* Pn0_2_1)*h_1_1) + ((h_1_1*Pn0_1_2 + h_2_1* Pn0_2_2)*h_2_1));
	r_temp_1_1 = 1.0f/r_temp_1_1;
	r_K_1_1 = (Pn0_1_1*h_1_1+Pn0_1_2*h_2_1)*r_temp_1_1;
	r_K_2_1 = (Pn0_2_1*h_1_1+Pn0_2_2*h_2_1)*r_temp_1_1;

	r_temp_1_1 = r_K_1_1*h_1_1;
	r_temp_1_2 = r_K_1_1*h_2_1;
	r_temp_2_1 = r_K_2_1*h_1_1;
	r_temp_2_2 = r_K_2_1*h_2_1;
	Pn1_1_1 = Pn0_1_1 - (r_temp_1_1*Pn0_1_1+r_temp_1_2*Pn0_2_1);
	Pn1_1_2 = Pn0_1_2 - (r_temp_1_1*Pn0_1_2+r_temp_1_2*Pn0_2_2);
	Pn1_2_1 = Pn0_2_1 - (r_temp_2_1*Pn0_1_1+r_temp_2_2*Pn0_2_1);
	Pn1_2_2 = Pn0_2_2 - (r_temp_2_1*Pn0_1_2+r_temp_2_2*Pn0_2_2);

	theta0_1_1 = x_1_1;
	theta0_2_1 = x_2_1;

	theta1_1_1 = theta0_1_1 + r_K_1_1*(r_Uq - (h_1_1*theta0_1_1*0.999f+h_2_1*theta0_2_1*0.999f));
	theta1_2_1 = theta0_2_1 + r_K_2_1*(r_Uq - (h_1_1*theta0_1_1*0.999f+h_2_1*theta0_2_1*0.999f));

	x_1_1 = theta1_1_1;
	x_2_1 = theta1_2_1;
	x_1_2 = Pn1_1_1;
	x_1_3 = Pn1_1_2;
	x_2_2 = Pn1_2_1;
	x_2_3 = Pn1_2_2;
}

