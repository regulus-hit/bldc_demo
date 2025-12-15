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

/* RLS Algorithm Configuration Parameters */
#define RLS_INITIAL_RESISTANCE          0.05f    /* Initial Rs estimate (Ohms): 50 mΩ typical for small PMSM */
#define RLS_INITIAL_FLUX_LINKAGE        0.01f    /* Initial flux linkage estimate (Wb): 10 mWb typical for small PM motors */
#define RLS_INITIAL_COVARIANCE_BASE     0.0008f  /* Base covariance value for initialization */
#define RLS_COVARIANCE_SCALE_DIAGONAL   2.0f     /* Scale factor for diagonal covariance elements (fast adaptation) */
#define RLS_COVARIANCE_SCALE_OFFDIAG    0.0f     /* Scale factor for off-diagonal elements (uncorrelated assumption) */
#define RLS_FORGETTING_FACTOR           0.999f   /* Forgetting factor: 0.999 allows slow parameter drift tracking */

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
 * of stator resistance Rs and permanent magnet flux linkage. Unlike single-parameter
 * RLS (L_identification), this algorithm estimates two coupled parameters simultaneously
 * from voltage-current relationships in the q-axis.
 * 
 * Initial Parameter Estimates:
 * - Rs: 0.05 Ω (50 mΩ, typical for small PMSM motors)
 * - Flux linkage: 0.01 Wb (10 mWb, typical for small permanent magnet motors)
 * 
 * Initial Covariance Matrix (2x2):
 * - Diagonal elements: Large values (0.0016) for fast initial convergence
 * - Off-diagonal elements: Zero (assumes parameters initially uncorrelated)
 * 
 * The dual-parameter identification uses a 2x2 covariance matrix and 2x1 gain vector,
 * requiring more computational effort than single-parameter RLS but providing
 * simultaneous estimation of both electrical parameters.
 * 
 * State Vector Structure:
 * - x_1_1: Current Rs estimate
 * - x_2_1: Current flux estimate
 * - x_1_2, x_1_3, x_2_2, x_2_3: Elements of 2x2 covariance matrix P
 * 
 * @param xD Discrete state vector storage (not currently used by this implementation)
 */
void R_flux_identification_Start_wrapper(real_T *xD)
{
	/* Initial parameter estimates */
	theta0_1_1 = RLS_INITIAL_RESISTANCE;   /* Initial resistance: 50 mΩ */
	theta0_2_1 = RLS_INITIAL_FLUX_LINKAGE; /* Initial flux linkage: 10 mWb */

	/* Initial covariance matrix (2x2, large diagonal for fast adaptation) */
	Pn0_1_1 = RLS_INITIAL_COVARIANCE_BASE * RLS_COVARIANCE_SCALE_DIAGONAL; /* Variance for Rs */
	Pn0_1_2 = RLS_INITIAL_COVARIANCE_BASE * RLS_COVARIANCE_SCALE_OFFDIAG;  /* Covariance Rs-flux (uncorrelated) */
	Pn0_2_1 = RLS_INITIAL_COVARIANCE_BASE * RLS_COVARIANCE_SCALE_OFFDIAG;  /* Covariance flux-Rs (uncorrelated) */
	Pn0_2_2 = RLS_INITIAL_COVARIANCE_BASE * RLS_COVARIANCE_SCALE_DIAGONAL; /* Variance for flux */

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
 * Extracts and returns the current estimated motor parameters from the RLS algorithm.
 * This function is called by Simulink-generated code to read the identified parameters
 * which can then be used by the EKF for improved state estimation accuracy.
 * 
 * The identified parameters represent online estimates that adapt to motor operating
 * conditions, temperature changes, and magnetic saturation effects.
 * 
 * @param u Input vector (not used in output stage)
 * @param y Output vector: [estimated_Rs (Ohms), estimated_flux (Webers)]
 * @param xD Discrete state vector (not used in this implementation)
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
 * of stator resistance Rs and flux linkage. This is the core algorithm that performs
 * online parameter identification during motor operation.
 * 
 * Algorithm Steps:
 * 1. Extract observation coefficients and measurements from input vector
 *    - h[1]: Observation coefficient linking Rs to output (current-dependent)
 *    - h[2]: Observation coefficient linking flux to output (speed-dependent)
 *    - r_Uq: Measured q-axis voltage (torque-producing component)
 * 
 * 2. Compute 2x1 RLS gain vector K = P * h / (1 + h^T * P * h)
 *    - Denominator is scalar innovation covariance
 *    - Numerator is cross-covariance between parameters and output
 * 
 * 3. Update 2x2 covariance matrix: P_new = P - K * h^T * P
 *    - Reduces uncertainty in parameter estimates after measurement
 *    - Joseph form ensures numerical stability and symmetry
 * 
 * 4. Compute prediction error: e = measured - predicted
 *    - predicted = h[1]*Rs + h[2]*flux
 *    - Forgetting factor 0.999 allows slow parameter drift tracking
 * 
 * 5. Update parameter estimates: theta_new = theta_old + K * e
 *    - Corrects both Rs and flux based on measurement residual
 *    - Gain K determines step size for each parameter
 * 
 * 6. Store updated states for next iteration
 * 
 * Forgetting Factor (0.999):
 * - Values < 1.0 slowly "forget" old data, allowing parameter tracking
 * - 0.999 means data 1000 samples ago has ~37% weight (exponential decay)
 * - Trade-off: faster tracking vs. higher noise sensitivity
 * 
 * Input u[3]: [observation_coef_Rs, observation_coef_flux, measured_Uq]
 * States: [Rs_estimate, flux_estimate, P[1,1], P[1,2], P[2,1], P[2,2]]
 * 
 * Typical operating conditions:
 * - Sample rate: 10 kHz (100 µs period, same as EKF)
 * - Convergence time: ~0.1-1 second depending on excitation
 * - Requires sufficient motor current and speed variation for observability
 * 
 * @param u Input measurements for RLS update
 * @param y Output estimates (updated by Outputs_wrapper)
 * @param xD Discrete state vector (not used in this implementation)
 */
void R_flux_identification_Update_wrapper(const real32_T *u, real32_T *y, real_T *xD)
{
	/* Extract inputs from input vector */
	h_1_1 = u[0];  /* Observation coefficient for Rs (typically current-related) */
	h_2_1 = u[1];  /* Observation coefficient for flux (typically speed-related) */
	r_Uq = u[2];   /* Measured q-axis voltage (torque-producing component) */
	
	/* Get previous covariance matrix P0 from state variables */
	Pn0_1_1 = x_1_2;  /* P[1,1] */
	Pn0_1_2 = x_1_3;  /* P[1,2] */
	Pn0_2_1 = x_2_2;  /* P[2,1] */
	Pn0_2_2 = x_2_3;  /* P[2,2] */

	/* 
	 * Compute RLS gain K = P * h / (1 + h^T * P * h)
	 * First: Calculate innovation covariance (scalar): 1 + h^T * P * h
	 */
	r_temp_1_1 = 1.0f + (((h_1_1 * Pn0_1_1 + h_2_1 * Pn0_2_1) * h_1_1) + 
	                      ((h_1_1 * Pn0_1_2 + h_2_1 * Pn0_2_2) * h_2_1));
	r_temp_1_1 = 1.0f / r_temp_1_1;  /* Invert innovation covariance */
	
	/* Compute 2x1 gain vector K = P * h / (1 + h^T * P * h) */
	r_K_1_1 = (Pn0_1_1 * h_1_1 + Pn0_1_2 * h_2_1) * r_temp_1_1;  /* Gain for Rs */
	r_K_2_1 = (Pn0_2_1 * h_1_1 + Pn0_2_2 * h_2_1) * r_temp_1_1;  /* Gain for flux */

	/* 
	 * Update covariance matrix: P_new = P_old - K * h^T * P_old
	 * First: Compute K * h^T (outer product giving 2x2 matrix)
	 */
	r_temp_1_1 = r_K_1_1 * h_1_1;  /* K[1] * h[1] */
	r_temp_1_2 = r_K_1_1 * h_2_1;  /* K[1] * h[2] */
	r_temp_2_1 = r_K_2_1 * h_1_1;  /* K[2] * h[1] */
	r_temp_2_2 = r_K_2_1 * h_2_1;  /* K[2] * h[2] */
	
	/* Second: P_new = P_old - (K * h^T) * P_old (Joseph form update) */
	Pn1_1_1 = Pn0_1_1 - (r_temp_1_1 * Pn0_1_1 + r_temp_1_2 * Pn0_2_1);
	Pn1_1_2 = Pn0_1_2 - (r_temp_1_1 * Pn0_1_2 + r_temp_1_2 * Pn0_2_2);
	Pn1_2_1 = Pn0_2_1 - (r_temp_2_1 * Pn0_1_1 + r_temp_2_2 * Pn0_2_1);
	Pn1_2_2 = Pn0_2_2 - (r_temp_2_1 * Pn0_1_2 + r_temp_2_2 * Pn0_2_2);

	/* Get previous parameter estimates */
	theta0_1_1 = x_1_1;  /* Previous Rs estimate */
	theta0_2_1 = x_2_1;  /* Previous flux estimate */

	/* 
	 * Update parameter estimates: theta_new = theta_old + K * e
	 * Prediction error e = measured - predicted
	 * predicted = h[1]*Rs + h[2]*flux
	 * Forgetting factor 0.999 in prediction for parameter drift tracking
	 */
	theta1_1_1 = theta0_1_1 + r_K_1_1 * (r_Uq - (h_1_1 * theta0_1_1 * RLS_FORGETTING_FACTOR + h_2_1 * theta0_2_1 * RLS_FORGETTING_FACTOR));
	theta1_2_1 = theta0_2_1 + r_K_2_1 * (r_Uq - (h_1_1 * theta0_1_1 * RLS_FORGETTING_FACTOR + h_2_1 * theta0_2_1 * RLS_FORGETTING_FACTOR));

	/* Store updated states for next iteration */
	x_1_1 = theta1_1_1;  /* Updated Rs estimate */
	x_2_1 = theta1_2_1;  /* Updated flux estimate */
	x_1_2 = Pn1_1_1;     /* Updated covariance P[1,1] */
	x_1_3 = Pn1_1_2;     /* Updated covariance P[1,2] */
	x_2_2 = Pn1_2_1;     /* Updated covariance P[2,1] */
	x_2_3 = Pn1_2_2;     /* Updated covariance P[2,2] */
}

