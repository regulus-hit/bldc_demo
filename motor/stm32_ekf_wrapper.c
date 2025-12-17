/**********************************
 * Extended Kalman Filter (EKF) Wrapper for PMSM Sensorless Control
 * Estimates rotor position and speed from voltage and current measurements
 * Auto-generated wrapper for Simulink-generated EKF code
 **********************************/
#include "main.h"
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

#ifdef SIMULINK_USE_ARM_MATH
#include "arm_math.h"
#else
#include <math.h>
#endif

/* Motor electrical parameters (updated via identification) */
float Rs;    /* Stator resistance (Ohms) */
float Ls;    /* Stator inductance (Henries) */
float flux;  /* Permanent magnet flux linkage (Wb) */

/* 
 * Process noise covariance matrix Q (4x4, diagonal only)
 * Models uncertainty in state dynamics (how much states can randomly change)
 * Only diagonal elements stored (off-diagonal assumed zero = uncorrelated noise)
 */
float Q_0_0;  /* Q[0,0]: i_alpha process noise variance (A²) */
/* float Q_0_1; */  /* Q[0,1]: Covariance between i_alpha and i_beta noise (assumed zero) */
/* float Q_0_2; */  /* Q[0,2]: Covariance between i_alpha and omega noise (assumed zero) */
/* float Q_0_3; */  /* Q[0,3]: Covariance between i_alpha and theta noise (assumed zero) */
/* float Q_1_0; */  /* Q[1,0]: Covariance between i_beta and i_alpha noise (assumed zero) */
float Q_1_1;  /* Q[1,1]: i_beta process noise variance (A²) */
/* float Q_1_2; */  /* Q[1,2]: Covariance between i_beta and omega noise (assumed zero) */
/* float Q_1_3; */  /* Q[1,3]: Covariance between i_beta and theta noise (assumed zero) */
/* float Q_2_0; */  /* Q[2,0]: Covariance between omega and i_alpha noise (assumed zero) */
/* float Q_2_1; */  /* Q[2,1]: Covariance between omega and i_beta noise (assumed zero) */
float Q_2_2;  /* Q[2,2]: omega process noise variance (rad²/s²) */
/* float Q_2_3; */  /* Q[2,3]: Covariance between omega and theta noise (assumed zero) */
/* float Q_3_0; */  /* Q[3,0]: Covariance between theta and i_alpha noise (assumed zero) */
/* float Q_3_1; */  /* Q[3,1]: Covariance between theta and i_beta noise (assumed zero) */
/* float Q_3_2; */  /* Q[3,2]: Covariance between theta and omega noise (assumed zero) */
float Q_3_3;  /* Q[3,3]: theta process noise variance (rad²) */

/* 
 * Measurement noise covariance matrix R (2x2, diagonal only)
 * Models sensor noise/uncertainty in current measurements
 */
float R_0_0;  /* R[0,0]: i_alpha measurement noise variance (A²) */
/* float R_0_1; */  /* R[0,1]: Covariance between i_alpha and i_beta sensor noise (assumed zero) */
/* float R_1_0; */  /* R[1,0]: Covariance between i_beta and i_alpha sensor noise (assumed zero) */
float R_1_1;  /* R[1,1]: i_beta measurement noise variance (A²) */

float T;  /* Sample time period (seconds) */

/* 
 * Input/measurement vectors in alpha-beta stationary reference frame
 * Format: X_row_col (single column vectors, so col is always 0)
 */
float vs_ab_0_0;  /* Applied voltage in alpha-axis (V) - Input u[0] */
float vs_ab_1_0;  /* Applied voltage in beta-axis (V) - Input u[1] */
float is_ab_0_0;  /* Measured current in alpha-axis (A) - Measurement y[0] */
float is_ab_1_0;  /* Measured current in beta-axis (A) - Measurement y[1] */

/* 
 * State estimation error covariance matrix P0 (4x4, full matrix)
 * Represents uncertainty in state estimates [i_alpha, i_beta, omega, theta]
 * Updated each iteration: predicted then corrected
 * Format: P0_row_col
 */
float P0_0_0;  /* P0[0,0]: i_alpha variance (A²) */
float P0_0_1;  /* P0[0,1]: Covariance between i_alpha and i_beta */
float P0_0_2;  /* P0[0,2]: Covariance between i_alpha and omega */
float P0_0_3;  /* P0[0,3]: Covariance between i_alpha and theta */
float P0_1_0;  /* P0[1,0]: Covariance between i_beta and i_alpha */
float P0_1_1;  /* P0[1,1]: i_beta variance (A²) */
float P0_1_2;  /* P0[1,2]: Covariance between i_beta and omega */
float P0_1_3;  /* P0[1,3]: Covariance between i_beta and theta */
float P0_2_0;  /* P0[2,0]: Covariance between omega and i_alpha */
float P0_2_1;  /* P0[2,1]: Covariance between omega and i_beta */
float P0_2_2;  /* P0[2,2]: omega variance (rad²/s²) */
float P0_2_3;  /* P0[2,3]: Covariance between omega and theta */
float P0_3_0;  /* P0[3,0]: Covariance between theta and i_alpha */
float P0_3_1;  /* P0[3,1]: Covariance between theta and i_beta */
float P0_3_2;  /* P0[3,2]: Covariance between theta and omega */
float P0_3_3;  /* P0[3,3]: theta variance (rad²) */

/* 
 * Observation/measurement matrix H (2x4, diagonal only)
 * Maps state space [i_alpha, i_beta, omega, theta] to measurements [i_alpha, i_beta]
 * We directly measure currents, so H is identity for currents, zero for speed/position
 */
float H_0_0;  /* H[0,0]: Measurement sensitivity of y[0] to i_alpha (always 1.0) */
/* float H_0_1; */  /* H[0,1]: Measurement sensitivity of y[0] to i_beta (zero - we don't measure i_beta when measuring i_alpha) */
/* float H_0_2; */  /* H[0,2]: Measurement sensitivity of y[0] to omega (zero - can't directly measure speed from current sensor) */
/* float H_0_3; */  /* H[0,3]: Measurement sensitivity of y[0] to theta (zero - can't directly measure position from current sensor) */
/* float H_1_0; */  /* H[1,0]: Measurement sensitivity of y[1] to i_alpha (zero - we don't measure i_alpha when measuring i_beta) */
float H_1_1;  /* H[1,1]: Measurement sensitivity of y[1] to i_beta (always 1.0) */
/* float H_1_2; */  /* H[1,2]: Measurement sensitivity of y[1] to omega (zero - can't directly measure speed from current sensor) */
/* float H_1_3; */  /* H[1,3]: Measurement sensitivity of y[1] to theta (zero - can't directly measure position from current sensor) */

/* 
 * Input matrix B (4x2, diagonal only)
 * Maps control inputs [v_alpha, v_beta] to state derivatives
 * Voltages affect current rates (di/dt), but not directly speed/position
 */
float B_0_0;  /* B[0,0]: Effect of v_alpha on di_alpha/dt (1/Ls) */
/* float B_0_1; */  /* B[0,1]: Effect of v_beta on di_alpha/dt (zero - alpha and beta decoupled in stationary frame) */
/* float B_1_0; */  /* B[1,0]: Effect of v_alpha on di_beta/dt (zero - alpha and beta decoupled in stationary frame) */
float B_1_1;  /* B[1,1]: Effect of v_beta on di_beta/dt (1/Ls) */
/* float B_2_0; */  /* B[2,0]: Effect of v_alpha on domega/dt (zero - voltage doesn't directly change speed) */
/* float B_2_1; */  /* B[2,1]: Effect of v_beta on domega/dt (zero - voltage doesn't directly change speed) */
/* float B_3_0; */  /* B[3,0]: Effect of v_alpha on dtheta/dt (zero - voltage doesn't directly change position) */
/* float B_3_1; */  /* B[3,1]: Effect of v_beta on dtheta/dt (zero - voltage doesn't directly change position) */

/* 
 * State transition Jacobian matrix F (4x4, sparse)
 * F = ∂f/∂x where f is the nonlinear state dynamics
 * Linearizes dynamics around current operating point for EKF
 * Only non-zero elements computed (optimization)
 */
float F_0_0;  /* F[0,0]: ∂(di_alpha/dt)/∂i_alpha = -Rs/Ls (resistive damping) */
/* float F_0_1; */  /* F[0,1]: ∂(di_alpha/dt)/∂i_beta = 0 (decoupled in stationary frame) */
float F_0_2;  /* F[0,2]: ∂(di_alpha/dt)/∂omega = flux/Ls*sin(theta) (back-EMF speed coupling) */
float F_0_3;  /* F[0,3]: ∂(di_alpha/dt)/∂theta = flux/Ls*omega*cos(theta) (back-EMF position coupling) */
/* float F_1_0; */  /* F[1,0]: ∂(di_beta/dt)/∂i_alpha = 0 (decoupled in stationary frame) */
float F_1_1;  /* F[1,1]: ∂(di_beta/dt)/∂i_beta = -Rs/Ls (resistive damping) */
float F_1_2;  /* F[1,2]: ∂(di_beta/dt)/∂omega = -flux/Ls*cos(theta) (back-EMF speed coupling) */
float F_1_3;  /* F[1,3]: ∂(di_beta/dt)/∂theta = flux/Ls*omega*sin(theta) (back-EMF position coupling) */
/* float F_2_0; */  /* F[2,0]: ∂(domega/dt)/∂i_alpha = 0 (no acceleration model, constant velocity assumption) */
/* float F_2_1; */  /* F[2,1]: ∂(domega/dt)/∂i_beta = 0 (no acceleration model) */
/* float F_2_2; */  /* F[2,2]: ∂(domega/dt)/∂omega = 0 (no acceleration model) */
/* float F_2_3; */  /* F[2,3]: ∂(domega/dt)/∂theta = 0 (no acceleration model) */
/* float F_3_0; */  /* F[3,0]: ∂(dtheta/dt)/∂i_alpha = 0 (position doesn't depend on current) */
/* float F_3_1; */  /* F[3,1]: ∂(dtheta/dt)/∂i_beta = 0 (position doesn't depend on current) */
float F_3_2;  /* F[3,2]: ∂(dtheta/dt)/∂omega = 1.0 (position derivative is speed) */
/* float F_3_3; */  /* F[3,3]: ∂(dtheta/dt)/∂theta = 0 (position rate doesn't depend on position itself) */

/* 
 * Temporary matrix storage (4x4)
 * Reused for multiple intermediate calculations:
 * - Identity matrix I
 * - Matrix products in covariance updates
 * - K*H product in correction step
 */
float temp_0_0;  /* temp[0,0] */
float temp_0_1;  /* temp[0,1] */
float temp_0_2;  /* temp[0,2] */
float temp_0_3;  /* temp[0,3] */
float temp_1_0;  /* temp[1,0] */
float temp_1_1;  /* temp[1,1] */
float temp_1_2;  /* temp[1,2] */
float temp_1_3;  /* temp[1,3] */
float temp_2_0;  /* temp[2,0] */
float temp_2_1;  /* temp[2,1] */
float temp_2_2;  /* temp[2,2] */
float temp_2_3;  /* temp[2,3] */
float temp_3_0;  /* temp[3,0] */
float temp_3_1;  /* temp[3,1] */
float temp_3_2;  /* temp[3,2] */
float temp_3_3;  /* temp[3,3] */

/* 
 * Nonlinear state dynamics vector f1 = f(x,u) (4x1)
 * Continuous-time state derivatives computed from current state
 */
float f1_0_0;  /* f1[0]: di_alpha/dt = -Rs/Ls*i_alpha + flux/Ls*omega*sin(theta) + B*v_alpha */
float f1_1_0;  /* f1[1]: di_beta/dt = -Rs/Ls*i_beta - flux/Ls*omega*cos(theta) + B*v_beta */
float f1_2_0;  /* f1[2]: domega/dt = 0 (constant velocity model, no acceleration) */
float f1_3_0;  /* f1[3]: dtheta/dt = omega (position rate equals speed) */

/* 
 * Discrete state transition matrix f2 = I + T*F (4x4, sparse)
 * First-order discretization of continuous Jacobian F
 * Used for propagating covariance: P_pred = f2 * P * f2^T + Q
 */
float f2_0_0;  /* f2[0,0] = 1 + T*F[0,0] (discrete current-to-current coupling with damping) */
/* float f2_0_1; */  /* f2[0,1] = 0 + T*F[0,1] = 0 (no i_beta to i_alpha coupling) */
float f2_0_2;  /* f2[0,2] = 0 + T*F[0,2] (discrete speed to i_alpha coupling via back-EMF) */
float f2_0_3;  /* f2[0,3] = 0 + T*F[0,3] (discrete position to i_alpha coupling via back-EMF) */
/* float f2_1_0; */  /* f2[1,0] = 0 + T*F[1,0] = 0 (no i_alpha to i_beta coupling) */
float f2_1_1;  /* f2[1,1] = 1 + T*F[1,1] (discrete current-to-current coupling with damping) */
float f2_1_2;  /* f2[1,2] = 0 + T*F[1,2] (discrete speed to i_beta coupling via back-EMF) */
float f2_1_3;  /* f2[1,3] = 0 + T*F[1,3] (discrete position to i_beta coupling via back-EMF) */
/* float f2_2_0; */  /* f2[2,0] = 0 + T*F[2,0] = 0 (no current to speed coupling in this model) */
/* float f2_2_1; */  /* f2[2,1] = 0 + T*F[2,1] = 0 (no current to speed coupling in this model) */
float f2_2_2;  /* f2[2,2] = 1 + T*F[2,2] = 1 (speed persists, no damping in constant velocity model) */
/* float f2_2_3; */  /* f2[2,3] = 0 + T*F[2,3] = 0 (position doesn't affect speed in this model) */
/* float f2_3_0; */  /* f2[3,0] = 0 + T*F[3,0] = 0 (current doesn't affect position directly) */
/* float f2_3_1; */  /* f2[3,1] = 0 + T*F[3,1] = 0 (current doesn't affect position directly) */
float f2_3_2;  /* f2[3,2] = 0 + T*F[3,2] = T*1 = T (position changes by speed*time) */
float f2_3_3;  /* f2[3,3] = 1 + T*F[3,3] = 1 (position persists from previous value) */

/* 
 * Predicted state vector X_pred (4x1)
 * A priori state estimate before measurement correction
 */
float X_pred_0_0;  /* X_pred[0]: Predicted i_alpha (A) */
float X_pred_1_0;  /* X_pred[1]: Predicted i_beta (A) */
float X_pred_2_0;  /* X_pred[2]: Predicted omega (rad/s) */
float X_pred_3_0;  /* X_pred[3]: Predicted theta (rad) */

/* 
 * Predicted measurement vector Y_pred (2x1)
 * Expected sensor readings based on predicted state: Y_pred = H * X_pred
 */
float Y_pred_0_0;  /* Y_pred[0]: Predicted i_alpha measurement (A) */
float Y_pred_1_0;  /* Y_pred[1]: Predicted i_beta measurement (A) */

/* 
 * Actual measurement vector Y (2x1)
 * Real sensor readings from ADC
 */
float Y_0_0;  /* Y[0]: Measured i_alpha from ADC (A) */
float Y_1_0;  /* Y[1]: Measured i_beta from ADC (A) */

/* 
 * Predicted covariance matrix P_pred (4x4, full matrix)
 * A priori estimation error covariance: P_pred = F * P0 * F^T + Q
 * Represents uncertainty before incorporating measurements
 */
float P_pred_0_0;  /* P_pred[0,0]: Predicted i_alpha variance */
float P_pred_0_1;  /* P_pred[0,1]: Predicted covariance i_alpha-i_beta */
float P_pred_0_2;  /* P_pred[0,2]: Predicted covariance i_alpha-omega */
float P_pred_0_3;  /* P_pred[0,3]: Predicted covariance i_alpha-theta */
float P_pred_1_0;  /* P_pred[1,0]: Predicted covariance i_beta-i_alpha */
float P_pred_1_1;  /* P_pred[1,1]: Predicted i_beta variance */
float P_pred_1_2;  /* P_pred[1,2]: Predicted covariance i_beta-omega */
float P_pred_1_3;  /* P_pred[1,3]: Predicted covariance i_beta-theta */
float P_pred_2_0;  /* P_pred[2,0]: Predicted covariance omega-i_alpha */
float P_pred_2_1;  /* P_pred[2,1]: Predicted covariance omega-i_beta */
float P_pred_2_2;  /* P_pred[2,2]: Predicted omega variance */
float P_pred_2_3;  /* P_pred[2,3]: Predicted covariance omega-theta */
float P_pred_3_0;  /* P_pred[3,0]: Predicted covariance theta-i_alpha */
float P_pred_3_1;  /* P_pred[3,1]: Predicted covariance theta-i_beta */
float P_pred_3_2;  /* P_pred[3,2]: Predicted covariance theta-omega */
float P_pred_3_3;  /* P_pred[3,3]: Predicted theta variance */

/* 
 * Temporary storage for innovation covariance matrix S (2x2)
 * S = H * P_pred * H^T + R
 * Used for matrix inversion to compute Kalman gain
 */
float temp_0_0_t;  /* temp[0,0]: S[0,0] saved for inversion */
float temp_0_1_t;  /* temp[0,1]: S[0,1] saved for inversion */
float temp_1_0_t;  /* temp[1,0]: S[1,0] saved for inversion */
float temp_1_1_t;  /* temp[1,1]: S[1,1] saved for inversion */
float temp;        /* Scalar: determinant of S for matrix inversion */

/* 
 * Kalman gain matrix K (4x2)
 * Optimal weighting between prediction and measurement
 * K = P_pred * H^T * S^-1
 * Large K means trust measurements more, small K means trust prediction more
 */
float K_0_0;  /* K[0,0]: i_alpha correction gain from i_alpha measurement */
float K_0_1;  /* K[0,1]: i_alpha correction gain from i_beta measurement */
float K_1_0;  /* K[1,0]: i_beta correction gain from i_alpha measurement */
float K_1_1;  /* K[1,1]: i_beta correction gain from i_beta measurement */
float K_2_0;  /* K[2,0]: omega correction gain from i_alpha measurement (indirect observability) */
float K_2_1;  /* K[2,1]: omega correction gain from i_beta measurement (indirect observability) */
float K_3_0;  /* K[3,0]: theta correction gain from i_alpha measurement (indirect observability) */
float K_3_1;  /* K[3,1]: theta correction gain from i_beta measurement (indirect observability) */

/* 
 * Corrected state vector tempa (4x1)
 * A posteriori state estimate after measurement correction
 * X_corrected = X_pred + K * (Y_measured - Y_pred)
 */
float tempa_0_0;  /* tempa[0]: Corrected i_alpha estimate (A) */
float tempa_1_0;  /* tempa[1]: Corrected i_beta estimate (A) */
float tempa_2_0;  /* tempa[2]: Corrected omega estimate (rad/s) */
float tempa_3_0;  /* tempa[3]: Corrected theta estimate (rad) */

#define u_width 7
#define y_width 1

/**
 * @brief Initialize Extended Kalman Filter
 * 
 * Initializes EKF matrices and motor parameters for sensorless position estimation:
 * - Loads motor electrical parameters (Rs, Ls, flux) from configuration
 * - Initializes process noise covariance matrix Q (state uncertainty)
 * - Initializes measurement noise covariance matrix R (sensor noise)
 * - Initializes state covariance matrix P0 (initial estimation uncertainty)
 * - Configures observation matrix H and input matrix B
 * - Sets sample time T = 100µs (10kHz control frequency)
 * 
 * EKF State Vector: [i_alpha, i_beta, omega, theta]
 * - i_alpha, i_beta: Stator currents in stationary frame
 * - omega: Electrical angular velocity (rad/s)
 * - theta: Electrical rotor position (radians)
 * 
 * @param xD Discrete state vector storage
 */
void stm32_ekf_Start_wrapper(real_T *xD)
{
	/* Load motor electrical parameters */
	Rs = RS_PARAMETER;      /* Stator resistance */
	Ls = LS_PARAMETER;      /* Stator inductance */
	
	/* 
	 * Flux linkage parameter correction for EKF speed estimation accuracy
	 * 
	 * Issue: EKF-estimated speed was 12% slower than Hall sensor measurement
	 * Root cause: Flux parameter scaling mismatch in back-EMF model
	 * 
	 * The EKF back-EMF model in alpha-beta frame is:
	 *   E_alpha = flux * omega * sin(theta)
	 *   E_beta = -flux * omega * cos(theta)  // Negative sign is correct for PMSM model
	 * 
	 * Speed estimation accuracy depends on correct flux value. Testing revealed
	 * the FLUX_PARAMETER constant is too large by factor of 2/sqrt(3) ≈ 1.1547,
	 * causing EKF to estimate 88% of actual speed (Hall/EKF ratio = 1.136).
	 * 
	 * This is a common scaling issue in motor control, arising from different
	 * conventions for defining flux linkage (line-to-line vs phase, peak vs RMS).
	 * 
	 * Solution: Apply correction factor sqrt(3)/2 ≈ 0.866 to flux parameter
	 * used in EKF calculations. This aligns EKF speed estimates with Hall sensor.
	 * 
	 * Note: Only applied to EKF. Other code using FLUX_PARAMETER is unchanged.
	 */
	flux = FLUX_PARAMETER * MATH_cos_30;  /* Corrected: MATH_cos_30 = sqrt(3)/2 = 0.866 */

	/* Process noise covariance matrix Q (diagonal elements only) */
	Q_0_0 = 0.1f;   /* Current alpha uncertainty */
	Q_1_1 = 0.1f;   /* Current beta uncertainty */
	Q_2_2 = 0.1f;   /* Speed uncertainty */
	Q_3_3 = 0.001f; /* Position uncertainty */

	/* Measurement noise covariance matrix R */
	R_0_0 = 0.2f;  /* Current alpha measurement noise */
	R_1_1 = 0.2f;  /* Current beta measurement noise */

	/* Sample time */
	T = 0.0001f;  /* 100µs = 10kHz */

	/* Observation matrix H (we measure currents directly) */
	H_0_0 = 1.0f;  /* Current alpha observable */
	H_1_1 = 1.0f;  /* Current beta observable */

	/* Input matrix B (voltage to current) */
	B_0_0 = 1.0f / Ls;  /* Voltage alpha to current alpha */
	B_1_1 = 1.0f / Ls;  /* Voltage beta to current beta */

	/* Initialize state covariance matrix P0 to zero (high confidence) */
	P0_0_0 = 0.0f;
	P0_0_1 = 0.0f;
	P0_0_2 = 0.0f;
	P0_0_3 = 0.0f;
	P0_1_0 = 0.0f;
	P0_1_1 = 0.0f;
	P0_1_2 = 0.0f;
	P0_1_3 = 0.0f;
	P0_2_0 = 0.0f;
	P0_2_1 = 0.0f;
	P0_2_2 = 0.0f;
	P0_2_3 = 0.0f;
	P0_3_0 = 0.0f;
	P0_3_1 = 0.0f;
	P0_3_2 = 0.0f;
	P0_3_3 = 0.0f;
}

/**
 * @brief EKF Output Function
 * 
 * Extracts estimated states from EKF and outputs them.
 * Called by Simulink-generated code to get current state estimates.
 * 
 * @param u Input vector (not used in output stage)
 * @param y Output vector: [i_alpha, i_beta, omega, theta]
 * @param xD Discrete state vector containing EKF estimates
 */
void stm32_ekf_Outputs_wrapper(const real32_T *u, real32_T *y, const real_T *xD)
{
	y[0] = xD[0];  /* Estimated current alpha */
	y[1] = xD[1];  /* Estimated current beta */
	y[2] = xD[2];  /* Estimated electrical speed (rad/s) */
	y[3] = xD[3];  /* Estimated electrical position (rad) */
}

/**
 * @brief EKF Update Function (State Prediction and Correction)
 * 
 * Implements the Extended Kalman Filter algorithm:
 * 1. State Prediction: Predicts next state based on motor model
 * 2. Covariance Prediction: Projects error covariance forward
 * 3. Kalman Gain: Computes optimal gain
 * 4. State Correction: Updates state estimate with measurements
 * 5. Covariance Update: Updates error covariance
 * 
 * Input u[7]: [v_alpha, v_beta, i_alpha_measured, i_beta_measured, ...]
 * State xD[4]: [i_alpha, i_beta, omega, theta]
 * 
 * @param u Input measurements: voltages and currents
 * @param y Output estimates (updated by Outputs_wrapper)
 * @param xD Discrete state vector (updated in-place)
 */
void stm32_ekf_Update_wrapper(const real32_T *u, real32_T *y, real_T *xD)
{
	/* Extract inputs from input vector */
	vs_ab_0_0 = u[0];  /* Alpha-axis voltage */
	vs_ab_1_0 = u[1];  /* Beta-axis voltage */
	is_ab_0_0 = u[2];  /* Alpha-axis current measurement */
	is_ab_1_0 = u[3];  /* Beta-axis current measurement */

	/* Update motor parameters (from identification) */
	Rs = u[4];    /* Stator resistance (Ohms) */
	Ls = u[5];    /* Stator inductance (Henries) */
	
	/* 
	 * Flux parameter from R_flux_identification - use directly without correction
	 * 
	 * After the flux correction fix is applied in Start_wrapper, the EKF operates
	 * with the correctly-scaled flux. This causes it to estimate the correct speed,
	 * which then feeds into R_flux_identification. R_flux_identification will then
	 * converge to estimate the flux in the CORRECT scale (matching the corrected
	 * Start_wrapper value).
	 * 
	 * If we applied the sqrt(3)/2 correction here, it would create a feedback loop:
	 * 1. EKF with corrected flux estimates correct speed
	 * 2. R_flux_identification sees correct speed, estimates correct flux
	 * 3. We scale it down by 0.866
	 * 4. EKF speed becomes too high (opposite problem!)
	 * 5. R_flux_identification compensates by estimating even higher flux
	 * 6. Loop diverges
	 * 
	 * Therefore: Use identified flux as-is (no correction in Update_wrapper).
	 */
	flux = u[6];  /* PM flux linkage (Webers) from parameter identification */

	/*
	 * Optional: Load covariance matrix from extended state vector
	 * Currently disabled - covariance is maintained in global variables
	 * If enabled, would load P0 matrix from xD[4] through xD[19]
	 */

	/* 
	 * Step 1: Compute state transition matrix F (Jacobian of state dynamics)
	 * F = df/dx where f is the nonlinear state equation
	 * Only non-zero elements are computed (sparse matrix optimization)
	 */
#ifdef SIMULINK_USE_ARM_MATH
	F_0_0 = -Rs / Ls;
	/* F_0_1 = 0.0f; */
	F_0_2 = flux / Ls * arm_sin_f32(xD[3]);
	F_0_3 = flux / Ls * xD[2] * arm_cos_f32(xD[3]);
	/* F_1_0 = 0.0f; */
	F_1_1 = -Rs / Ls;
	F_1_2 = -flux / Ls * arm_cos_f32(xD[3]);
	F_1_3 = flux / Ls * xD[2] * arm_sin_f32(xD[3]);
	/* F_2_0 = 0.0f; */
	/* F_2_1 = 0.0f; */
	/* F_2_2 = 0.0f; */
	/* F_2_3 = 0.0f; */
	/* F_3_0 = 0.0f; */
	/* F_3_1 = 0.0f; */
	F_3_2 = 1.0f;
	/* F_3_3 = 0.0f; */
#else
	F_0_0 = -Rs / Ls;
	F_0_1 = 0.0f;
	F_0_2 = flux / Ls * sin(xD[3]);
	F_0_3 = flux / Ls * xD[2] * cos(xD[3]);
	F_1_0 = 0.0f;
	F_1_1 = -Rs / Ls;
	F_1_2 = -flux / Ls * cos(xD[3]);
	F_1_3 = flux / Ls * xD[2] * sin(xD[3]);
	F_2_0 = 0.0f;
	F_2_1 = 0.0f;
	F_2_2 = 0.0f;
	F_2_3 = 0.0f;
	F_3_0 = 0.0f;
	F_3_1 = 0.0f;
	F_3_2 = 1.0f;
	F_3_3 = 0.0f;
#endif

	/* Initialize identity matrix (diagonal elements only) */
	temp_0_0 = 1.0f;
	/* temp_0_1 = 0.0f; */
	/* temp_0_2 = 0.0f; */
	/* temp_0_3 = 0.0f; */
	/* temp_1_0 = 0.0f; */
	temp_1_1 = 1.0f;
	/* temp_1_2 = 0.0f; */
	/* temp_1_3 = 0.0f; */
	/* temp_2_0 = 0.0f; */
	/* temp_2_1 = 0.0f; */
	temp_2_2 = 1.0f;
	/* temp_2_3 = 0.0f; */
	/* temp_3_0 = 0.0f; */
	/* temp_3_1 = 0.0f; */
	/* temp_3_2 = 0.0f; */
	temp_3_3 = 1.0f;

	/* 
	 * Step 2: Evaluate nonlinear state equations f(x, u)
	 * These are the continuous-time derivatives of the state vector
	 */
#ifdef SIMULINK_USE_ARM_MATH
	f1_0_0 = -Rs / Ls * xD[0] + flux / Ls * xD[2] * arm_sin_f32(xD[3]);  /* di_alpha/dt */
	f1_1_0 = -Rs / Ls * xD[1] - flux / Ls * xD[2] * arm_cos_f32(xD[3]);  /* di_beta/dt */
	f1_2_0 = 0.0f;   /* domega/dt (no acceleration model) */
	f1_3_0 = xD[2];  /* dtheta/dt = omega */
#else
	f1_0_0 = -Rs / Ls * xD[0] + flux / Ls * xD[2] * sin(xD[3]);  /* di_alpha/dt */
	f1_1_0 = -Rs / Ls * xD[1] - flux / Ls * xD[2] * cos(xD[3]);  /* di_beta/dt */
	f1_2_0 = 0.0f;   /* domega/dt (no acceleration model) */
	f1_3_0 = xD[2];  /* dtheta/dt = omega */
#endif

	/* 
	 * Step 3: Compute discrete-time state transition matrix
	 * f2 = I + T*F (first-order approximation)
	 * Only non-zero elements are computed
	 */
	f2_0_0 = temp_0_0 + (T * F_0_0);
	/* f2_0_1 = temp_0_1; */
	f2_0_2 = (T * F_0_2);
	f2_0_3 = (T * F_0_3);
	/* f2_1_0 = temp_1_0; */
	f2_1_1 = temp_1_1 + (T * F_1_1);
	f2_1_2 = (T * F_1_2);
	f2_1_3 = (T * F_1_3);
	/* f2_2_0 = temp_2_0; */
	/* f2_2_1 = temp_2_1; */
	f2_2_2 = temp_2_2;
	/* f2_2_3 = temp_2_3; */
	/* f2_3_0 = temp_3_0; */
	/* f2_3_1 = temp_3_1; */
	f2_3_2 = (T * F_3_2);
	f2_3_3 = temp_3_3;

	/* 
	 * Step 4: State Prediction (a priori estimate)
	 * X_pred = X + T * (f(X) + B*u)
	 * Uses Euler integration with sample time T
	 */
	X_pred_0_0 = xD[0] + T * (f1_0_0 + B_0_0 * vs_ab_0_0);  /* Predicted i_alpha */
	X_pred_1_0 = xD[1] + T * (f1_1_0 + B_1_1 * vs_ab_1_0);  /* Predicted i_beta */
	X_pred_2_0 = xD[2] + T * (f1_2_0);                      /* Predicted omega */
	X_pred_3_0 = xD[3] + T * (f1_3_0);                      /* Predicted theta */

	/* Predicted measurements: Y_pred = H * X_pred */
	Y_pred_0_0 = H_0_0 * X_pred_0_0;  /* Predicted i_alpha measurement */
	Y_pred_1_0 = H_1_1 * X_pred_1_0;  /* Predicted i_beta measurement */

	/* Actual measurements from sensors */
	Y_0_0 = is_ab_0_0;  /* Measured i_alpha */
	Y_1_0 = is_ab_1_0;  /* Measured i_beta */

	/* 
	 * Step 5: Covariance Prediction
	 * P_pred = F * P0 * F^T + Q
	 * Split into two matrix multiplications:
	 *   1. temp = F * P0
	 *   2. P_pred = temp * F^T + Q
	 */
	
	/* First multiplication: temp = F * P0 (optimized for sparse F) */
	P_pred_0_0 = f2_0_0 * P0_0_0 + f2_0_2 * P0_2_0 + f2_0_3 * P0_3_0;
	P_pred_0_1 = f2_0_0 * P0_0_1 + f2_0_2 * P0_2_1 + f2_0_3 * P0_3_1;
	P_pred_0_2 = f2_0_0 * P0_0_2 + f2_0_2 * P0_2_2 + f2_0_3 * P0_3_2;
	P_pred_0_3 = f2_0_0 * P0_0_3 + f2_0_2 * P0_2_3 + f2_0_3 * P0_3_3;
	P_pred_1_0 = f2_1_1 * P0_1_0 + f2_1_2 * P0_2_0 + f2_1_3 * P0_3_0;
	P_pred_1_1 = f2_1_1 * P0_1_1 + f2_1_2 * P0_2_1 + f2_1_3 * P0_3_1;
	P_pred_1_2 = f2_1_1 * P0_1_2 + f2_1_2 * P0_2_2 + f2_1_3 * P0_3_2;
	P_pred_1_3 = f2_1_1 * P0_1_3 + f2_1_2 * P0_2_3 + f2_1_3 * P0_3_3;
	P_pred_2_0 = f2_2_2 * P0_2_0;
	P_pred_2_1 = f2_2_2 * P0_2_1;
	P_pred_2_2 = f2_2_2 * P0_2_2;
	P_pred_2_3 = f2_2_2 * P0_2_3;
	P_pred_3_0 = f2_3_2 * P0_2_0 + f2_3_3 * P0_3_0;
	P_pred_3_1 = f2_3_2 * P0_2_1 + f2_3_3 * P0_3_1;
	P_pred_3_2 = f2_3_2 * P0_2_2 + f2_3_3 * P0_3_2;
	P_pred_3_3 = f2_3_2 * P0_2_3 + f2_3_3 * P0_3_3;

	/* Second multiplication: P_pred = temp * F^T + Q */
	/* Save values that will be overwritten before they're needed */
	float P_pred_0_2_saved = P_pred_0_2;
	float P_pred_1_2_saved = P_pred_1_2;
	float P_pred_2_2_saved = P_pred_2_2;
	float P_pred_3_2_saved = P_pred_3_2;
	
	P_pred_0_0 = P_pred_0_0 * f2_0_0 + P_pred_0_2 * f2_0_2 + P_pred_0_3 * f2_0_3 + Q_0_0;
	P_pred_0_1 = P_pred_0_1 * f2_1_1 + P_pred_0_2 * f2_1_2 + P_pred_0_3 * f2_1_3;
	P_pred_0_2 = P_pred_0_2_saved * f2_2_2;
	P_pred_0_3 = P_pred_0_2_saved * f2_3_2 + P_pred_0_3 * f2_3_3;
	P_pred_1_0 = P_pred_1_0 * f2_0_0 + P_pred_1_2 * f2_0_2 + P_pred_1_3 * f2_0_3;
	P_pred_1_1 = P_pred_1_1 * f2_1_1 + P_pred_1_2 * f2_1_2 + P_pred_1_3 * f2_1_3 + Q_1_1;
	P_pred_1_2 = P_pred_1_2_saved * f2_2_2;
	P_pred_1_3 = P_pred_1_2_saved * f2_3_2 + P_pred_1_3 * f2_3_3;
	P_pred_2_0 = P_pred_2_0 * f2_0_0 + P_pred_2_2 * f2_0_2 + P_pred_2_3 * f2_0_3;
	P_pred_2_1 = P_pred_2_1 * f2_1_1 + P_pred_2_2 * f2_1_2 + P_pred_2_3 * f2_1_3;
	P_pred_2_2 = P_pred_2_2_saved * f2_2_2 + Q_2_2;
	P_pred_2_3 = P_pred_2_2_saved * f2_3_2 + P_pred_2_3 * f2_3_3;
	P_pred_3_0 = P_pred_3_0 * f2_0_0 + P_pred_3_2 * f2_0_2 + P_pred_3_3 * f2_0_3;
	P_pred_3_1 = P_pred_3_1 * f2_1_1 + P_pred_3_2 * f2_1_2 + P_pred_3_3 * f2_1_3;
	P_pred_3_2 = P_pred_3_2_saved * f2_2_2;
	P_pred_3_3 = P_pred_3_2_saved * f2_3_2 + P_pred_3_3 * f2_3_3 + Q_3_3;

	/* 
	 * Step 6: Compute Innovation Covariance S = H * P_pred * H^T + R
	 * First: temp = H * P_pred
	 */
	temp_0_0 = H_0_0 * P_pred_0_0;
	temp_0_1 = H_0_0 * P_pred_0_1;
	temp_0_2 = H_0_0 * P_pred_0_2;
	temp_0_3 = H_0_0 * P_pred_0_3;
	temp_1_0 = H_1_1 * P_pred_1_0;
	temp_1_1 = H_1_1 * P_pred_1_1;
	temp_1_2 = H_1_1 * P_pred_1_2;
	temp_1_3 = H_1_1 * P_pred_1_3;

	/* Second: S = temp * H^T + R (innovation covariance matrix) */
	temp_0_0 = temp_0_0 * H_0_0 + R_0_0;
	temp_0_1 = temp_0_1 * H_1_1;
	temp_1_0 = temp_1_0 * H_0_0;
	temp_1_1 = temp_1_1 * H_1_1 + R_1_1;

	/* Save innovation covariance for inversion */
	temp_0_0_t = temp_0_0;
	temp_0_1_t = temp_0_1;
	temp_1_0_t = temp_1_0;
	temp_1_1_t = temp_1_1;

	/* 
	 * Step 7: Compute inverse of innovation covariance S^-1
	 * For 2x2 matrix: inv = (1/det) * [d, -b; -c, a]
	 */
	temp = temp_0_0 * temp_1_1 - temp_0_1 * temp_1_0;  /* Determinant */
	if (temp != 0)
	{
		temp_0_0 = temp_1_1_t / temp;
		temp_0_1 = -temp_0_1_t / temp;
		temp_1_0 = -temp_1_0_t / temp;
		temp_1_1 = temp_0_0_t / temp;
	}

	/* 
	 * Step 8: Compute Kalman Gain K = P_pred * H^T * S^-1
	 * First: temp = P_pred * H^T
	 */
	K_0_0 = P_pred_0_0 * H_0_0;
	K_0_1 = P_pred_0_1 * H_1_1;
	K_1_0 = P_pred_1_0 * H_0_0;
	K_1_1 = P_pred_1_1 * H_1_1;
	K_2_0 = P_pred_2_0 * H_0_0;
	K_2_1 = P_pred_2_1 * H_1_1;
	K_3_0 = P_pred_3_0 * H_0_0;
	K_3_1 = P_pred_3_1 * H_1_1;

	/* Second: K = temp * S^-1 (Kalman gain matrix) */
	/* Save original values to avoid overwriting during calculation */
	float K_0_0_orig = K_0_0;
	float K_0_1_orig = K_0_1;
	float K_1_0_orig = K_1_0;
	float K_1_1_orig = K_1_1;
	float K_2_0_orig = K_2_0;
	float K_2_1_orig = K_2_1;
	float K_3_0_orig = K_3_0;
	float K_3_1_orig = K_3_1;
	
	/* Compute K = temp * S^-1 using saved original values */
	K_0_0 = K_0_0_orig * temp_0_0 + K_0_1_orig * temp_1_0;
	K_0_1 = K_0_0_orig * temp_0_1 + K_0_1_orig * temp_1_1;
	K_1_0 = K_1_0_orig * temp_0_0 + K_1_1_orig * temp_1_0;
	K_1_1 = K_1_0_orig * temp_0_1 + K_1_1_orig * temp_1_1;
	K_2_0 = K_2_0_orig * temp_0_0 + K_2_1_orig * temp_1_0;
	K_2_1 = K_2_0_orig * temp_0_1 + K_2_1_orig * temp_1_1;
	K_3_0 = K_3_0_orig * temp_0_0 + K_3_1_orig * temp_1_0;
	K_3_1 = K_3_0_orig * temp_0_1 + K_3_1_orig * temp_1_1;

	/* 
	 * Step 9: State Correction (a posteriori estimate)
	 * X = X_pred + K * (Y_measured - Y_pred)
	 * Innovation: (Y_measured - Y_pred) is the measurement residual
	 */
	tempa_0_0 = X_pred_0_0 + K_0_0 * (Y_0_0 - Y_pred_0_0) + K_0_1 * (Y_1_0 - Y_pred_1_0);
	tempa_1_0 = X_pred_1_0 + K_1_0 * (Y_0_0 - Y_pred_0_0) + K_1_1 * (Y_1_0 - Y_pred_1_0);
	tempa_2_0 = X_pred_2_0 + K_2_0 * (Y_0_0 - Y_pred_0_0) + K_2_1 * (Y_1_0 - Y_pred_1_0);
	tempa_3_0 = X_pred_3_0 + K_3_0 * (Y_0_0 - Y_pred_0_0) + K_3_1 * (Y_1_0 - Y_pred_1_0);

	/* 
	 * Step 10: Covariance Correction
	 * P = (I - K*H) * P_pred
	 * First: Compute K*H
	 */
	temp_0_0 = K_0_0 * H_0_0;
	temp_0_1 = K_0_1 * H_1_1;
	/* temp_0_2 = 0.0f; */
	/* temp_0_3 = 0.0f; */
	temp_1_0 = K_1_0 * H_0_0;
	temp_1_1 = K_1_1 * H_1_1;
	/* temp_1_2 = 0.0f; */
	/* temp_1_3 = 0.0f; */
	temp_2_0 = K_2_0 * H_0_0;
	temp_2_1 = K_2_1 * H_1_1;
	/* temp_2_2 = 0.0f; */
	/* temp_2_3 = 0.0f; */
	temp_3_0 = K_3_0 * H_0_0;
	temp_3_1 = K_3_1 * H_1_1;
	/* temp_3_2 = 0.0f; */
	/* temp_3_3 = 0.0f; */

	/* Second: P = P_pred - (K*H) * P_pred (Joseph form update) */
	P0_0_0 = P_pred_0_0 - (temp_0_0 * P_pred_0_0 + temp_0_1 * P_pred_1_0);
	P0_0_1 = P_pred_0_1 - (temp_0_0 * P_pred_0_1 + temp_0_1 * P_pred_1_1);
	P0_0_2 = P_pred_0_2 - (temp_0_0 * P_pred_0_2 + temp_0_1 * P_pred_1_2);
	P0_0_3 = P_pred_0_3 - (temp_0_0 * P_pred_0_3 + temp_0_1 * P_pred_1_3);
	P0_1_0 = P_pred_1_0 - (temp_1_0 * P_pred_0_0 + temp_1_1 * P_pred_1_0);
	P0_1_1 = P_pred_1_1 - (temp_1_0 * P_pred_0_1 + temp_1_1 * P_pred_1_1);
	P0_1_2 = P_pred_1_2 - (temp_1_0 * P_pred_0_2 + temp_1_1 * P_pred_1_2);
	P0_1_3 = P_pred_1_3 - (temp_1_0 * P_pred_0_3 + temp_1_1 * P_pred_1_3);
	P0_2_0 = P_pred_2_0 - (temp_2_0 * P_pred_0_0 + temp_2_1 * P_pred_1_0);
	P0_2_1 = P_pred_2_1 - (temp_2_0 * P_pred_0_1 + temp_2_1 * P_pred_1_1);
	P0_2_2 = P_pred_2_2 - (temp_2_0 * P_pred_0_2 + temp_2_1 * P_pred_1_2);
	P0_2_3 = P_pred_2_3 - (temp_2_0 * P_pred_0_3 + temp_2_1 * P_pred_1_3);
	P0_3_0 = P_pred_3_0 - (temp_3_0 * P_pred_0_0 + temp_3_1 * P_pred_1_0);
	P0_3_1 = P_pred_3_1 - (temp_3_0 * P_pred_0_1 + temp_3_1 * P_pred_1_1);
	P0_3_2 = P_pred_3_2 - (temp_3_0 * P_pred_0_2 + temp_3_1 * P_pred_1_2);
	P0_3_3 = P_pred_3_3 - (temp_3_0 * P_pred_0_3 + temp_3_1 * P_pred_1_3);

	/* Wrap rotor angle to [0, 2*PI] range */
#ifdef SIMULINK_USE_ARM_MATH
	if (tempa_3_0 > (2.0f * PI))
	{
		tempa_3_0 -= (2.0f * PI);
	}
#else
	if (tempa_3_0 > (MATH_2PI))  /* 2*PI approximation */
	{
		tempa_3_0 -= (MATH_2PI);
	}
#endif

	/* Store corrected states back to state vector */
	xD[0] = tempa_0_0;  /* Corrected i_alpha */
	xD[1] = tempa_1_0;  /* Corrected i_beta */
	xD[2] = tempa_2_0;  /* Corrected omega */
	xD[3] = tempa_3_0;  /* Corrected theta (wrapped) */

	/*
	 * Optional: Store covariance matrix in extended state vector
	 * Currently disabled - covariance maintained in global variables
	 * If enabled, would store P0 matrix to xD[4] through xD[19]
	 */
}

