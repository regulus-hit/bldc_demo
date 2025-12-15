/**********************************
 * Hybrid Hall+EKF Observer Implementation
 * 
 * References:
 * - TI MotorWare: InstaSPIN-FOC with sensor fusion
 * - SimpleFOC: Hybrid angle sensor implementation
 * - STMicroelectronics: Sensorless/sensored switching strategies
 **********************************/
#include "main.h"
#include "hybrid_observer.h"
#include "foc_define_parameter.h"
#include <math.h>

/* Hybrid observer state */
HYBRID_OBSERVER_STATE Hybrid_Observer_State = {0};

/**
 * @brief Initialize Hybrid Observer
 * 
 * Initializes state variables to safe defaults.
 */
void hybrid_observer_initialize(void)
{
	Hybrid_Observer_State.position_fused = 0.0f;
	Hybrid_Observer_State.speed_fused = 0.0f;
	Hybrid_Observer_State.hall_position_prev = 0.0f;
	Hybrid_Observer_State.hall_speed_prev = 0.0f;
	Hybrid_Observer_State.initialized = 0;
}

/**
 * @brief Normalize angle to [0, 2*PI] range
 * 
 * Uses modulo arithmetic with fmod for floating point.
 * Handles negative angles correctly.
 * 
 * @param angle Input angle (rad)
 * @return Normalized angle in [0, 2*PI] range
 */
float normalize_angle(float angle)
{
	/* Use fmodf for floating point modulo */
	angle = fmodf(angle, MATH_2PI);
	
	/* Handle negative angles */
	if (angle < 0.0f)
	{
		angle += MATH_2PI;
	}
	
	return angle;
}

/**
 * @brief Compute shortest angular difference
 * 
 * Computes the shortest signed difference from angle1 to angle2.
 * Result is in range [-PI, PI] to represent the shortest rotation direction.
 * 
 * Example:
 * - angle1 = 0.1 rad, angle2 = 6.2 rad (near 2*PI)
 * - Direct difference: 6.1 rad clockwise
 * - Shortest path: -0.183 rad counterclockwise (wraps through 0)
 * 
 * @param angle1 First angle (rad)
 * @param angle2 Second angle (rad)
 * @return Shortest angular difference (rad), range [-PI, PI]
 */
float angle_difference(float angle1, float angle2)
{
	float diff = angle2 - angle1;
	
	/* Normalize to [-PI, PI] range */
	while (diff > MATH_PI)
	{
		diff -= MATH_2PI;
	}
	while (diff < -MATH_PI)
	{
		diff += MATH_2PI;
	}
	
	return diff;
}

/**
 * @brief Update Hybrid Observer
 * 
 * Implements complementary filtering between EKF and Hall sensor measurements.
 * 
 * Algorithm Design (following TI MotorWare and SimpleFOC approaches):
 * 
 * 1. Low-Speed Operation (speed < HYBRID_HALL_MIN_SPEED):
 *    - Hall timing becomes unreliable due to low update rate
 *    - Use pure EKF estimation
 *    - EKF tracks current-based back-EMF for position/speed
 * 
 * 2. High-Speed Operation (speed >= HYBRID_HALL_MIN_SPEED):
 *    - Hall sensors provide reliable position snapshots every 60°
 *    - EKF provides smooth continuous estimation between Hall edges
 *    - Complementary filter combines both:
 *      * Position: fused = EKF + weight * (Hall - EKF)
 *      * Speed: fused = (1-weight) * EKF + weight * Hall
 * 
 * 3. Divergence Detection:
 *    - If position error > HYBRID_HALL_MAX_POSITION_ERROR
 *    - EKF has likely diverged (wrong pole count, parameter error, etc.)
 *    - Trust Hall sensor directly and reset EKF confidence
 * 
 * Benefits:
 * - Smooth position/speed at all speeds (EKF interpolates between Hall edges)
 * - Robust against EKF divergence (Hall provides absolute reference)
 * - Reduced Hall quantization noise (EKF smooths 60° steps)
 * - Better startup performance (Hall initializes EKF)
 * 
 * @param ekf_position EKF estimated position (rad)
 * @param ekf_speed EKF estimated speed (rad/s)
 * @param hall_position Hall sensor position (rad)
 * @param hall_speed Hall sensor speed (rad/s)
 * @param fused_position Output: Fused position estimate (rad)
 * @param fused_speed Output: Fused speed estimate (rad/s)
 */
void hybrid_observer_update(
	float ekf_position,
	float ekf_speed,
	float hall_position,
	float hall_speed,
	float *fused_position,
	float *fused_speed
)
{
	float position_error;
	float speed_abs;
	
	/* First-time initialization: use Hall sensor readings */
	if (!Hybrid_Observer_State.initialized)
	{
		Hybrid_Observer_State.position_fused = hall_position;
		Hybrid_Observer_State.speed_fused = hall_speed;
		Hybrid_Observer_State.hall_position_prev = hall_position;
		Hybrid_Observer_State.hall_speed_prev = hall_speed;
		Hybrid_Observer_State.initialized = 1;
		
		*fused_position = hall_position;
		*fused_speed = hall_speed;
		return;
	}
	
	/* Compute absolute speed for threshold check */
	speed_abs = fabsf(ekf_speed);
	
	/* 
	 * Low-speed operation: Use pure EKF
	 * Hall sensor timing becomes unreliable at very low speeds
	 * (long periods between Hall edges, susceptible to noise)
	 */
	if (speed_abs < HYBRID_HALL_MIN_SPEED)
	{
		/* Use EKF estimates directly */
		Hybrid_Observer_State.position_fused = ekf_position;
		Hybrid_Observer_State.speed_fused = ekf_speed;
		
		*fused_position = ekf_position;
		*fused_speed = ekf_speed;
		return;
	}
	
	/* 
	 * Compute position error between EKF and Hall sensor
	 * Use shortest angular path for proper wrapping
	 */
	position_error = angle_difference(ekf_position, hall_position);
	
	/* 
	 * Divergence detection: Check if EKF has drifted too far from Hall
	 * If error exceeds one Hall sector (60°), EKF is likely diverged
	 */
	if (fabsf(position_error) > HYBRID_HALL_MAX_POSITION_ERROR)
	{
		/* EKF diverged - trust Hall sensor directly */
		Hybrid_Observer_State.position_fused = hall_position;
		Hybrid_Observer_State.speed_fused = hall_speed;
		
		*fused_position = hall_position;
		*fused_speed = hall_speed;
		
		/* Update previous values */
		Hybrid_Observer_State.hall_position_prev = hall_position;
		Hybrid_Observer_State.hall_speed_prev = hall_speed;
		return;
	}
	
	/* 
	 * Normal operation: Complementary filter fusion
	 * 
	 * Position fusion using weighted correction:
	 *   fused = ekf + weight * (hall - ekf)
	 *   fused = (1-weight) * ekf + weight * hall
	 * 
	 * This provides smooth tracking:
	 * - EKF provides high-frequency smooth interpolation between Hall edges
	 * - Hall provides low-frequency absolute position correction
	 * - Similar to complementary filter in IMU sensor fusion
	 */
	Hybrid_Observer_State.position_fused = ekf_position + 
		HYBRID_HALL_POSITION_WEIGHT * position_error;
	
	/* Normalize fused position to [0, 2*PI] */
	Hybrid_Observer_State.position_fused = 
		normalize_angle(Hybrid_Observer_State.position_fused);
	
	/*
	 * Speed fusion using weighted average:
	 *   fused = (1-weight) * ekf + weight * hall
	 * 
	 * Benefits:
	 * - EKF speed is smooth but may drift over time
	 * - Hall speed provides absolute reference but is noisy (computed from edge timing)
	 * - Weighted average combines smooth EKF with Hall accuracy
	 */
	Hybrid_Observer_State.speed_fused = 
		(1.0f - HYBRID_HALL_SPEED_WEIGHT) * ekf_speed + 
		HYBRID_HALL_SPEED_WEIGHT * hall_speed;
	
	/* Output fused estimates */
	*fused_position = Hybrid_Observer_State.position_fused;
	*fused_speed = Hybrid_Observer_State.speed_fused;
	
	/* Update previous Hall values for next iteration */
	Hybrid_Observer_State.hall_position_prev = hall_position;
	Hybrid_Observer_State.hall_speed_prev = hall_speed;
}
