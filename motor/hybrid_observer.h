/**********************************
 * Hybrid Hall+EKF Observer for BLDC Motor
 * Combines Hall sensor measurements with EKF state estimation
 * for improved robustness and accuracy
 **********************************/
#ifndef __HYBRID_OBSERVER_H__
#define __HYBRID_OBSERVER_H__

#include <stdint.h>

/**
 * @brief Hybrid Observer State Structure
 * 
 * Maintains internal state for the hybrid Hall+EKF observer:
 * - Fused position and speed estimates
 * - Previous Hall sensor readings for change detection
 * - Fusion filter states
 */
typedef struct
{
	float position_fused;        /* Fused rotor position estimate (rad) */
	float speed_fused;           /* Fused rotor speed estimate (rad/s) */
	float hall_position_prev;    /* Previous Hall position for validation */
	float hall_speed_prev;       /* Previous Hall speed for filtering */
	uint8_t initialized;         /* Initialization flag */
} HYBRID_OBSERVER_STATE;

/* Global hybrid observer state */
extern HYBRID_OBSERVER_STATE Hybrid_Observer_State;

/**
 * @brief Initialize Hybrid Observer
 * 
 * Initializes the hybrid observer state variables and filters.
 * Must be called once before starting motor operation.
 */
void hybrid_observer_initialize(void);

/**
 * @brief Update Hybrid Observer
 * 
 * Fuses EKF state estimates with Hall sensor measurements using
 * complementary filtering and measurement validation.
 * 
 * Algorithm:
 * 1. Validate Hall sensor measurements (speed threshold, position error)
 * 2. Compute position error between EKF and Hall
 * 3. Apply complementary filter for smooth fusion
 * 4. Handle angle wrapping (0 to 2*PI)
 * 5. Output fused position and speed
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
);

/**
 * @brief Normalize angle to [0, 2*PI] range
 * 
 * @param angle Input angle (rad)
 * @return Normalized angle in [0, 2*PI] range
 */
float normalize_angle(float angle);

/**
 * @brief Compute shortest angular difference
 * 
 * Computes the shortest signed angular difference from angle1 to angle2,
 * accounting for 2*PI wraparound.
 * 
 * @param angle1 First angle (rad)
 * @param angle2 Second angle (rad)
 * @return Shortest angular difference (rad), range [-PI, PI]
 */
float angle_difference(float angle1, float angle2);

#endif /* __HYBRID_OBSERVER_H__ */
