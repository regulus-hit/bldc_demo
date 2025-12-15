#ifndef __HALL_SENSOR_H__
#define __HALL_SENSOR_H__

#include <stdint.h>

/* Hall sensor timer configuration */
#define HALL_TIM_CLOCK (uint32_t)90000000
#define HALL_SAMPLE_FREQ (uint32_t)10000

/* Phase shift compensation: 60 degrees (electrical) */
#define PHASE_SHIFT_ANGLE (float)(60.0f/360.0f*2.0f*PI)

/* Conversion factors for angle and speed calculation */
#define HALL_ANGLE_FACTOR (float)((float)HALL_TIM_CLOCK/(float)HALL_SAMPLE_FREQ*PI/3.0f)
#define HALL_SPEED_FACTOR (float)((float)HALL_TIM_CLOCK/6.0f)

extern float hall_angle;		/* Rotor angle from Hall sensors */
extern float hall_angle_add;	/* Angle increment per sample */
extern float hall_speed;		/* Rotor speed from Hall sensors */

#ifdef ENABLE_HALL_INTERPOLATION
/* Hall sensor interpolation state */
extern float hall_angle_interpolated;  /* Interpolated rotor angle (rad) */
extern float hall_misalignment_offset; /* Detected misalignment offset (rad) */
extern uint32_t hall_edge_timestamp;   /* Timestamp of last Hall edge (system ticks) */
extern uint32_t hall_edge_interval;    /* Time between last two Hall edges (system ticks) */

/**
 * @brief Initialize Hall sensor interpolation
 * 
 * Initializes interpolation state variables and misalignment correction.
 * Must be called once before motor operation.
 */
void hall_interpolation_initialize(void);

/**
 * @brief Update Hall sensor interpolation
 * 
 * Called at control loop frequency (10kHz) to interpolate position between Hall edges.
 * Uses velocity-based linear interpolation for higher resolution position feedback.
 * 
 * Algorithm:
 * 1. Calculate time elapsed since last Hall edge
 * 2. Interpolate position: angle = last_edge_angle + velocity * time_elapsed
 * 3. Apply misalignment offset correction if enabled
 * 4. Normalize angle to [0, 2*PI] range
 * 
 * @param current_time Current system timestamp (microseconds or timer ticks)
 */
void hall_interpolation_update(uint32_t current_time);
#endif

void hall_sensor_c_tim2_sub(void);

#endif
