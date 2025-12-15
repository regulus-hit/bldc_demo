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

void hall_sensor_c_tim2_sub(void);

#endif
