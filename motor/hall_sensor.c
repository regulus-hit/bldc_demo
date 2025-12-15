/**********************************
 * Hall Sensor Interface for BLDC Motor
 * Provides rotor position and speed feedback
 * 
 * References:
 * - TI MotorWare: Velocity-based Hall interpolation
 * - SimpleFOC: Hall sensor edge detection and angle estimation
 * - Microchip AN4413: BLDC Motor Control with Hall Sensors
 **********************************/
#include "main.h"
#include "hall_sensor.h"

uint8_t hall_read_temp;

float hall_angle;
float hall_angle_add;
float hall_speed;

#ifdef ENABLE_HALL_INTERPOLATION
/* Hall sensor interpolation state */
float hall_angle_interpolated = 0.0f;
float hall_misalignment_offset = HALL_MISALIGNMENT_OFFSET_INITIAL;
uint32_t hall_edge_timestamp = 0;
uint32_t hall_edge_interval = 0;
float hall_last_edge_angle = 0.0f;     /* Angle at last Hall edge */
uint8_t hall_interpolation_initialized = 0;

/**
 * @brief Normalize angle to [0, 2*PI] range efficiently
 * 
 * Uses fmodf for efficient normalization. In typical FOC operation,
 * angles are already near [0, 2π], so this is faster than while loops.
 * 
 * @param angle Input angle (radians)
 * @return Normalized angle in [0, 2*PI] range
 */
static inline float normalize_angle_to_2pi(float angle)
{
	/* Use fmodf for efficient normalization */
	angle = fmodf(angle, MATH_2PI);
	
	/* Handle negative angles */
	if (angle < 0.0f)
	{
		angle += MATH_2PI;
	}
	
	return angle;
}

/**
 * @brief Initialize Hall sensor interpolation
 */
void hall_interpolation_initialize(void)
{
	hall_angle_interpolated = 0.0f;
	hall_misalignment_offset = HALL_MISALIGNMENT_OFFSET_INITIAL;
	hall_edge_timestamp = 0;
	hall_edge_interval = 0;
	hall_last_edge_angle = 0.0f;
	hall_interpolation_initialized = 1;
}

/**
 * @brief Update Hall sensor interpolation
 * 
 * Implements velocity-based linear interpolation between Hall edges
 * following TI MotorWare and SimpleFOC approaches.
 * 
 * Algorithm:
 * - At low speeds (< HALL_INTERPOLATION_MIN_SPEED): Use pure Hall position
 * - At high speeds (>= HALL_INTERPOLATION_MIN_SPEED): Interpolate between edges
 *   - Position = last_edge_angle + velocity * time_since_edge
 *   - Apply misalignment correction
 * 
 * Note: HALL_TIM is configured in TIM_SlaveMode_Reset, so the counter resets
 * to zero on each Hall edge. Therefore, TIM_GetCounter() directly gives us
 * the time elapsed since the last Hall edge.
 * 
 * @param timer_count Current Hall timer counter value (time since last edge)
 */
void hall_interpolation_update(uint32_t timer_count)
{
	float speed_abs;
	float time_since_edge;
	float interpolated_increment;
	
	/* Check if interpolation is initialized */
	if (!hall_interpolation_initialized)
	{
		hall_interpolation_initialize();
	}
	
	/* Get absolute speed for threshold check */
	speed_abs = fabsf(hall_speed * MATH_2PI);  /* Convert Hz to rad/s */
	
	/* Low-speed mode: Use pure Hall sensor position (no interpolation) */
	if (speed_abs < HALL_INTERPOLATION_MIN_SPEED)
	{
		/* Use raw Hall angle with misalignment correction */
		hall_angle_interpolated = normalize_angle_to_2pi(hall_angle + hall_misalignment_offset);
		return;
	}
	
	/* High-speed mode: Interpolate position between Hall edges */
	
	/* Timer count directly gives time since last Hall edge (counter resets on edges)
	 * This is more reliable than timestamp subtraction */
	time_since_edge = (float)timer_count;
	
	/* Calculate interpolated angle increment based on velocity
	 * Angular velocity = hall_speed * 2*PI (rad/s electrical)
	 * Time conversion: timer ticks to seconds = ticks / HALL_TIM_CLOCK
	 * Increment = velocity * time = (hall_speed * 2*PI) * (time_since_edge / HALL_TIM_CLOCK)
	 */
	interpolated_increment = (hall_speed * MATH_2PI) * (time_since_edge / (float)HALL_TIM_CLOCK);
	
	/* Interpolated angle = last edge angle + increment + misalignment offset */
	hall_angle_interpolated = normalize_angle_to_2pi(
		hall_last_edge_angle + interpolated_increment + hall_misalignment_offset
	);
	
#ifdef ENABLE_HALL_MISALIGNMENT_CORRECTION
	/**
	 * Automatic Misalignment Offset Correction
	 * 
	 * Hall sensors may be physically misaligned to motor UVW phases.
	 * This causes a constant angle offset between Hall readings and true rotor position.
	 * 
	 * Detection method (similar to TI MotorWare approach):
	 * - At Hall edge transitions, compare interpolated position with new Hall reading
	 * - The difference indicates misalignment
	 * - Filter this error slowly to adapt offset over time
	 * 
	 * Note: Only update when speed is stable (not during acceleration/deceleration)
	 */
	
	/* Check if we're at high speed with reliable Hall timing */
	if (speed_abs > HALL_INTERPOLATION_MIN_SPEED * 1.5f)
	{
		/* At the moment of Hall edge, interpolated angle should match Hall angle
		 * If they differ consistently, it indicates misalignment
		 * We update this correction slowly over many cycles in hall_sensor_c_tim2_sub() */
	}
#endif
}
#endif  /* ENABLE_HALL_INTERPOLATION */

/**
 * @brief Hall Sensor Timer Interrupt Handler
 * 
 * Triggered on Hall sensor state changes to update rotor position and speed.
 * Hall sensors provide position feedback every 60 electrical degrees.
 * 
 * The three Hall sensors (A, B, C) create 6 distinct states per electrical cycle,
 * allowing 60-degree resolution position sensing.
 * 
 * When ENABLE_HALL_INTERPOLATION is defined:
 * - Records edge timestamp and interval for velocity-based interpolation
 * - Updates misalignment offset correction (if enabled)
 */
void hall_sensor_c_tim2_sub(void)
{
	float temp;
	if (TIM_GetFlagStatus(HALL_TIM, TIM_FLAG_CC1) == SET)
	{
		/* Capture edge timestamp (read once for consistency) */
		uint32_t current_timestamp = TIM_GetCapture1(HALL_TIM);
		temp = (float)current_timestamp;
		
		/* Calculate speed and angle increment from capture period */
		hall_angle_add = (float)HALL_ANGLE_FACTOR / temp;
		hall_speed = (float)HALL_SPEED_FACTOR / temp;
		
#ifdef ENABLE_HALL_INTERPOLATION
		/* Save edge timing information for interpolation */
		hall_edge_interval = current_timestamp - hall_edge_timestamp;
		hall_edge_timestamp = current_timestamp;
#endif
		
		/* Read all three Hall sensor states */
		hall_read_temp = GPIO_ReadInputDataBit(HALL_CH3_GPIO_PORT, HALL_CH3_PIN);
		hall_read_temp |= GPIO_ReadInputDataBit(HALL_CH2_GPIO_PORT, HALL_CH2_PIN) << 1;
		hall_read_temp |= GPIO_ReadInputDataBit(HALL_CH1_GPIO_PORT, HALL_CH1_PIN) << 2;

		/* Decode Hall sensor state to rotor angle (6 states per electrical cycle) */
		if (hall_read_temp == 0x05)
		{
			hall_angle = 0.0f + PHASE_SHIFT_ANGLE;					/* State 101: 0 degrees */
		}
		else if (hall_read_temp == 0x04)
		{
			hall_angle = (MATH_PI / 3.0f) + PHASE_SHIFT_ANGLE;		/* State 100: 60 degrees */
		}
		else if (hall_read_temp == 0x06)
		{
			hall_angle = (MATH_PI * 2.0f / 3.0f) + PHASE_SHIFT_ANGLE;	/* State 110: 120 degrees */
		}
		else if (hall_read_temp == 0x02)
		{
			hall_angle = MATH_PI + PHASE_SHIFT_ANGLE;				/* State 010: 180 degrees */
		}
		else if (hall_read_temp == 0x03)
		{
			hall_angle = (MATH_PI * 4.0f / 3.0f) + PHASE_SHIFT_ANGLE;	/* State 011: 240 degrees */
		}
		else if (hall_read_temp == 0x01)
		{
			hall_angle = (MATH_PI * 5.0f / 3.0f) + PHASE_SHIFT_ANGLE;	/* State 001: 300 degrees */
		}
		
		/* Wrap angle to [0, 2*PI] range */
		if (hall_angle < 0.0f)
		{
			hall_angle += MATH_2PI;
		}
		else if (hall_angle > MATH_2PI)
		{
			hall_angle -= MATH_2PI;
		}

#ifdef ENABLE_HALL_INTERPOLATION
		/* Save edge angle for interpolation */
		hall_last_edge_angle = hall_angle;
		
#ifdef ENABLE_HALL_MISALIGNMENT_CORRECTION
		/**
		 * Automatic Misalignment Offset Correction at Hall Edge
		 * 
		 * At Hall edge transitions, compare interpolated position with actual Hall reading.
		 * If there's a consistent error, it indicates Hall sensor misalignment.
		 * 
		 * Algorithm:
		 * 1. At Hall edge, compare hall_angle_interpolated with new hall_angle
		 * 2. Calculate error (considering angle wrapping)
		 * 3. Low-pass filter the error to update misalignment offset
		 * 4. Limit correction to reasonable range
		 */
		if (hall_interpolation_initialized)
		{
			float speed_abs = fabsf(hall_speed * MATH_2PI);
			
			/* Only update correction at stable high speeds */
			if (speed_abs > HALL_INTERPOLATION_MIN_SPEED * 1.5f)
			{
				/* Calculate position error at Hall edge (shortest angular path) */
				float position_error = hall_angle - hall_angle_interpolated;
				
				/* Normalize error to [-PI, PI] range using efficient approach */
				position_error = fmodf(position_error + MATH_PI, MATH_2PI) - MATH_PI;
				if (position_error < -MATH_PI)
				{
					position_error += MATH_2PI;
				}
				
				/* Low-pass filter to slowly adapt misalignment offset
				 * offset = offset + alpha * error
				 * This gradually corrects systematic misalignment */
				hall_misalignment_offset += HALL_MISALIGNMENT_FILTER_COEFF * position_error;
				
				/* Limit correction to prevent runaway */
				if (hall_misalignment_offset > HALL_MISALIGNMENT_MAX_CORRECTION)
				{
					hall_misalignment_offset = HALL_MISALIGNMENT_MAX_CORRECTION;
				}
				else if (hall_misalignment_offset < -HALL_MISALIGNMENT_MAX_CORRECTION)
				{
					hall_misalignment_offset = -HALL_MISALIGNMENT_MAX_CORRECTION;
				}
			}
		}
#endif  /* ENABLE_HALL_MISALIGNMENT_CORRECTION */
#endif  /* ENABLE_HALL_INTERPOLATION */

		TIM_ClearFlag(HALL_TIM, TIM_FLAG_CC1);
	}
}
