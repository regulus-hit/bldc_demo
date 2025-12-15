/**********************************
 * Hall Sensor Interface for BLDC Motor
 * Provides rotor position and speed feedback
 **********************************/
#include "main.h"
#include "hall_sensor.h"

uint8_t hall_read_temp;

float hall_angle;
float hall_angle_add;
float hall_speed;

/**
 * @brief Hall Sensor Timer Interrupt Handler
 * 
 * Triggered on Hall sensor state changes to update rotor position and speed.
 * Hall sensors provide position feedback every 60 electrical degrees.
 * 
 * The three Hall sensors (A, B, C) create 6 distinct states per electrical cycle,
 * allowing 60-degree resolution position sensing.
 */
void hall_sensor_c_tim2_sub(void)
{
	float temp;
	if (TIM_GetFlagStatus(HALL_TIM, TIM_FLAG_CC1) == SET)
	{
		/* Calculate speed and angle increment from capture period */
		temp = (float)(TIM_GetCapture1(HALL_TIM));
		hall_angle_add = (float)HALL_ANGLE_FACTOR / (float)(temp);
		hall_speed = (float)HALL_SPEED_FACTOR / (float)(temp);
		
		/* Read all three Hall sensor states */
		hall_read_temp = GPIO_ReadInputDataBit(HALL_CH3_GPIO_PORT, HALL_CH3_PIN);
		hall_read_temp |= GPIO_ReadInputDataBit(HALL_CH2_GPIO_PORT, HALL_CH2_PIN) << 1;
		hall_read_temp |= GPIO_ReadInputDataBit(HALL_CH1_GPIO_PORT, HALL_CH1_PIN) << 2;

		/* Decode Hall sensor state to rotor angle (6 states per electrical cycle) */
		if (hall_read_temp == 0x05)
		{
			hall_angle = 0.0f + PHASE_SHIFT_ANGLE;				/* State 101: 0 degrees */
		}
		else if (hall_read_temp == 0x04)
		{
			hall_angle = (PI / 3.0f) + PHASE_SHIFT_ANGLE;		/* State 100: 60 degrees */
		}
		else if (hall_read_temp == 0x06)
		{
			hall_angle = (PI * 2.0f / 3.0f) + PHASE_SHIFT_ANGLE;	/* State 110: 120 degrees */
		}
		else if (hall_read_temp == 0x02)
		{
			hall_angle = PI + PHASE_SHIFT_ANGLE;				/* State 010: 180 degrees */
		}
		else if (hall_read_temp == 0x03)
		{
			hall_angle = (PI * 4.0f / 3.0f) + PHASE_SHIFT_ANGLE;	/* State 011: 240 degrees */
		}
		else if (hall_read_temp == 0x01)
		{
			hall_angle = (PI * 5.0f / 3.0f) + PHASE_SHIFT_ANGLE;	/* State 001: 300 degrees */
		}
		
		/* Wrap angle to [0, 2*PI] range */
		if (hall_angle < 0.0f)
		{
			hall_angle += 2.0f * PI;
		}
		else if (hall_angle > (2.0f * PI))
		{
			hall_angle -= 2.0f * PI;
		}

		TIM_ClearFlag(HALL_TIM, TIM_FLAG_CC1);
	}
}
