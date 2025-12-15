/**********************************
 * External Interrupt Handlers
 * User button input processing
 **********************************/
#include "main.h"
#include "exti.h"

uint8_t key1_flag;
uint8_t key2_flag;
uint8_t key3_flag;

/**
 * @brief External Interrupt Line 2 Handler (Key3)
 * Speed increase button
 */
void EXTI2_IRQHandler(void)
{
	if (EXTI_GetITStatus(KEY_3_EXTI_LINE) != RESET)
	{
		key3_flag = 1;
		EXTI_ClearITPendingBit(KEY_3_EXTI_LINE);
	}
}

/**
 * @brief External Interrupt Lines 5-9 Handler (Key2)
 * Speed decrease button
 */
void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(KEY_2_EXTI_LINE) != RESET)
	{
		key2_flag = 1;
		EXTI_ClearITPendingBit(KEY_2_EXTI_LINE);
	}
}

/**
 * @brief External Interrupt Line 4 Handler (Key1)
 * Motor start/stop and direction change button
 * Rising edge: clear flag, Falling edge: set flag
 */
void EXTI4_IRQHandler(void)
{
	if (EXTI_GetITStatus(KEY_1_EXTI_LINE) != RESET)
	{
		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == Bit_SET)
		{
			key1_flag = 0;	/* Rising edge */
		}
		else
		{
			key1_flag = 1;	/* Falling edge - button pressed */
		}
		EXTI_ClearITPendingBit(KEY_1_EXTI_LINE);
	}
}
