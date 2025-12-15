/**
 * @file	user/main.c
 * @brief	Main entry and simple timing helpers for the BLDC demo.
 *
 * Tidied indentation (tabs) and added Doxygen-style comments for exported
 * functions. No behavior changes were made.
 */

#include "main.h"
#include "UpperComputer.h"
#include "USART2.h"

static __IO uint32_t uwTimingDelay;

/**
 * @brief	Main program entry point.
 *
 * Initializes peripherals, drivers and the FOC algorithm, then enters the
 * main loop where periodic protections and display handling run.
 *
 * @return	Does not return.
 */
int main(void)
{
	/* 初始化 */
	Usart2_config();
	USART_ConfigInterrupt();

	hard_init();
	OLED_Init();
	drv8301_init();
	/* pc_communication_init(); */
	foc_algorithm_initialize();

	if (get_offset_flag == 0)
	{
		get_offset_flag = 1;
		TIM_CtrlPWMOutputs(PWM_TIM, ENABLE);
	}
	GPIO_SetBits(DRV8301_ENGATE_GPIO_PORT, DRV8301_ENGATE_PIN);

	while (1) /* 主循环 */
	{
		drv8301_protection();
		oled_display_handle();
	}
}

/**
 * @brief	Set a millisecond delay counter.
 *
 * This function sets the global timing delay counter. The expectation is
 * that a SysTick or timer interrupt calls TimingDelay_Decrement periodically
 * to count down this counter.
 *
 * @param[in] nTime  Delay value in ticks (typically milliseconds).
 */
void Delay(__IO uint32_t nTime)
{
	uwTimingDelay = nTime;

	while (uwTimingDelay != 0)
		;
}

/**
 * @brief	Decrement the global timing delay counter.
 *
 * Intended to be invoked from a millisecond SysTick handler or similar.
 * Decrements uwTimingDelay if it is non-zero.
 */
void TimingDelay_Decrement(void)
{
	if (uwTimingDelay != 0x00)
	{
		uwTimingDelay--;
	}
}

#ifdef USE_FULL_ASSERT

/**
 * @brief	Report the name of the source file and the source line number
 *		where an assert_param error has occurred.
 *
 * This function is called by the assert macro (if enabled). It enters an
 * infinite loop so the failure can be observed during debugging.
 *
 * @param[in] file  Source file name where the assert failed.
 * @param[in] line  Line number in the source file.
 */
void assert_failed(uint8_t* file, uint32_t line)
{

	while (1)
	{
	}
}
#endif
