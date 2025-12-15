/**********************************
 * Main Program Entry Point
 * BLDC Motor FOC Control System
 **********************************/
#include "main.h"
#include "UpperComputer.h"
#include "USART2.h"

static __IO uint32_t uwTimingDelay;

/**
 * @brief Main Program
 * 
 * Initialization sequence:
 * 1. UART communication for PC telemetry
 * 2. Hardware peripherals (GPIO, timers, ADC, PWM)
 * 3. OLED display
 * 4. DRV8301 gate driver
 * 5. FOC algorithm components
 * 6. ADC offset calibration
 * 
 * Main loop monitors fault conditions and updates display.
 * Motor control runs in ADC interrupt at 10kHz.
 */
int main(void)
{
	/* Initialize UART for PC communication */
	Usart2_config();
	USART_ConfigInterrupt();

	/* Initialize hardware peripherals */
	hard_init();
	
	/* Initialize OLED display */
	OLED_Init();
	
	/* Initialize DRV8301 motor driver */
	drv8301_init();
	
	/* Initialize FOC control algorithms */
	foc_algorithm_initialize();
	
	/* Start ADC offset calibration */
	if (get_offset_flag == 0)
	{
		get_offset_flag = 1;
		TIM_CtrlPWMOutputs(PWM_TIM, ENABLE);
	}
	
	/* Enable DRV8301 gate drivers */
	GPIO_SetBits(DRV8301_ENGATE_GPIO_PORT, DRV8301_ENGATE_PIN);

	/* Main loop - low frequency tasks */
	while (1)
	{
		drv8301_protection();	/* Monitor for driver faults */
		oled_display_handle();	/* Update OLED display */
	}
}


/**
 * @brief Delay function using SysTick
 * @param nTime Time in milliseconds
 */
void Delay(__IO uint32_t nTime)
{
	uwTimingDelay = nTime;
	while (uwTimingDelay != 0);
}

/**
 * @brief Decrement delay counter (called from SysTick handler)
 */
void TimingDelay_Decrement(void)
{
	if (uwTimingDelay != 0x00)
	{
		uwTimingDelay--;
	}
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{

	while (1)
	{
	}
}
#endif
