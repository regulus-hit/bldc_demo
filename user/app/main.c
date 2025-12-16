/**********************************
 * Main Program Entry Point
 * BLDC Motor FOC Control System
 **********************************/
#include "../main.h"
#include "../interface/UpperComputer.h"
#include "../interface/USART2.h"

/* Private functions ---------------------------------------------------------*/

static __IO uint32_t uwTimingDelay;
static __IO uint32_t uwSysTickCount;
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

	uwTimingDelay = 0;
	uwSysTickCount = 0;

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
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/
void SysTick_Handler(void)
{
	low_task_c_systick_sub();
}

/**
 * @brief TIM1 Update/TIM10 Interrupt Handler
 * 
 * Handles PWM timer update events. Currently only clears the flag.
 * Could be used for PWM cycle synchronization or debugging.
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
	TIM_ClearFlag(PWM_TIM, TIM_FLAG_Update);
}

void TIM2_IRQHandler(void)
{
	hall_sensor_c_tim2_sub();
}

void ADC_IRQHandler(void)
{
	adc_c_adc_sub();
}

void DMA1_Stream6_IRQHandler(void)
{
	adc_c_dma1_stream6_sub();
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

void SysTick_update(void)
{
	uwSysTickCount += 1;
}

uint32_t GetSysUptime(void)
{
	return uwSysTickCount;
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{

	while (1)
	{
	}
}
#endif
