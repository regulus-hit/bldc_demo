/**********************************
 * Hardware Configuration and Initialization
 * STM32F4 board-specific peripheral setup for BLDC motor control
 **********************************/
#include "main.h"
#include "board_config.h"

/**
 * @brief Initialize System Clocks and Peripherals
 * 
 * Enables clocks for all hardware peripherals used in the BLDC control system:
 * - GPIO ports for PWM, Hall sensors, ADC, OLED, DRV8301, and user interface
 * - Timers for PWM generation, Hall sensor capture, and task scheduling
 * - ADC for current and voltage sensing
 * - DMA for efficient data transfers
 * - SysTick timer configured for 1ms interrupts (1kHz)
 * 
 * Must be called early in system initialization before any peripheral use.
 */
void hardware_clock_init(void)
{
	RCC_ClocksTypeDef RCC_Clocks;

	/* Get system clock frequencies */
	RCC_GetClocksFreq(&RCC_Clocks);

	/* Enable GPIO clocks for OLED display interface */
	RCC_AHB1PeriphClockCmd(OLED_SPIx_SCK_GPIO_CLK | OLED_SPIx_MOSI_GPIO_CLK | OLED_DC_GPIO_CLK | OLED_RESET_GPIO_CLK, ENABLE);

	/* Enable GPIO clocks for DRV8301 SPI interface */
	RCC_AHB1PeriphClockCmd(DRV8301_SPIx_SCK_GPIO_CLK | DRV8301_SPIx_MOSI_GPIO_CLK | DRV8301_SPIx_MISO_GPIO_CLK | DRV8301_SPIx_CS_GPIO_CLK, ENABLE);

	/* Enable GPIO clocks for user input keys */
	RCC_AHB1PeriphClockCmd(KEY_1_GPIO_CLK | KEY_2_GPIO_CLK | KEY_3_GPIO_CLK, ENABLE);

	/* Enable GPIO clocks for 3-phase PWM outputs (high-side) */
	RCC_AHB1PeriphClockCmd(PWM_AH_GPIO_CLK | PWM_BH_GPIO_CLK | PWM_CH_GPIO_CLK, ENABLE);

	/* Enable GPIO clocks for 3-phase PWM outputs (low-side) */
	RCC_AHB1PeriphClockCmd(PWM_AL_GPIO_CLK | PWM_BL_GPIO_CLK | PWM_CL_GPIO_CLK, ENABLE);

	/* Enable GPIO clocks for Hall sensor inputs */
	RCC_AHB1PeriphClockCmd(HALL_CH1_GPIO_CLK | HALL_CH2_GPIO_CLK | HALL_CH3_GPIO_CLK, ENABLE);

	/* Enable GPIO clocks for DRV8301 control and status pins */
	RCC_AHB1PeriphClockCmd(DRV8301_ENGATE_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(DRV8301_FAULT_GPIO_CLK, ENABLE);

	/* Enable GPIO clocks for ADC inputs (current and voltage sensing) */
	RCC_AHB1PeriphClockCmd(VBUS_ADC_GPIO_CLK | A_CURRENT_ADC_GPIO_CLK | B_CURRENT_ADC_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(TOTAL_CURRENT_ADC_GPIO_CLK | TEMPERATURE_ADC_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(U_VOLT_ADC_GPIO_CLK | V_VOLT_ADC_GPIO_CLK | W_VOLT_ADC_GPIO_CLK, ENABLE);

	/* Enable GPIO clock for user LED */
	RCC_AHB1PeriphClockCmd(USER_LED_GPIO_CLK, ENABLE);

	/* Enable peripheral clocks */
	RCC_APB2PeriphClockCmd(PWM_TIM_CLK, ENABLE);           /* PWM timer */
	RCC_APB2PeriphClockCmd(SAMPLE_ADC_CLK, ENABLE);        /* ADC for current/voltage sampling */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); /* System configuration controller */
	RCC_APB1PeriphClockCmd(HALL_TIM_CLK, ENABLE);          /* Hall sensor timer */
	RCC_APB1PeriphClockCmd(COMMUNICATION_TASK_TIM_CLK, ENABLE); /* Communication task timer */
	RCC_AHB1PeriphClockCmd(DMA2_CLK, ENABLE);              /* DMA controller */

	/* Configure SysTick for 1ms interrupts (1kHz system tick) */
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
}

/**
 * @brief Initialize Hall Sensor Interface
 * 
 * Configures GPIO pins and timer for Hall sensor position feedback:
 * - Three Hall sensor digital inputs (A, B, C)
 * - Timer in Hall sensor interface mode for automatic position capture
 * - Input capture on state changes for speed/position measurement
 * - 32-bit timer period for wide speed range
 * - Digital filtering to reject noise
 * 
 * Hall sensors provide 60-degree resolution position feedback for BLDC motors.
 */
void hardware_hall_sensor_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_HALLTimeBaseInitStructure;
	TIM_ICInitTypeDef TIM_HALLICInitStructure;

	/* Configure Hall sensor channel 1 GPIO */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = HALL_CH1_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(HALL_CH1_GPIO_PORT, &GPIO_InitStructure);

	/* Configure Hall sensor channel 2 GPIO */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = HALL_CH2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(HALL_CH2_GPIO_PORT, &GPIO_InitStructure);

	/* Configure Hall sensor channel 3 GPIO */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = HALL_CH3_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(HALL_CH3_GPIO_PORT, &GPIO_InitStructure);

	/* Connect GPIO pins to timer alternate functions */
	GPIO_PinAFConfig(HALL_CH1_GPIO_PORT, HALL_CH1_SOURCE, HALL_CH1_AF);
	GPIO_PinAFConfig(HALL_CH2_GPIO_PORT, HALL_CH2_SOURCE, HALL_CH2_AF);
	GPIO_PinAFConfig(HALL_CH3_GPIO_PORT, HALL_CH3_SOURCE, HALL_CH3_AF);

	/* Configure timer for Hall sensor interface */
	TIM_DeInit(HALL_TIM);
	TIM_TimeBaseStructInit(&TIM_HALLTimeBaseInitStructure);
	TIM_HALLTimeBaseInitStructure.TIM_Period = 0xffffffff;  /* 32-bit period for wide range */
	TIM_HALLTimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(HALL_TIM, &TIM_HALLTimeBaseInitStructure);

	/* Configure input capture for Hall sensor events */
	TIM_ICStructInit(&TIM_HALLICInitStructure);
	TIM_HALLICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_HALLICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge; /* Trigger on any edge */
	TIM_HALLICInitStructure.TIM_ICFilter = 0x0f;  /* Digital filter to reject noise */
	TIM_ICInit(HALL_TIM, &TIM_HALLICInitStructure);

	/* Configure timer for Hall sensor mode */
	TIM_PrescalerConfig(HALL_TIM, (uint16_t)0, TIM_PSCReloadMode_Immediate);
	TIM_InternalClockConfig(HALL_TIM);
	TIM_SelectHallSensor(HALL_TIM, ENABLE);  /* Enable Hall sensor interface */
	TIM_SelectInputTrigger(HALL_TIM, TIM_TS_TI1FP1);
	TIM_SelectSlaveMode(HALL_TIM, TIM_SlaveMode_Reset);  /* Reset counter on Hall state change */

	/* Clear all timer flags */
	TIM_ClearFlag(HALL_TIM, TIM_FLAG_Update | TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3 | TIM_FLAG_CC4 |
	              TIM_FLAG_Trigger | TIM_FLAG_CC1OF | TIM_FLAG_CC2OF | TIM_FLAG_CC3OF | TIM_FLAG_CC4OF);

	/* Enable capture interrupt */
	TIM_ITConfig(HALL_TIM, TIM_IT_CC1, ENABLE);
	TIM_SetCounter(HALL_TIM, 0);
	TIM_Cmd(HALL_TIM, ENABLE);
}

/**
 * @brief Initialize Communication Task Timer
 * 
 * Configures timer for periodic communication tasks.
 * This timer generates interrupts for scheduled data transmission
 * to PC or other external devices.
 */
void hardware_communication_init(void)
{
	TIM_TimeBaseInitTypeDef COM_TASK_TIM_TimeBaseStructure;

	TIM_DeInit(COMMUNICATION_TASK_TIM);
	TIM_TimeBaseStructInit(&COM_TASK_TIM_TimeBaseStructure);
	COM_TASK_TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	COM_TASK_TIM_TimeBaseStructure.TIM_Prescaler = COMMUNICATION_TASK_TIM_PRESCALER;
	COM_TASK_TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	COM_TASK_TIM_TimeBaseStructure.TIM_Period = COM_TASK_TIM_PERIOD;
	TIM_TimeBaseInit(COMMUNICATION_TASK_TIM, &COM_TASK_TIM_TimeBaseStructure);
	TIM_ITConfig(COMMUNICATION_TASK_TIM, TIM_IT_Update, ENABLE);
	TIM_Cmd(COMMUNICATION_TASK_TIM, ENABLE);
}

/**
 * @brief Initialize DRV8301 Gate Driver GPIO Pins
 * 
 * Configures GPIO pins for DRV8301 gate driver control:
 * - FAULT pin: Input to monitor gate driver faults (overcurrent, overtemperature, etc.)
 * - EN_GATE pin: Output to enable/disable all gate drivers
 * - SPI pins: For configuration and status reading
 * 
 * The DRV8301 provides gate drive for 6 MOSFETs in a 3-phase inverter.
 */
void hardware_drv8301_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure FAULT pin as input to monitor gate driver status */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_InitStructure.GPIO_Pin = DRV8301_FAULT_PIN;
	GPIO_Init(DRV8301_FAULT_GPIO_PORT, &GPIO_InitStructure);

	/* Configure EN_GATE pin as output to enable/disable gate drivers */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin = DRV8301_ENGATE_PIN;
	GPIO_Init(DRV8301_ENGATE_GPIO_PORT, &GPIO_InitStructure);

	/* Configure SPI pins as outputs for bit-banged SPI communication */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	GPIO_InitStructure.GPIO_Pin = DRV8301_SPIx_SCK_PIN;
	GPIO_Init(DRV8301_SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = DRV8301_SPIx_MOSI_PIN;
	GPIO_Init(DRV8301_SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = DRV8301_SPIx_CS_PIN;
	GPIO_Init(DRV8301_SPIx_CS_GPIO_PORT, &GPIO_InitStructure);

	/* Configure MISO pin as open-drain input with pull-up */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Pin = DRV8301_SPIx_MISO_PIN;
	GPIO_Init(DRV8301_SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);

	/* Set default pin states for SPI communication */
	GPIO_SetBits(DRV8301_ENGATE_GPIO_PORT, DRV8301_ENGATE_PIN);
	GPIO_SetBits(DRV8301_SPIx_CS_GPIO_PORT, DRV8301_SPIx_CS_PIN);
	GPIO_SetBits(DRV8301_SPIx_MISO_GPIO_PORT, DRV8301_SPIx_MISO_PIN);
	GPIO_ResetBits(DRV8301_SPIx_SCK_GPIO_PORT, DRV8301_SPIx_SCK_PIN);
	GPIO_ResetBits(DRV8301_SPIx_MOSI_GPIO_PORT, DRV8301_SPIx_MOSI_PIN);
}

/**
 * @brief Initialize OLED Display GPIO Pins
 * 
 * Configures GPIO pins for SPI-based OLED display communication:
 * - SCK: SPI clock output
 * - MOSI: SPI data output
 * - DC: Data/Command selection pin (0=command, 1=data)
 * - RESET: Display hardware reset pin
 * 
 * Uses bit-banged SPI protocol for display control.
 */
void hardware_oled_screen_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure all OLED pins as outputs with push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin = OLED_SPIx_SCK_PIN;
	GPIO_Init(OLED_SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = OLED_SPIx_MOSI_PIN;
	GPIO_Init(OLED_SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = OLED_DC_PIN;
	GPIO_Init(OLED_DC_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = OLED_RESET_PIN;
	GPIO_Init(OLED_RESET_GPIO_PORT, &GPIO_InitStructure);

	/* Assert RESET to keep display in reset state until initialization */
	GPIO_ResetBits(OLED_RESET_GPIO_PORT, OLED_RESET_PIN);
}

/**
 * @brief Initialize LED Indicator GPIO Pin
 * 
 * Configures GPIO pin for user LED indicator.
 * Used for visual feedback of system status.
 */
void hardware_led_indicator_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = USER_LED_PIN;
	GPIO_Init(USER_LED_GPIO_PORT, &GPIO_InitStructure);
}

/**
 * @brief Initialize PWM Timer for Motor Control
 * 
 * Configures advanced timer for 3-phase complementary PWM generation:
 * - Center-aligned PWM mode for reduced current ripple
 * - Complementary outputs with dead-time insertion to prevent shoot-through
 * - Frequency: 10kHz (configurable via PWM_TIM_PULSE)
 * - Dead-time: Prevents both high and low side FETs from conducting simultaneously
 * - Initial duty cycle: 50% (motor stopped state)
 * - Channel 4: Used to trigger ADC for current sampling synchronization
 * 
 * This timer drives the 6 MOSFETs (3 high-side, 3 low-side) in the 3-phase inverter.
 */
void hardware_pwm_init(void)
{
	GPIO_InitTypeDef			GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef		PWM_TIM_TimeBaseStructure;
	TIM_OCInitTypeDef			PWM_TIM_OCInitStructure;
	TIM_BDTRInitTypeDef			PWM_TIM_BDTRInitStructure;

	/* PWM config start */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_InitStructure.GPIO_Pin = PWM_AH_PIN;
	GPIO_Init(PWM_AH_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PWM_AL_PIN;
	GPIO_Init(PWM_AL_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PWM_BH_PIN;
	GPIO_Init(PWM_BH_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PWM_BL_PIN;
	GPIO_Init(PWM_BL_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PWM_CH_PIN;
	GPIO_Init(PWM_CH_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PWM_CL_PIN;
	GPIO_Init(PWM_CL_GPIO_PORT, &GPIO_InitStructure);  

	/* Connect GPIO pins to timer alternate functions */
	GPIO_PinAFConfig(PWM_AH_GPIO_PORT, PWM_AH_SOURCE, PWM_AH_AF);
	GPIO_PinAFConfig(PWM_AL_GPIO_PORT, PWM_AL_SOURCE, PWM_AL_AF);
	GPIO_PinAFConfig(PWM_BH_GPIO_PORT, PWM_BH_SOURCE, PWM_BH_AF);
	GPIO_PinAFConfig(PWM_BL_GPIO_PORT, PWM_BL_SOURCE, PWM_BL_AF);
	GPIO_PinAFConfig(PWM_CH_GPIO_PORT, PWM_CH_SOURCE, PWM_CH_AF);
	GPIO_PinAFConfig(PWM_CL_GPIO_PORT, PWM_CL_SOURCE, PWM_CL_AF);

	/* Configure timer time base */
	TIM_DeInit(PWM_TIM);
	TIM_TimeBaseStructInit(&PWM_TIM_TimeBaseStructure);

	PWM_TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  /* No prescaler - full speed */
	PWM_TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;  /* Center-aligned for low ripple */
	PWM_TIM_TimeBaseStructure.TIM_Period = PWM_TIM_PULSE;  /* PWM period */
	PWM_TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	PWM_TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(PWM_TIM, &PWM_TIM_TimeBaseStructure);

	/* Configure output compare channels 1-3 for 3-phase complementary PWM */
	TIM_OCStructInit(&PWM_TIM_OCInitStructure);
	PWM_TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	PWM_TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	PWM_TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;  /* Enable complementary output */
	PWM_TIM_OCInitStructure.TIM_Pulse = PWM_TIM_PULSE >> 1;  /* 50% duty cycle (motor stopped) */
	PWM_TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	PWM_TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	PWM_TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	PWM_TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OC1Init(PWM_TIM, &PWM_TIM_OCInitStructure);  /* Phase A */
	TIM_OC2Init(PWM_TIM, &PWM_TIM_OCInitStructure);  /* Phase B */
	TIM_OC3Init(PWM_TIM, &PWM_TIM_OCInitStructure);  /* Phase C */

	/* Configure channel 4 for ADC trigger synchronization */
	PWM_TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	PWM_TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	PWM_TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	PWM_TIM_OCInitStructure.TIM_Pulse = PWM_TIM_PULSE;
	PWM_TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	PWM_TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	PWM_TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	PWM_TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	TIM_OC4Init(PWM_TIM, &PWM_TIM_OCInitStructure);

	/* Configure break and dead-time register (BDTR) */
	PWM_TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	PWM_TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	PWM_TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
	PWM_TIM_BDTRInitStructure.TIM_DeadTime = PWM_DEAD_TIME;  /* Dead-time to prevent shoot-through */
	PWM_TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	PWM_TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
	PWM_TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
	TIM_BDTRConfig(PWM_TIM, &PWM_TIM_BDTRInitStructure);

	/* Enable timer */
	TIM_Cmd(PWM_TIM, ENABLE);

	/* Set initial duty cycles to 50% (safe state) */
	PWM_TIM->CCR1 = PWM_TIM_PULSE >> 1;  /* Phase A */
	PWM_TIM->CCR2 = PWM_TIM_PULSE >> 1;  /* Phase B */
	PWM_TIM->CCR3 = PWM_TIM_PULSE >> 1;  /* Phase C */
	PWM_TIM->CCR4 = PWM_TIM_PULSE - 1;   /* ADC trigger timing */
}

/**
 * @brief Initialize DMA for ADC Data Transfer
 * 
 * Configures DMA to automatically transfer ADC conversion results to memory:
 * - Channel: DMA2 Stream 0 (connected to ADC1)
 * - Transfer: ADC data register -> memory buffer
 * - Mode: Circular (continuous transfer without CPU intervention)
 * - Priority: High (critical for real-time motor control)
 * - Buffer size: 4 samples (for multi-channel ADC)
 * 
 * DMA allows ADC sampling without CPU overhead, critical for 10kHz control loop.
 */
void hardware_dma_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	DMA_InitStructure.DMA_Channel = DMA_CHANNEL0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC1ConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 4;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  /* Continuous operation */
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_STREAM0, &DMA_InitStructure);
	DMA_Cmd(DMA2_STREAM0, ENABLE);
}

/**
 * @brief Initialize ADC for Current and Voltage Sensing
 * 
 * Configures ADC for motor current and bus voltage measurement:
 * - Regular channels: Bus voltage and other analog signals (via DMA)
 * - Injected channels: Phase currents A, B (triggered by PWM for synchronization)
 * - Resolution: 12-bit (0-4095)
 * - Conversion triggered by PWM timer for precise timing
 * - Injected group generates interrupt for real-time FOC execution
 * 
 * The ADC is synchronized with PWM to sample currents at the optimal point
 * in the PWM cycle, minimizing switching noise in measurements.
 */
void hardware_adc_init(void)
{
	GPIO_InitTypeDef		GPIO_InitStructure;
	ADC_InitTypeDef			ADC_InitStructure;
	ADC_CommonInitTypeDef	ADC_CommonInitStructure;

	/* ADC  config start */

	/* Configure ADC input pins as analog */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_InitStructure.GPIO_Pin = VBUS_ADC_PIN;
	GPIO_Init(VBUS_ADC_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = A_CURRENT_ADC_PIN;
	GPIO_Init(A_CURRENT_ADC_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = B_CURRENT_ADC_PIN;
	GPIO_Init(B_CURRENT_ADC_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = TOTAL_CURRENT_ADC_PIN;
	GPIO_Init(TOTAL_CURRENT_ADC_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = TEMPERATURE_ADC_PIN;
	GPIO_Init(TEMPERATURE_ADC_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = U_VOLT_ADC_PIN;
	GPIO_Init(U_VOLT_ADC_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = V_VOLT_ADC_PIN;
	GPIO_Init(V_VOLT_ADC_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = W_VOLT_ADC_PIN;
	GPIO_Init(W_VOLT_ADC_GPIO_PORT, &GPIO_InitStructure);

	/* Configure ADC peripheral */
	ADC_DeInit();

	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 4;

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;

	ADC_CommonInit(&ADC_CommonInitStructure);
	ADC_Init(SAMPLE_ADC, &ADC_InitStructure);

	/* Configure injected channels (triggered by PWM for FOC) */
	ADC_InjectedSequencerLengthConfig(SAMPLE_ADC, 4);

	ADC_ExternalTrigInjectedConvConfig(SAMPLE_ADC, ADC_ExternalTrigInjecConv_T1_CC4);
	ADC_ExternalTrigInjectedConvEdgeConfig(SAMPLE_ADC, ADC_ExternalTrigInjecConvEdge_Rising);

	ADC_InjectedChannelConfig(SAMPLE_ADC, VBUS_ADC_CHANNEL, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(SAMPLE_ADC, A_CURRENT_ADC_CHANNEL, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(SAMPLE_ADC, B_CURRENT_ADC_CHANNEL, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(SAMPLE_ADC, TOTAL_CURRENT_ADC_CHANNEL, 4, ADC_SampleTime_15Cycles);

	/* Configure regular channels (converted via DMA) */
	ADC_RegularChannelConfig(SAMPLE_ADC, U_VOLT_ADC_CHANNEL, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(SAMPLE_ADC, V_VOLT_ADC_CHANNEL, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(SAMPLE_ADC, W_VOLT_ADC_CHANNEL, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(SAMPLE_ADC, TEMPERATURE_ADC_CHANNEL, 4, ADC_SampleTime_15Cycles);

	/* Enable DMA for regular channels */
	ADC_DMARequestAfterLastTransferCmd(SAMPLE_ADC, ENABLE);
	ADC_DMACmd(SAMPLE_ADC, ENABLE);

	/* Enable interrupt for injected channels (critical for FOC timing) */
	ADC_ITConfig(SAMPLE_ADC, ADC_IT_JEOC, ENABLE);
	ADC_ClearFlag(SAMPLE_ADC, ADC_FLAG_JEOC);
	ADC_Cmd(SAMPLE_ADC, ENABLE);
	ADC_SoftwareStartConv(SAMPLE_ADC);
}

/**
 * @brief Initialize External Interrupts for User Buttons
 * 
 * Configures GPIO pins and EXTI lines for three user control buttons:
 * - KEY1: Rising and falling edge triggered (for press/release detection)
 * - KEY2: Falling edge triggered (speed decrease)
 * - KEY3: Falling edge triggered (speed increase)
 * 
 * Button interrupts allow responsive user input for motor control.
 */
void hardware_exti_button_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	/* Configure button GPIO pins as inputs */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_InitStructure.GPIO_Pin = KEY_1_PIN;
	GPIO_Init(KEY_1_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = KEY_2_PIN;
	GPIO_Init(KEY_2_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = KEY_3_PIN;
	GPIO_Init(KEY_3_GPIO_PORT, &GPIO_InitStructure);

	/* Connect GPIO pins to EXTI lines */
	SYSCFG_EXTILineConfig(KEY_1_EXTI_GPIO_PORT, KEY_1_EXTI_SOURCE);
	SYSCFG_EXTILineConfig(KEY_2_EXTI_GPIO_PORT, KEY_2_EXTI_SOURCE);
	SYSCFG_EXTILineConfig(KEY_3_EXTI_GPIO_PORT, KEY_3_EXTI_SOURCE);

	/* Configure KEY2 and KEY3 for falling edge (button press) */
	EXTI_InitStructure.EXTI_Line = KEY_2_EXTI_LINE | KEY_3_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Configure KEY1 for both edges (press and release detection) */
	EXTI_InitStructure.EXTI_Line = KEY_1_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

/**
 * @brief Initialize USART2 DMA for Data Transmission
 * 
 * Configures DMA for USART2 transmit to enable non-blocking data transfer.
 * Used for sending motor telemetry data to PC without blocking FOC control loop.
 * DMA automatically transfers data buffer to USART, freeing CPU for control tasks.
 */
void hardware_usart2_dma_init(void)
{
	DMA_InitTypeDef DMA_USART2_TX_InitStructure;

	/* USART2 DMA config start */

	RCC_AHB1PeriphClockCmd(USART2_TX_DMA_CLK, ENABLE);

	DMA_DeInit(USART2_TX_DMA_STREAM);

	DMA_USART2_TX_InitStructure.DMA_Channel = USART2_TX_DMA_CHANNEL;
	DMA_USART2_TX_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
	DMA_USART2_TX_InitStructure.DMA_Memory0BaseAddr = (uint32_t)0;  /* Set at runtime */
	DMA_USART2_TX_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_USART2_TX_InitStructure.DMA_BufferSize = 0;  /* Set at runtime */
	DMA_USART2_TX_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_USART2_TX_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_USART2_TX_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_USART2_TX_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_USART2_TX_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_USART2_TX_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_USART2_TX_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_USART2_TX_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_USART2_TX_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_USART2_TX_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(USART2_TX_DMA_STREAM, &DMA_USART2_TX_InitStructure);
}

/**
 * @brief Initialize Interrupt Priorities (NVIC)
 * 
 * Configures nested vectored interrupt controller priorities for all interrupts:
 * - Priority group 2: 2 bits for preemption, 2 bits for sub-priority
 * 
 * Interrupt Priority Hierarchy (lower number = higher priority):
 * 1. ADC (Priority 1): Highest - critical for FOC control loop timing
 * 2. Hall Timer (Priority 2): High - rotor position/speed feedback
 * 3. External Interrupts (Priority 3): Low - user button inputs
 * 4. DMA (Priority 3): Low - data transfer completion
 * 
 * This priority scheme ensures real-time motor control is never interrupted
 * by non-critical tasks.
 */
void hardware_interrupt_init(void)
{
	/* Configure priority grouping */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* ADC interrupt: Highest priority for FOC control */
	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* External interrupt line 2 (KEY3 - speed increase) */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* External interrupt line 4 (KEY1 - start/stop/direction) */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* External interrupt lines 5-9 (KEY2 - speed decrease) */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Hall sensor timer interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* USB OTG interrupt: Very high priority for USB communication */
	NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* USART2 DMA interrupt: Medium priority for telemetry data */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_TX_DMA_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

#ifdef __ICCARM__
	/* SysTick interrupt: Low priority for timing tasks */
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}

/**
 * @brief Master Hardware Initialization Function
 * 
 * Initializes all hardware peripherals in correct sequence:
 * 1. System clocks and SysTick timer
 * 2. Hall sensor interface for position feedback
 * 3. DRV8301 gate driver GPIO
 * 4. OLED display interface
 * 5. LED indicator
 * 6. PWM timer for motor control
 * 7. DMA for ADC and USART
 * 8. ADC for current/voltage sensing
 * 9. External interrupts for user buttons
 * 10. USART2 DMA for telemetry
 * 11. Interrupt priorities (NVIC)
 * 
 * Must be called once at system startup before main loop.
 */
void hard_init(void)
{
	hardware_clock_init();
	hardware_hall_sensor_init();
	/* hardware_communication_init(); */  /* Optional communication timer */
	hardware_drv8301_init();
	hardware_oled_screen_init();
	hardware_led_indicator_init();
	hardware_pwm_init();
	hardware_dma_init();
	hardware_adc_init();
	hardware_exti_button_init();
	hardware_usart2_dma_init();
	/* communication_init(); */  /* Optional: USB communication initialization */

	hardware_interrupt_init();
}
