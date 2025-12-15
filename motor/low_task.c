/**********************************
 * Low Frequency Tasks
 * Handles motor start/stop, user input, and speed control
 **********************************/
#include "main.h"
#include "low_task.h"
#include "adc.h"

uint16_t hz_100_cnt = 0;
uint8_t motor_start_stop = 0;
uint8_t motor_start_stop_pre = 1;

uint16_t key1_cnt;
uint8_t key1_press_flag = 0;

/**
 * @brief Start Motor Operation
 * 
 * Initializes all FOC components and enables PWM to begin motor control.
 * Sets initial speed reference and resets control states.
 */
void motor_start(void)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_9);
	foc_algorithm_initialize();
	Speed_Ref = motor_direction * 25.0F;	/* Initial startup speed */
	speed_close_loop_flag = 0;				/* Start with open-loop current control */
	Iq_ref = 0.0f;

	hall_angle_add = 0.0005f;
	hall_speed = 0.0f;
	TIM_CtrlPWMOutputs(PWM_TIM, ENABLE);

	motor_run_display_flag = 1;
}

/**
 * @brief Stop Motor Operation
 * 
 * Disables PWM outputs to stop the motor.
 */
void motor_stop(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_9);
	TIM_CtrlPWMOutputs(PWM_TIM, DISABLE);
	motor_run_display_flag = 0;
}


/**
 * @brief Low Frequency Control Task
 * 
 * Handles user input from three keys for motor control:
 * - Key1: Short press toggles motor start/stop; long press (>100ms) reverses direction
 * - Key2: Decreases speed reference by 5 Hz (minimum 25 Hz)
 * - Key3: Increases speed reference by 5 Hz (maximum 200 Hz)
 * 
 * This function is called at 100Hz from the SysTick interrupt handler.
 * It manages motor state transitions and speed adjustments based on user input.
 */
void lowfreq_control_task(void)
{
	/* Check if ADC offset calibration is complete */
	if (get_offset_flag == 2)
	{
		/* Handle motor start/stop state changes */
		if (motor_start_stop_pre != motor_start_stop)
		{
			motor_start_stop_pre = motor_start_stop;
			if (motor_start_stop == 1)
			{
				motor_start();
			}
			else
			{
				motor_stop();
			}
		}
	}

	/* Key1: Motor start/stop and direction control */
	if (key1_flag == 1)
	{
		key1_press_flag = 1;
	}
	if (key1_press_flag)
	{
		key1_cnt++;
		if (key1_cnt < 100 && key1_flag == 0)
		{
			/* Short press: Toggle motor start/stop */
			if (motor_start_stop == 0)
			{
				motor_start_stop = 1;
			}
			else
			{
				motor_start_stop = 0;
			}
			key1_flag = 0;
			key1_cnt = 0;
			key1_press_flag = 0;
		}
		else if (key1_cnt > 100)
		{
			/* Long press: Reverse motor direction */
			motor_stop();
			motor_direction = -motor_direction;
			motor_start();
			key1_cnt = 0;
			key1_flag = 0;
			key1_press_flag = 0;
		}
	}

	/* Key2: Decrease speed reference */
	if (key2_flag == 1)
	{
		display_flag = 1;  /* Display running parameters */
		if (motor_direction != -1.0f)
		{
			if (Speed_Ref > 25.0f)  /* Minimum speed: 25 Hz */
			{
				Speed_Ref -= 5.0f;  /* Step size: 5 Hz */
			}
		}
		else
		{
			if (Speed_Ref < -25.0f)  /* Minimum speed (reverse): -25 Hz */
			{
				Speed_Ref += 5.0f;  /* Step size: 5 Hz */
			}
		}
		key2_flag = 0;
	}

	/* Key3: Increase speed reference */
	if (key3_flag == 1)
	{
		if (motor_direction != -1.0f)
		{
			Speed_Ref += 5.0f;  /* Step size: 5 Hz */
			if (Speed_Ref > 200.0f)  /* Maximum speed: 200 Hz */
			{
				Speed_Ref = 200.0f;
			}
		}
		else
		{
			Speed_Ref -= 5.0f;  /* Step size: 5 Hz */
			if (Speed_Ref < -200.0f)  /* Maximum speed (reverse): -200 Hz */
			{
				Speed_Ref = -200.0f;
			}
		}
		key3_flag = 0;
	}
}

/**
 * @brief SysTick Interrupt Handler
 * 
 * System tick interrupt running at 1kHz (1ms period).
 * Executes high priority tasks:
 * - Speed PI controller calculation (1kHz rate)
 * - DRV8301 fault monitoring
 * - Low frequency control tasks (100Hz rate via divider)
 * - System timing delay decrement
 * 
 * This is the main task scheduler for non-real-time control functions.
 */
void SysTick_Handler(void)
{
	/* Monitor DRV8301 gate driver for faults */
	if (drv8301_init_ok_flag == 1)
	{
		drv8301_protection();
	}

	/* Execute speed PI controller at 1kHz */
	Speed_Pid_Calc(Speed_Ref, Speed_Fdk, &Speed_Pid_Out, &Speed_Pid);

	/* Divide down to 100Hz for low priority tasks */
	hz_100_cnt++;
	if (hz_100_cnt == 10)
	{
		lowfreq_control_task();
		TimingDelay_Decrement();
		hz_100_cnt = 0;
	}
}
