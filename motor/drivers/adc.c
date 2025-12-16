/**********************************
 * ADC and Motor Control Main Loop
 * Handles ADC sampling, current/voltage measurement, and FOC execution
 **********************************/
#include "../../user/main.h"
#include "adc.h"
#include "../../user/interface/UpperComputer.h"

#ifdef HYBRID_HALL_EKF_SELECT
#include "../sensors/hybrid_observer.h"
#endif

#define SEND_BUFFER_SIZE 20  /* 16 bytes data + 4 bytes tail */
uint8_t send_buffer[SEND_BUFFER_SIZE];
volatile uint8_t dma_tx_busy = 0;

/* Temporary variables for data transmission */
float tempFloat[4];
uint8_t tempData[16];

int32_t ia_test, ib_test, ic_test;

/* Three-phase current measurements */
double Ia, Ib, Ic;
float Ia_test, Ib_test, Ic_test;

/* DC bus voltage */
float Vbus;

#ifdef ENABLE_VBUS_FILTERING
/* Filtered DC bus voltage for improved SVPWM accuracy */
float Vbus_filtered = 0.0f;
uint8_t Vbus_filter_initialized = 0;
#endif

uint16_t ADC1ConvertedValue[5];
uint16_t i = 0;

/* ADC offset calibration */
uint32_t A_offset, B_offset;
uint8_t get_offset_flag = 0;
uint8_t get_offset_sample_cnt = 0;

/* Display and control flags */
uint8_t oled_display_sample_freq = 0;
uint8_t speed_close_loop_flag;

/* Motor control variables */
float Iq_ref;
float EKF_Hz;
float theta_add;
float theta, myref = 0.00001;
float motor_direction = 1.0f;

extern float Rs;
extern float Ls;
extern float flux;
void send_PC(float wave1, float wave2, float wave3, float wave4);

/**
 * @brief Calibrate ADC current measurement offsets
 * 
 * Accumulates 128 ADC samples with motor disabled to determine zero-current offset.
 * This calibration is essential for accurate current measurement in FOC control.
 * The offset compensates for op-amp bias and ADC offset errors.
 * 
 * @param a_offset Pointer to phase A offset accumulator
 * @param b_offset Pointer to phase B offset accumulator
 */
void get_offset(uint32_t *a_offset, uint32_t *b_offset)
{
	if (get_offset_sample_cnt < 128)
	{
		/* Accumulate ADC samples */
		*a_offset += ADC1->JDR2;
		*b_offset += ADC1->JDR3;
		get_offset_sample_cnt++;
	}
	else
	{
		/* Average the accumulated samples (divide by 128 = right shift 7) */
		*a_offset >>= 7;
		*b_offset >>= 7;
#ifdef ADC_OFFSET_CHECK
		/* Validate offset values - should be near mid-scale (2048 for 12-bit ADC) */
		/* Allow ±200 counts tolerance for op-amp offset and ADC accuracy */
		if ((*a_offset > 2048 + 200) || (*a_offset < 2048 - 200) ||
		    (*b_offset > 2048 + 200) || (*b_offset < 2048 - 200))
		{
			/* Offset out of range - retry calibration */
			*a_offset = 2048;
			*b_offset = 2048;
			get_offset_sample_cnt = 0;
			/* Keep get_offset_flag at 1 to retry */
		}
		else
		{
			/* Valid offset - proceed to normal operation */
			get_offset_sample_cnt = 0;
			TIM_CtrlPWMOutputs(PWM_TIM, DISABLE);
			get_offset_flag = 2;
		}
#else
		get_offset_sample_cnt = 0;
		TIM_CtrlPWMOutputs(PWM_TIM, DISABLE);
		get_offset_flag = 2;
#endif
	}
}

/**
 * @brief Main motor control loop executed at PWM frequency (10kHz)
 * 
 * This is the core FOC control function that:
 * 1. Samples motor currents and bus voltage from ADC
 * 2. Performs current offset compensation
 * 3. Manages startup current ramping
 * 4. Switches between open-loop startup and closed-loop speed control
 * 5. Executes FOC algorithm (Clarke/Park transforms, current PI, SVPWM)
 * 6. Updates PWM duty cycles for motor control
 * 7. Monitors for fault conditions
 * 
 * Called from ADC interrupt service routine at each PWM cycle.
 */
void motor_run(void)
{
	float vbus_temp;
	double ia_temp, ib_temp;
	
	/* Read ADC values for bus voltage and phase currents */
	vbus_temp = (float)(ADC1->JDR1);								/* Bus voltage ADC raw value */
	ia_temp = (int16_t)((int16_t)A_offset - (int16_t)ADC1->JDR2);	/* Phase A current (offset compensated) */
	ib_temp = (int16_t)((int16_t)B_offset - (int16_t)ADC1->JDR3);	/* Phase B current (offset compensated) */
	
	/* Convert ADC values to real physical units */
	Vbus = vbus_temp * VBUS_CONVERSION_FACTOR;						/* Convert to actual voltage via voltage divider */
	
#ifdef ENABLE_VBUS_FILTERING
	/* Apply low-pass filter to bus voltage to reduce switching ripple
	 * First-order IIR filter: Vbus_filtered = alpha * Vbus_new + (1-alpha) * Vbus_old
	 * This reduces SVPWM calculation errors from DC link capacitor ripple */
	if (!Vbus_filter_initialized)
	{
		Vbus_filtered = Vbus;  /* Initialize filter with first measurement */
		Vbus_filter_initialized = 1;
	}
	else
	{
		Vbus_filtered = VBUS_FILTER_ALPHA * Vbus + (1.0f - VBUS_FILTER_ALPHA) * Vbus_filtered;
	}
#endif
	
	Ia = ia_temp * SAMPLE_CURR_CON_FACTOR;							/* Convert to actual current via shunt and amp gain */
	Ib = ib_temp * SAMPLE_CURR_CON_FACTOR;							/* Convert to actual current via shunt and amp gain */
	Ic = -Ia - Ib;													/* Calculate phase C current using Kirchhoff's law */
	
	Ia_test = Ia;
	Ib_test = Ib;
	Ic_test = Ic;
  
	/**
	 * Speed loop startup sequence management:
	 * Stage 0: Open-loop startup - ramp up Iq current slowly to spin motor
	 *          This provides smooth acceleration and prevents current spikes
	 *          Current loop and position estimation are active, but speed loop is open
	 * Stage 1: Transition - gradually reduce startup current
	 * Stage 2: Closed-loop operation - speed PID takes over Iq reference
	 */
	if (speed_close_loop_flag == 0)
	{
		/* Open-loop startup: slowly ramp torque current to accelerate motor */
		if ((Iq_ref < MOTOR_STARTUP_CURRENT * motor_direction))
		{
#ifdef ENABLE_STARTUP_CURRENT_PROFILING
			Iq_ref += STARTUP_CURRENT_RAMP_UP_RATE;	/* Configurable ramp rate */
#else
			Iq_ref += 0.001f;	/* Default: Gradual ramp prevents inrush current */
#endif
		}
		else
		{
			speed_close_loop_flag = 1;	/* Move to transition stage */
		}
	}
	else
	{
		if (speed_close_loop_flag == 1)
		{
			/* Transition stage: reduce startup current before speed loop takes over */
			if (Iq_ref > (MOTOR_STARTUP_CURRENT * motor_direction / 2.0f))
			{
#ifdef ENABLE_STARTUP_CURRENT_PROFILING
				Iq_ref -= STARTUP_CURRENT_RAMP_DOWN_RATE;	/* Configurable ramp rate */
#else
				Iq_ref -= 0.001f;	/* Default ramp rate */
#endif
			}
			else
			{
				speed_close_loop_flag = 2;	/* Enable closed-loop speed control */
			}
		}
	}

	/**
	 * Sensor mode selection for BLDC motor control:
	 * Two modes available via conditional compilation:
	 * 1. HALL_FOC_SELECT: Uses Hall sensors for rotor position and speed
	 * 2. SENSORLESS_FOC_SELECT: Uses Extended Kalman Filter (EKF) state observer
	 */

#ifdef HALL_FOC_SELECT	/* Hall sensor based FOC control */

	/* Check if speed is high enough for closed-loop speed control */
	if ((hall_speed * 2.0f * PI) > SPEED_LOOP_CLOSE_RAD_S)
	{
		/* Closed-loop speed control active */
		FOC_Input.Id_ref = 0.0f;				/* Zero d-axis current for maximum torque/amp */
		Speed_Fdk = hall_speed * 2.0f * PI;		/* Speed feedback from Hall sensors */
		FOC_Input.Iq_ref = Speed_Pid_Out;		/* Iq from speed controller */
		
#ifdef ENABLE_FIELD_WEAKENING
		/* Field-Weakening Control: Inject negative Id current at high speeds
		 * This extends the speed range by weakening the rotor flux
		 * Id_fw = -K_fw * (|speed| - base_speed) when speed > base_speed */
		float speed_abs = fabs(hall_speed * 2.0f * PI);
		if (speed_abs > FIELD_WEAKENING_BASE_SPEED)
		{
			float Id_fw = -FIELD_WEAKENING_GAIN * (speed_abs - FIELD_WEAKENING_BASE_SPEED);
			/* Limit negative Id to prevent demagnetization */
			if (Id_fw < FIELD_WEAKENING_MAX_NEG_ID)
			{
				Id_fw = FIELD_WEAKENING_MAX_NEG_ID;
			}
			FOC_Input.Id_ref = Id_fw;
		}
#endif
	}
	else
	{
		/* Open-loop startup mode */
		FOC_Input.Id_ref = 0.0f;
		FOC_Input.Iq_ref = Iq_ref;				/* Fixed startup current */
		Speed_Pid.I_Sum = Iq_ref;				/* Pre-load integral to prevent windup */
	}
	
	/* Position and speed from Hall sensors
	 * Use interpolated position when available for higher resolution */
#ifdef ENABLE_HALL_INTERPOLATION
	FOC_Input.theta = hall_angle_interpolated;  /* Interpolated position (higher resolution) */
#else
	FOC_Input.theta = hall_angle;               /* Raw Hall position (60° resolution) */
#endif
	FOC_Input.speed_fdk = hall_speed * 2.0f * PI;

#endif

#ifdef SENSORLESS_FOC_SELECT	/* Sensorless FOC using EKF state observer */

	/* Check if estimated speed is high enough for closed-loop speed control */
	if (fabs(FOC_Output.EKF[2]) > fabs(SPEED_LOOP_CLOSE_RAD_S))
	{
		/* Closed-loop speed control active */
		FOC_Input.Id_ref = 0.0f;			/* Zero d-axis current for SPMSM */
		Speed_Fdk = FOC_Output.EKF[2];		/* Speed from EKF observer */
		FOC_Input.Iq_ref = Speed_Pid_Out;	/* Iq from speed controller */
		
#ifdef ENABLE_FIELD_WEAKENING
		/* Field-Weakening Control: Inject negative Id current at high speeds
		 * This extends the speed range by weakening the rotor flux
		 * Id_fw = -K_fw * (|speed| - base_speed) when speed > base_speed */
		float speed_abs = fabs(FOC_Output.EKF[2]);
		if (speed_abs > FIELD_WEAKENING_BASE_SPEED)
		{
			float Id_fw = -FIELD_WEAKENING_GAIN * (speed_abs - FIELD_WEAKENING_BASE_SPEED);
			/* Limit negative Id to prevent demagnetization */
			if (Id_fw < FIELD_WEAKENING_MAX_NEG_ID)
			{
				Id_fw = FIELD_WEAKENING_MAX_NEG_ID;
			}
			FOC_Input.Id_ref = Id_fw;
		}
#endif
	}
	else
	{
		/* Open-loop startup mode - EKF converging */
		FOC_Input.Id_ref = 0.0f;
		FOC_Input.Iq_ref = Iq_ref;
		Speed_Pid.I_Sum = Iq_ref;
	}
	
	/* Position and speed from Extended Kalman Filter */
	FOC_Input.theta = FOC_Output.EKF[3] + myref;
	FOC_Input.speed_fdk = FOC_Output.EKF[2];

#endif

#ifdef HYBRID_HALL_EKF_SELECT	/* Hybrid Hall+EKF observer mode */

	/**
	 * Hybrid Observer Mode: Fuse EKF estimates with Hall sensor measurements
	 * 
	 * Benefits:
	 * - Smooth position/speed from EKF interpolation between Hall edges
	 * - Robustness from Hall sensor absolute position reference
	 * - Better performance than pure Hall (reduces quantization noise)
	 * - Better robustness than pure EKF (Hall prevents divergence)
	 * 
	 * Architecture:
	 * 1. EKF runs continuously, estimating position/speed from back-EMF
	 * 2. Hall sensors provide periodic position snapshots (every 60°)
	 * 3. Complementary filter fuses both measurements
	 * 4. At low speeds: trust EKF more (Hall timing unreliable)
	 * 5. At high speeds: use both for optimal performance
	 * 6. On divergence: Hall takes over to re-anchor EKF
	 */
	
	float fused_position, fused_speed;
	
	/* Update hybrid observer with both EKF and Hall measurements */
	hybrid_observer_update(
		FOC_Output.EKF[3] + myref,  /* EKF position estimate (rad) */
		FOC_Output.EKF[2],           /* EKF speed estimate (rad/s) */
		hall_angle,                  /* Hall sensor position (rad) */
		hall_speed * MATH_2PI,       /* Hall speed converted: Hz → rad/s (mechanical * 2π) */
		&fused_position,             /* Output: fused position */
		&fused_speed                 /* Output: fused speed */
	);
	
	/* Check if speed is high enough for closed-loop speed control */
	if (fabsf(fused_speed) > fabsf(SPEED_LOOP_CLOSE_RAD_S))
	{
		/* Closed-loop speed control active */
		FOC_Input.Id_ref = 0.0f;           /* Zero d-axis current for SPMSM */
		Speed_Fdk = fused_speed;           /* Speed feedback from hybrid observer */
		FOC_Input.Iq_ref = Speed_Pid_Out;  /* Iq from speed controller */
		
#ifdef ENABLE_FIELD_WEAKENING
		/* Field-Weakening Control: Inject negative Id current at high speeds
		 * This extends the speed range by weakening the rotor flux
		 * Id_fw = -K_fw * (|speed| - base_speed) when speed > base_speed */
		float speed_abs = fabsf(fused_speed);
		if (speed_abs > FIELD_WEAKENING_BASE_SPEED)
		{
			float Id_fw = -FIELD_WEAKENING_GAIN * (speed_abs - FIELD_WEAKENING_BASE_SPEED);
			/* Limit negative Id to prevent demagnetization */
			if (Id_fw < FIELD_WEAKENING_MAX_NEG_ID)
			{
				Id_fw = FIELD_WEAKENING_MAX_NEG_ID;
			}
			FOC_Input.Id_ref = Id_fw;
		}
#endif
	}
	else
	{
		/* Open-loop startup mode */
		FOC_Input.Id_ref = 0.0f;
		FOC_Input.Iq_ref = Iq_ref;
		Speed_Pid.I_Sum = Iq_ref;
	}
	
	/* Position and speed from hybrid observer (fused EKF+Hall) */
	FOC_Input.theta = fused_position;
	FOC_Input.speed_fdk = fused_speed;

#endif

	EKF_Hz = FOC_Output.EKF[2] / (2.0f * PI);

	/* Prepare FOC algorithm inputs */
	FOC_Input.Id_ref = 0;					/* Zero d-axis current for Surface PM motors */
	FOC_Input.Tpwm = PWM_TIM_PULSE_TPWM;	/* PWM period for SVPWM calculations */
#ifdef ENABLE_VBUS_FILTERING
	FOC_Input.Udc = Vbus_filtered;			/* Use filtered DC bus voltage for improved SVPWM accuracy */
#else
	FOC_Input.Udc = Vbus;					/* Use unfiltered DC bus voltage (raw measurement) */
#endif

	/* Motor parameters for state observer and current control */
	FOC_Input.Rs = Rs;						/* Stator resistance */
	FOC_Input.Ls = Ls;						/* Stator inductance */
	FOC_Input.flux = flux;					/* Rotor flux linkage */

	/* Three-phase current measurements */
	FOC_Input.ia = Ia;
	FOC_Input.ib = Ib;
	FOC_Input.ic = Ic;

	/**
	 * Execute complete FOC algorithm:
	 * - Clarke transform (abc -> alpha-beta)
	 * - Park transform (alpha-beta -> dq)
	 * - Current PI controllers (d-axis and q-axis)
	 * - Inverse Park transform (dq -> alpha-beta)
	 * - SVPWM calculation (alpha-beta -> PWM duties)
	 * - EKF state observer (estimates rotor position and speed)
	 * - Motor parameter identification (optional)
	 */
	foc_algorithm_step();

	/* Update PWM duty cycles if motor is running */
	if (motor_start_stop == 1)
	{
		PWM_TIM->CCR1 = (uint16_t)(FOC_Output.Tcmp1);	/* Phase A PWM duty */
		PWM_TIM->CCR2 = (uint16_t)(FOC_Output.Tcmp2);	/* Phase B PWM duty */
		PWM_TIM->CCR3 = (uint16_t)(FOC_Output.Tcmp3);	/* Phase C PWM duty */
	}
	else
	{
		/* Motor stopped - set all phases to 50% duty (center-aligned) */
		PWM_TIM->CCR1 = PWM_TIM_PULSE >> 1;
		PWM_TIM->CCR2 = PWM_TIM_PULSE >> 1;
		PWM_TIM->CCR3 = PWM_TIM_PULSE >> 1;
	}

	/* Monitor DRV8301 for fault conditions */
	drv8301_protection();

	/* Send telemetry data to PC via UART (VOFA+ third-party GUI) */
	/* Data: Phase A current, Phase B current, PWM waveform, Rotor position */
	send_PC(FOC_Input.ia, FOC_Input.ib, FOC_Output.Tcmp1, FOC_Output.EKF[3]);

	/* Alternative: Custom PC application (commented out) */
	/* SendWaveformData(FOC_Input.ia, FOC_Input.ib, FOC_Output.Tcmp1, FOC_Output.EKF[3], Speed_Ref*60, Speed_Fdk*60/6.28318548F); */

	/* Update OLED display at reduced rate (1/10 of control frequency) */
	oled_display_sample_freq++;
	if (oled_display_sample_freq == 10)
	{
		if (display_data_flag == 0)
		{
			/* Store rotor position data for OLED oscilloscope display */
			display_data_buff[display_data_buff_cnt] = (int8_t)(FOC_Output.EKF[3] * 15.0f);
			display_data_buff_cnt++;
			if (display_data_buff_cnt == 127)
			{
				display_data_buff_cnt = 0;
				display_data_flag = 1;
			}
		}
		oled_display_sample_freq = 0;
	}
}


/**
 * @brief ADC Injected Channel Conversion Complete Interrupt Handler
 * 
 * Triggered at PWM frequency (10kHz) when ADC finishes sampling motor currents.
 * This is the highest priority interrupt in the system as it drives FOC control.
 * Timing is synchronized with PWM to sample currents at optimal point in cycle.
 */
void adc_c_adc_sub(void)
{
	if ((SAMPLE_ADC->SR & ADC_FLAG_JEOC) == ADC_FLAG_JEOC)
	{
		if (get_offset_flag == 2)
		{
			/* Normal operation - execute motor control */
			
			/* Update simulated hall angle (for testing) */
			hall_angle += hall_angle_add;
			if (hall_angle < 0.0f)
			{
				hall_angle += 2.0f * PI;
			}
			else if (hall_angle > (2.0f * PI))
			{
				hall_angle -= 2.0f * PI;
			}
			
#if defined(HALL_FOC_SELECT) && defined(ENABLE_HALL_INTERPOLATION)
			/* Update Hall sensor interpolation for higher resolution position feedback
			 * Uses current timer value to interpolate position between Hall edges */
			hall_interpolation_update(TIM_GetCounter(HALL_TIM));
#endif
			
			/* Execute main FOC control loop */
			motor_run();
		}
		else
		{
			if (get_offset_flag == 1)
			{
				/* Calibration mode - collect ADC offset samples */
				get_offset(&A_offset, &B_offset);
			}
		}
		ADC_ClearFlag(SAMPLE_ADC, ADC_FLAG_JEOC);
	}
}

/**
 * @brief Send waveform data to PC via UART with DMA
 * 
 * Transmits 4 float values to PC for real-time plotting using VOFA+ protocol.
 * Uses DMA to avoid blocking the main control loop. Data format:
 * - 16 bytes: 4 floats (4 bytes each)
 * - 4 bytes: Tail marker (0x00 0x00 0x80 0x7F) for frame sync
 * 
 * @param wave1 First data channel (typically Phase A current)
 * @param wave2 Second data channel (typically Phase B current)
 * @param wave3 Third data channel (typically PWM duty cycle)
 * @param wave4 Fourth data channel (typically rotor position)
 */
void send_PC(float wave1, float wave2, float wave3, float wave4)
{
	/* Skip if previous DMA transfer still in progress */
	if (dma_tx_busy)
	{
		return;
	}

	/* Pack float data into byte array */
	tempFloat[0] = wave1;
	tempFloat[1] = wave2;
	tempFloat[2] = wave3;
	tempFloat[3] = wave4;
	memcpy(tempData, (uint8_t *)tempFloat, sizeof(tempFloat));

	/* Prepare send buffer: 16 bytes data + 4 bytes tail marker */
	memcpy(send_buffer, tempData, 16);
	send_buffer[16] = 0x00;
	send_buffer[17] = 0x00;
	send_buffer[18] = 0x80;
	send_buffer[19] = 0x7F;

	/* Configure and start DMA transfer */
	DMA_Cmd(DMA1_Stream6, DISABLE);

	/* Clear all DMA flags */
	DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6);

	/* Set DMA parameters */
	USART2_TX_DMA_STREAM->M0AR = (uint32_t)send_buffer;
	USART2_TX_DMA_STREAM->NDTR = SEND_BUFFER_SIZE;

	/* Enable DMA transfer complete interrupt */
	DMA_ITConfig(USART2_TX_DMA_STREAM, DMA_IT_TC, ENABLE);

	/* Start DMA transfer */
	dma_tx_busy = 1;
	DMA_Cmd(USART2_TX_DMA_STREAM, ENABLE);

	/* Enable USART2 DMA transmission */
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}

/**
 * @brief DMA1 Stream 6 Transfer Complete Interrupt Handler
 * 
 * Called when UART TX DMA transfer completes.
 * Cleans up DMA and allows next transmission.
 */
void adc_c_dma1_stream6_sub(void)
{
	if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6))
	{
		DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);

		/* Disable USART2 DMA transmission */
		USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);

		/* Disable DMA stream */
		DMA_Cmd(DMA1_Stream6, DISABLE);

		/* Clear busy flag to allow next transfer */
		dma_tx_busy = 0;
	}
}
