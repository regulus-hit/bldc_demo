#ifndef __ADC_H_
#define __ADC_H_

#include <stdint.h>

/* ADC voltage reference */
#define ADC_REF_V                   (float)(3.3f)

/* Bus voltage sensing divider resistors */
#define VBUS_UP_RES                 (float)(95.3f)
#define VBUS_DOWN_RES               (float)(4.99f)
#define VBUS_CONVERSION_FACTOR      (float)(ADC_REF_V*(VBUS_UP_RES+VBUS_DOWN_RES)/VBUS_DOWN_RES/4095.0f)

/* Current sensing parameters */
#define SAMPLE_RES                  (double)(0.002f)	/* Current sense resistor value */
#define AMP_GAIN                    (double)(80.0f)		/* Op-amp gain for current sensing */
#define SAMPLE_CURR_CON_FACTOR      (double)(ADC_REF_V/4095.0f/AMP_GAIN/SAMPLE_RES)

extern uint8_t get_offset_flag;
extern float theta;
extern float angle;
extern float Iq_ref;
extern float EKF_Hz;
extern uint8_t speed_close_loop_flag;
extern uint16_t ADC1ConvertedValue[5];
extern float motor_direction;

void adc_c_adc_sub(void);
void adc_c_dma1_stream6_sub(void);

#endif
