#ifndef __LOW_TASK_H_
#define __LOW_TASK_H_

#include <stdint.h>

extern uint8_t motor_start_stop;
extern uint8_t key1_press_flag;

/**
 * @brief Start motor operation
 * 
 * Initializes FOC, enables PWM outputs, and begins motor control.
 */
void motor_start(void);

/**
 * @brief Stop motor operation
 * 
 * Disables PWM outputs and stops motor.
 */
void motor_stop(void);

#endif
