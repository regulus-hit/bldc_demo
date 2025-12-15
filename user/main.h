/**
	******************************************************************************
	* @file    Project/STM32F4xx_StdPeriph_Templates/main.h 
	* @author  MCD Application Team
	* @version V1.8.0
	* @date    04-November-2016
	* @brief   Header for main.c module
	******************************************************************************
	* @attention
	*
	* <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
	*
	* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
	* You may not use this file except in compliance with the License.
	* You may obtain a copy of the License at:
	*
	*        http://www.st.com/software_license_agreement_liberty_v2
	*
	* Unless required by applicable law or agreed to in writing, software 
	* distributed under the License is distributed on an "AS IS" BASIS, 
	* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	* See the License for the specific language governing permissions and
	* limitations under the License.
	*
	******************************************************************************
	*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include <math.h>

#include "stm32f4xx.h"

#include "foc_define_parameter.h"
#include "arm_math.h"
#include "board_config.h"
#include "oled_font.h"
#include "oled.h"
#include "drv8301.h"
#include "exti.h"
#include "foc_algorithm.h"
#include "adc.h"
#include "hall_sensor.h"
#include "communication.h"
#include "low_task.h"

#include "oled_display.h"

#include "speed_pid.h"
#include "pc_communication_init.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define MATH_PI 		3.1415926536F
#define MATH_2PI 		6.2831853072F
#define MATH_sqrt_3		1.7320508075F
#define MATH_cos_30		0.8660254038F
//#define MATH_cos_30	0.866025388F
#define MATH_cos_60		0.500F
/* Exported functions ------------------------------------------------------- */
typedef enum
{
	FALSE = 0, TRUE  = !FALSE
}
bool;

void SysTick_Handler(void);
void TIM1_UP_TIM10_IRQHandler(void);
void OTG_FS_IRQHandler(void);

void TimingDelay_Decrement(void);
extern void Delay(__IO uint32_t nTime);

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
