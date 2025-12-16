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

#include "public.h"
#include "../motor/foc_define_parameter.h"
#include "arm_math.h"
#include "hardware/board_config.h"
#include "interface/oled_font.h"
#include "interface/oled.h"
#include "../motor/drivers/drv8301.h"
#include "app/exti.h"
#include "../motor/control/foc_algorithm.h"
#include "../motor/drivers/adc.h"
#include "../motor/sensors/hall_sensor.h"
#include "../motor/control/low_task.h"

#include "interface/oled_display.h"

#include "../motor/control/speed_pid.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
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

void SysTick_update(void);
extern uint32_t GetSysUptime(void);

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
