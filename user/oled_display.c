/**********************************

**********************************/
#include "main.h"
#include "oled_display.h"
#include "drv8301.h"

uint8_t usb_open_display_flag;
uint8_t data_upload_display_flag;
uint8_t motor_run_display_flag;
uint8_t motor_run_display_flag_pre=1;
uint8_t display_static_flag;

uint8_t init_dispaly_flag;
uint8_t display_index;
uint8_t display_index_key;
uint8_t display_cnt;

uint8_t display_flag;
uint8_t clear_display_flag=0;
uint8_t drv8301_fault_flag = 0;

uint16_t drv8301_reg_read1[4];

void oled_display_handle(void)
{
	if(drv8301_fault_flag == 0)
	{
		if(display_flag==0)
		{
			clear_display_flag=0;
			motor_run_display_flag_pre = !motor_run_display_flag;
			OLED_DrawBMP(0,0,128,8,Logo);		
		}
		else if(display_flag==1)
		{
			if(clear_display_flag==0)
			{
				OLED_Clear();
				OLED_ShowString(0,2,"r_ref:");
				OLED_ShowString(0,3,"r_fbk:");
				OLED_ShowString(0,4,"hall_f:");
				OLED_ShowString(0,5,"ekf_f:");
				OLED_ShowString(0,6,"Iq_ref: .");
				clear_display_flag=1;
			}
			if(motor_run_display_flag_pre!=motor_run_display_flag)
			{
				if(motor_run_display_flag==1)
				{
				  OLED_ShowString(0,0,"Motor:run ");
				}
				else
				{
				  OLED_ShowString(0,0,"Motor:stop");
				}
				motor_run_display_flag_pre = motor_run_display_flag;
			}

			OLED_ShowNum(7*8,2,(uint32_t)Speed_Ref,3,16);
			OLED_ShowNum(7*8,3,(uint32_t)Speed_Fdk,3,16);
			OLED_ShowNum(7*8,4,(uint32_t)hall_speed,3,16);
			OLED_ShowNum(7*8,5,(uint32_t)EKF_Hz,3,16);

			
			OLED_ShowNum(7*8,6,(uint32_t)FOC_Input.Iq_ref,1,16);
			OLED_ShowNum(9*8,6,(uint32_t)(FOC_Input.Iq_ref*100),2,16);
			
		}
		else if(display_flag==2)
		{
			oled_display();
		}
	}
	else
	{
		OLED_DrawBMP(0,0,128,8,fault);
	}
}
