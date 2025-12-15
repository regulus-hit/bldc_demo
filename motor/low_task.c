/**********************************
      
**********************************/
#include "main.h"
#include "low_task.h"
#include "adc.h"


u16 hz_100_cnt = 0;
uint8_t motor_start_stop = 0;
uint8_t motor_start_stop_pre = 1;

uint16_t key1_cnt;
uint8_t key1_press_flag = 0;

void motor_start(void)
{
  GPIO_SetBits(GPIOC,GPIO_Pin_9);
  foc_algorithm_initialize();
  Speed_Ref=motor_direction*25.0F;//启动转速
  speed_close_loop_flag=0;
  Iq_ref=0.0f;

  hall_angle_add=0.0005f;
  hall_speed = 0.0f;
  TIM_CtrlPWMOutputs(PWM_TIM,ENABLE);
  
  motor_run_display_flag = 1;
}
void motor_stop(void)
{
  GPIO_ResetBits(GPIOC,GPIO_Pin_9);
  TIM_CtrlPWMOutputs(PWM_TIM,DISABLE);
  motor_run_display_flag = 0;
}


void low_control_task(void)
{
	if(get_offset_flag == 2)
  {
    if(motor_start_stop_pre!=motor_start_stop)
    {
      motor_start_stop_pre=motor_start_stop;
      if(motor_start_stop == 1)
      {
        motor_start();
      }
      else
      {
        motor_stop();
      }     
    }
  }
	
	
  if(key1_flag==1)
  {
		key1_press_flag = 1;
  }
	if(key1_press_flag){
		key1_cnt++;
		if(key1_cnt<100 && key1_flag == 0)
		{	
			if(motor_start_stop==0)
			{
				motor_start_stop=1;
			}
			else
			{
				motor_start_stop=0;
			}	
			key1_flag=0;
			key1_cnt = 0;
			key1_press_flag = 0;
		}else if(key1_cnt>100){
			motor_stop();
			motor_direction = -motor_direction;
			motor_start();	
			key1_cnt = 0;
			key1_flag = 0;
			key1_press_flag = 0;
		}
	} 
	
  if(key2_flag==1)
  {
		display_flag=1;//显示运行参数
		if(motor_direction!= -1.0f)	{
			if(Speed_Ref>25.0f)//最小速度
				Speed_Ref-=5.0f;//步进
		}else{
			if(Speed_Ref<-25.0f)//最小速度
				Speed_Ref+=5.0f;//步进		
		}
 
    key2_flag=0;
  }
	
  if(key3_flag==1)
  {
		if(motor_direction!= -1.0f)	
		{
		    Speed_Ref+=5.0f;//步进
			  if(Speed_Ref>200.0f)//最大速度 50
			    Speed_Ref=200.0f;
		}else{
				Speed_Ref-=4.0f;//步进
				if(Speed_Ref<-200.0f)//最大速度 50
					Speed_Ref=-200.0f;
		}   
		key3_flag=0;		
  }
}





void SysTick_Handler(void)
{
  //rtspeed_ref=20.0F;
  if(drv8301_init_ok_flag==1)
  {
     drv8301_protection();
  }
  Speed_Pid_Calc(Speed_Ref,Speed_Fdk,&Speed_Pid_Out,&Speed_Pid);
  hz_100_cnt++;
  if(hz_100_cnt==10)
  {
    //communication_handle();
    low_control_task();
    TimingDelay_Decrement();
    hz_100_cnt=0; 
  }
  
}
