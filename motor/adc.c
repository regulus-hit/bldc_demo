/**********************************
   
**********************************/
#include "main.h"
#include "adc.h"
#include "UpperComputer.h"

#define SEND_BUFFER_SIZE 20  // 16字节数据 + 4字节尾数据
uint8_t send_buffer[SEND_BUFFER_SIZE];
volatile uint8_t dma_tx_busy = 0;


float tempFloat[4];                    //定义的临时变量
uint8_t tempData[16];                    //定义的传输Buffer (3+1)*4

int32_t ia_test,ib_test,ic_test;

double Ia,Ib,Ic;
float Ia_test,Ib_test,Ic_test;
float Vbus;
uint16_t ADC1ConvertedValue[5];
uint16_t i = 0;
uint32_t A_offset,B_offset;
uint8_t get_offset_flag = 0;
uint8_t get_offset_sample_cnt = 0;
u8 oled_display_sample_freq = 0;
u8 speed_close_loop_flag;
float Iq_ref;
float EKF_Hz;

float theta_add;
float theta,myref=0.00001;

float motor_direction = 1.0f;

extern float Rs;
extern float Ls;
extern float flux;
void send_PC(float wave1,float wave2,float wave3,float wave4);

void get_offset(uint32_t *a_offset,uint32_t *b_offset)
{
  if(get_offset_sample_cnt<128)
  {
    *a_offset += ADC1->JDR2;
    *b_offset += ADC1->JDR3;
    get_offset_sample_cnt++;
  }
  else
  {
    *a_offset >>= 7;
    *b_offset >>= 7;
    get_offset_sample_cnt=0;
    TIM_CtrlPWMOutputs(PWM_TIM,DISABLE);
    get_offset_flag = 2;
  }
}

void motor_run(void)
{
  float vbus_temp;
  double ia_temp,ib_temp;
  vbus_temp = (float)(ADC1->JDR1);                                //得到母线电压 adc转换值
  ia_temp = (int16_t)((int16_t)A_offset - (int16_t)ADC1->JDR2);   //得到A相电流 adc转换值
  ib_temp = (int16_t)((int16_t)B_offset - (int16_t)ADC1->JDR3);   //得到B相电流 adc转换值
  Vbus = vbus_temp*VBUS_CONVERSION_FACTOR;                        //通过电压转换因子（通过分压电阻得到）把adc转换值 转化为 真实电压
  Ia = ia_temp*SAMPLE_CURR_CON_FACTOR;                            //通过电流转换因子（通过采样电阻和运算放大倍数得到）把adc采样值转化为真实电流值
  Ib = ib_temp*SAMPLE_CURR_CON_FACTOR;                            //通过电流转换因子（通过采样电阻和运算放大倍数得到）把adc采样值转化为真实电流值
  Ic = -Ia-Ib;                                                    //基于基尔霍夫电流定律，根据AB相电流去计算C相电流
  Ia_test = Ia;
  Ib_test = Ib;
  Ic_test = Ic;
  
  int_test2 = ADC1ConvertedValue[0];
  
  if(speed_close_loop_flag==0)         //速度环闭环切换控制，电机刚启动时速度环不闭环
  {                                    //并且电流参考值缓慢增加（防冲击），速度达到一定值
    if((Iq_ref<MOTOR_STARTUP_CURRENT*motor_direction)) //速度切入闭环
    {                                  //电流环在电机运行过程中全程闭环
      Iq_ref += 0.001f;              //角度在电机刚启动时就闭环运行，无需强拖，得益于卡尔曼滤波做   0.00003f; 
    }                                  //状态观测器低速性能比教好
    else
    {
      speed_close_loop_flag=1;
    }
  }
  else
  {
    if(speed_close_loop_flag==1)
    {
      if(Iq_ref>(MOTOR_STARTUP_CURRENT*motor_direction/2.0f))
      {
        Iq_ref -= 0.001f;
      }
      else
      {
        speed_close_loop_flag=2;
      }
    }
  }
  
  
  float_test3 = Speed_Ref*2.0f*PI;
  
//切换有感或无感方式运行，也就是选择从无感状态观测器方式得到角度和速度信息还是从有感方式得到角度和速度信息  
#ifdef  HALL_FOC_SELECT            //通过条件编译选择有感FOC运行，  

  if((hall_speed*2.0f*PI)>SPEED_LOOP_CLOSE_RAD_S)     //有感方式运行，霍尔传感器得到角度和速度信息
  {
    FOC_Input.Id_ref = 0.0f; //hall
    Speed_Fdk = hall_speed*2.0f*PI;
    FOC_Input.Iq_ref = Speed_Pid_Out;
  }
  else
  {
    FOC_Input.Id_ref = 0.0f; //hall
    FOC_Input.Iq_ref = Iq_ref;
    Speed_Pid.I_Sum = Iq_ref;
  }
  FOC_Input.theta = hall_angle;
  FOC_Input.speed_fdk = hall_speed*2.0f*PI;
  
#endif
  
#ifdef  SENSORLESS_FOC_SELECT            //通过条件编译选择无感FOC运行
  
  //if(FOC_Output.EKF[2]>SPEED_LOOP_CLOSE_RAD_S)      //无感方式运行，状态观测器得到角度和速度信息
	if (fabs(FOC_Output.EKF[2]) > fabs(SPEED_LOOP_CLOSE_RAD_S))
  {
    FOC_Input.Id_ref = 0.0f; //less
    Speed_Fdk = FOC_Output.EKF[2];
    FOC_Input.Iq_ref = Speed_Pid_Out;
  }
  else
  {
    FOC_Input.Id_ref = 0.0f;  //less
    FOC_Input.Iq_ref = Iq_ref;
    Speed_Pid.I_Sum = Iq_ref;
  }
  FOC_Input.theta = FOC_Output.EKF[3]+myref;
  FOC_Input.speed_fdk = FOC_Output.EKF[2];
  
#endif  

  
  
  EKF_Hz = FOC_Output.EKF[2]/(2.0f*PI);
  FOC_Input.Id_ref = 0;//0.0f; //less              
  FOC_Input.Tpwm = PWM_TIM_PULSE_TPWM;         //FOC运行函数需要用到的输入信息
  FOC_Input.Udc = Vbus;
  FOC_Input.Rs = Rs;
  FOC_Input.Ls = Ls;
  FOC_Input.flux = flux;
  
  FOC_Input.ia = Ia;
  FOC_Input.ib = Ib;
  FOC_Input.ic = Ic;           
  foc_algorithm_step();       //整个FOC运行函数（包括无感状态观测器，电流环，SVPWM，坐标变换，电机参数识别）
  
  if(motor_start_stop==1)
  {
    PWM_TIM->CCR1 = (u16)(FOC_Output.Tcmp1);     //通过SVPWM得到的占空比赋值给定时器的寄存器
    PWM_TIM->CCR2 = (u16)(FOC_Output.Tcmp2);
    PWM_TIM->CCR3 = (u16)(FOC_Output.Tcmp3);
  }
  else
  {
    PWM_TIM->CCR1 = PWM_TIM_PULSE>>1;
    PWM_TIM->CCR2 = PWM_TIM_PULSE>>1;
    PWM_TIM->CCR3 = PWM_TIM_PULSE>>1;
  }
  
  drv8301_protection(); 
  //communication_task();//原版上位机任务
	
	////////////////////////////////////////////////////////////////////////////////////////////////
	//此处修改上传波形数据及切换上位机，上位机使用其中一个请注释另一个
	
	//vofa第三方上位机  电流a，电流b，马鞍波，转子位置
	send_PC(FOC_Input.ia,FOC_Input.ib,FOC_Output.Tcmp1,FOC_Output.EKF[3]);
	
	
	//自开发上位机   电流a，电流b，马鞍波，转子位置，参考速度，EFK反馈速度
	//SendWaveformData(FOC_Input.ia,FOC_Input.ib,FOC_Output.Tcmp1,FOC_Output.EKF[3],Speed_Ref*60,Speed_Fdk*60/6.28318548F);
	

	////////////////////////////////////////////////////////////////////////////////////////////////
	
  oled_display_sample_freq++;
  if(oled_display_sample_freq == 10)         //电路板自带OLED显示屏 模拟示波器显示波形功能
  {
    if(display_data_flag==0)
    {
      display_data_buff[display_data_buff_cnt]= (s8)(FOC_Output.EKF[3]*15.0f);
      display_data_buff_cnt++;
      if(display_data_buff_cnt==127)
      {
        display_data_buff_cnt=0;
        display_data_flag=1;
      }
    }
    oled_display_sample_freq=0;
  }
}


void ADC_IRQHandler(void)
{
  
  if((SAMPLE_ADC->SR & ADC_FLAG_JEOC) == ADC_FLAG_JEOC)
  {
    if(get_offset_flag==2)
    {
      hall_angle += hall_angle_add;
      if(hall_angle<0.0f)
      {
        hall_angle += 2.0f*PI;
      }
      else if(hall_angle>(2.0f*PI))
      {
        hall_angle -= 2.0f*PI;
      }
      motor_run();
      
    }
    else
    {
      if(get_offset_flag==1)
      {
        get_offset(&A_offset,&B_offset);
      }
    }
    ADC_ClearFlag(SAMPLE_ADC, ADC_FLAG_JEOC);
  }
}

////发给vofa上位机
//void send_PC(float wave1,float wave2,float wave3,float wave4)
//{
//	tempFloat[0] = wave1;    //(float)转成浮点数
//	tempFloat[1] = wave2;
//	tempFloat[2] = wave3;
//	tempFloat[3] = wave4;
//	memcpy(tempData, (uint8_t *)tempFloat, sizeof(tempFloat));//通过拷贝把数据重新整理
//	 
//	//1	
//	USART_SendData(USART2, tempData[0]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[1]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[2]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[3]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));
//  //2	
//	USART_SendData(USART2, tempData[4]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[5]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[6]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[7]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));
//	//3	
//	USART_SendData(USART2, tempData[8]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[9]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[10]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[11]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));
//	//4	
//	USART_SendData(USART2, tempData[12]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[13]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[14]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, tempData[15]);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));

//	//尾	
//	USART_SendData(USART2, 0x00);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, 0x00);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, 0x80);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//	USART_SendData(USART2, 0x7F);while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));	
//}

//// 发送给VOFA上位机
//void send_PC(float wave1, float wave2, float wave3, float wave4)
//{
//    //static uint8_t wave_index = 0;  // 静态变量记录当前该发送哪个波形
//    
//    // 将4个波形数据存入数组
//    tempFloat[0] = wave1;
//    tempFloat[1] = wave2;
//    tempFloat[2] = wave3;
//    tempFloat[3] = wave4;
//    memcpy(tempData, (uint8_t *)tempFloat, sizeof(tempFloat));
//	
//		for(uint8_t i=0;i<16;i++){

//					USART_SendData(USART2, tempData[i]);
//					while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));
//		}
//    
//    // 发送尾数据（固定4个字节）
//    USART_SendData(USART2, 0x00); while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));    
//    USART_SendData(USART2, 0x00); while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));    
//    USART_SendData(USART2, 0x80); while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));    
//    USART_SendData(USART2, 0x7F); while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));
//    
//    // 更新索引，循环发送0,1,2,3
//    //wave_index = (wave_index + 1) % 2;
//}


// 修改send_PC函数
void send_PC(float wave1, float wave2, float wave3, float wave4)
{
    // 如果DMA正在传输，跳过本次发送以避免冲突
    if(dma_tx_busy) {
        return;
    }
    
    // 将4个波形数据存入数组
    tempFloat[0] = wave1;
    tempFloat[1] = wave2;
    tempFloat[2] = wave3;
    tempFloat[3] = wave4;
    memcpy(tempData, (uint8_t *)tempFloat, sizeof(tempFloat));
    
    // 准备发送缓冲区：16字节波形数据 + 4字节尾数据
    memcpy(send_buffer, tempData, 16);
    send_buffer[16] = 0x00;
    send_buffer[17] = 0x00;
    send_buffer[18] = 0x80;
    send_buffer[19] = 0x7F;
    
    // 配置并启动DMA传输
    DMA_Cmd(DMA1_Stream6, DISABLE);
    
    // 清除所有DMA标志
    DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6);
    
    // 设置DMA参数
    USART2_TX_DMA_STREAM->M0AR = (uint32_t)send_buffer;
    USART2_TX_DMA_STREAM->NDTR = SEND_BUFFER_SIZE;
    
    // 使能DMA传输完成中断
    DMA_ITConfig(USART2_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
    
    // 启动DMA传输
    dma_tx_busy = 1;
    DMA_Cmd(USART2_TX_DMA_STREAM, ENABLE);
    
    // 使能USART2的DMA传输
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}

// DMA传输完成中断处理函数
void DMA1_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6))
    {
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
        
        // 禁用USART2的DMA传输
        USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);
        
        // 禁用DMA
        DMA_Cmd(DMA1_Stream6, DISABLE);
        
        // 清除忙标志
        dma_tx_busy = 0;
    }
}
