#include "main.h"
#include "USART2.h"

void Usart2_config(void)
{ 
    USART_InitTypeDef usart_config_struct;
    GPIO_InitTypeDef  gpio_config_struct; 
    
		// Clock Config
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* 打开GPIO时钟 */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

		// GPIO AF Config ?
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);

		//GPIO_StructInit(&gpio_config_struct);////复位 gpio_config_struct -> default
		gpio_config_struct.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;//PA2,PA3 USART2	
		gpio_config_struct.GPIO_Mode = GPIO_Mode_AF;
		gpio_config_struct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &gpio_config_struct);
	
    // USART Config	
    USART_DeInit(USART2);
    usart_config_struct.USART_BaudRate = 2000000;
    usart_config_struct.USART_WordLength = USART_WordLength_8b;
    usart_config_struct.USART_StopBits = USART_StopBits_1;
    usart_config_struct.USART_Parity = USART_Parity_No;
    usart_config_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_config_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
    USART_Init(USART2, &usart_config_struct);

    USART_Cmd(USART2, ENABLE);	
		USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}



void Uart2_Data(unsigned char dat)
{
	  USART_SendData(USART2, (uint8_t)dat);
    while(RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));
}
