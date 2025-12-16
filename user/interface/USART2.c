/**********************************
 * USART2 Configuration for Motor Telemetry
 * High-speed serial communication for real-time data logging
 **********************************/
#include "../main.h"
#include "USART2.h"

/**
 * @brief Configure USART2 for High-Speed Telemetry
 * 
 * Initializes USART2 peripheral for motor data transmission to PC:
 * - Baud rate: 2 Mbps (high speed for real-time waveform data)
 * - Data format: 8N1 (8 data bits, no parity, 1 stop bit)
 * - Mode: TX and RX enabled
 * - DMA enabled for TX to offload CPU during transmission
 * - Pins: PA2 (TX), PA3 (RX)
 * 
 * Used with VOFA+ or similar PC applications for real-time plotting
 * of motor currents, voltages, speed, and position.
 */
void Usart2_config(void)
{
	USART_InitTypeDef usart_config_struct;
	GPIO_InitTypeDef gpio_config_struct;

	/* Enable GPIO and USART2 peripheral clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Configure GPIO alternate function for USART2 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);  /* TX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);  /* RX */

	/* Configure GPIO pins for USART2 */
	gpio_config_struct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;  /* PA2 (TX), PA3 (RX) */
	gpio_config_struct.GPIO_Mode = GPIO_Mode_AF;
	gpio_config_struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_config_struct);

	/* Configure USART2 parameters */
	USART_DeInit(USART2);
	usart_config_struct.USART_BaudRate = 2000000;  /* 2 Mbps for high-speed data */
	usart_config_struct.USART_WordLength = USART_WordLength_8b;
	usart_config_struct.USART_StopBits = USART_StopBits_1;
	usart_config_struct.USART_Parity = USART_Parity_No;
	usart_config_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_config_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &usart_config_struct);

	/* Enable USART2 and DMA for transmission */
	USART_Cmd(USART2, ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}

/**
 * @brief Send Single Byte via USART2 (Blocking)
 * 
 * Transmits one byte through USART2 and waits for completion.
 * This is a blocking function - use DMA for bulk transfers instead.
 * 
 * @param dat Byte to transmit
 */
void Uart2_Data(unsigned char dat)
{
	USART_SendData(USART2, (uint8_t)dat);
	while (RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC));
}
