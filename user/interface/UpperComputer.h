#ifndef __UPPERCOMPUTER_H__
#define __UPPERCOMPUTER_H__

#include <stdint.h>

/* ========================= Protocol & HW constants ========================= */

#define UC_FRAME_HEAD            ((uint8_t)0xAA)
#define UC_FRAME_TAIL            ((uint8_t)0x55)

/* USART2 base is 0x40004400 on APB1 (StdPeriph uses USART2 handle) */
#define UC_USARTx                USART2
#define UC_USART_IT_RXNE         USART_IT_RXNE
#define UC_USART_FLAG_TXE        USART_FLAG_TXE
#define UC_USARTx_IRQn           USART2_IRQn

/* STM32F4 Unique Device ID base (RM0390, Reference manual) */
#define UID_BASE                 ((uint32_t)0x1FFF7A10U)

/* Payload sizing (keep bounded in ISR) */
#define UC_MAX_PAYLOAD           (64u)

/* Waveform framing:
 * - Base payload is 4 floats = 16 bytes
 * - Occasionally append 2 x int16_t (reference, feedback) = +4 bytes
 */
#define UC_WAVE_BASE_LEN         (16u)
#define UC_WAVE_EXT_LEN          (20u)
#define UC_WAVE_EXT_PERIOD       (500u)  /* send extended fields every 500 frames */

/* Math constants */
#define TWO_PI_F                 (6.283185307179586f)

void USART_ConfigInterrupt(void);
void SendWaveformData(float wave1, float wave2, float wave3, float wave4, float referenceSpeed, float feedbackSpeed);
void SendSpeedData(uint8_t command, float speed);
void SendReferenceSpeed(float referenceSpeed);
void SendActualSpeed(float actualSpeed);

#endif
