/**
 * UpperComputer.c
 *
 * - Uses STM32F4 StdPeriph macros/types (USART2, flags, NVIC, etc.)
 * - Replaces magic numbers with named constants
 * - Keeps ISR lean and deterministic (no malloc; bounded loops only)
 * - Frame protocol:
 *    Response frames: 0xAA 0xAA [CMD] [LEN] [PAYLOAD...] [CHK] 0x55 0x55
 *    Waveform frames: 0xAA 0xAA 0xAA [LEN] [PAYLOAD...] [CHK] 0x55 0x55
 *    CHK for response frames = sum over {CMD, LEN, PAYLOAD} (uint8_t)
 *    CHK for waveform frames = (0xAA + sum over PAYLOAD) (uint8_t)
 */

#include <string.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

#include "../public.h"
#include "../main.h"
#include "../app/exti.h"
#include "../../motor/control/low_task.h"
#include "UpperComputer.h"


/* ========================== Module-local state ============================= */

static volatile uint8_t s_frameStarted = 0;
static volatile uint8_t s_rxIndex = 0;

/* Current RX frame under construction */
static volatile uint8_t s_cmd = 0;
static volatile uint8_t s_len = 0;
static volatile uint8_t s_payload[UC_MAX_PAYLOAD];
static volatile uint8_t s_checksum = 0;

/* Waveform extended-field cadence */
static uint16_t s_waveCounter = 0;

uint8_t mcu_uid[12];
/* ============================ Local utilities ============================== */

/* Sum of bytes modulo 256. */
static uint8_t UC_CalculateChecksum(const uint8_t* data, uint8_t len)
{
	uint8_t sum = 0;
	for (uint8_t i = 0; i < len; i++)
	{
		sum = (uint8_t)(sum + data[i]);
	}
	return sum;
}

/* USART2 blocking send (bounded loop per byte, using TXE). */
static void UC_SendBytes(const uint8_t* buf, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++)
	{
		while (USART_GetFlagStatus(UC_USARTx, UC_USART_FLAG_TXE) == RESET)
		{
			/* wait for TXE */
		}
		USART_SendData(UC_USARTx, buf[i]);
	}
}

/* Read 96-bit unique device ID into 12-byte buffer, big-endian word order. */
static void UC_ReadMCUID(uint8_t out12[12])
{
	const uint32_t uid0 = *(const uint32_t*)(UID_BASE + 0x00);
	const uint32_t uid1 = *(const uint32_t*)(UID_BASE + 0x04);
	const uint32_t uid2 = *(const uint32_t*)(UID_BASE + 0x08);

	out12[0]  = (uint8_t)(uid0 >> 24);
	out12[1]  = (uint8_t)(uid0 >> 16);
	out12[2]  = (uint8_t)(uid0 >> 8);
	out12[3]  = (uint8_t)(uid0 >> 0);

	out12[4]  = (uint8_t)(uid1 >> 24);
	out12[5]  = (uint8_t)(uid1 >> 16);
	out12[6]  = (uint8_t)(uid1 >> 8);
	out12[7]  = (uint8_t)(uid1 >> 0);

	out12[8]  = (uint8_t)(uid2 >> 24);
	out12[9]  = (uint8_t)(uid2 >> 16);
	out12[10] = (uint8_t)(uid2 >> 8);
	out12[11] = (uint8_t)(uid2 >> 0);
}

/* Pack response frame: 0xAA 0xAA [CMD] [LEN] [PAYLOAD...] [CHK] 0x55 0x55 */
static uint16_t UC_PackResponseFrame(uint8_t* out, uint8_t cmd, const uint8_t* payload, uint8_t len)
{
	out[0] = UC_FRAME_HEAD;
	out[1] = UC_FRAME_HEAD;
	out[2] = cmd;
	out[3] = len;

	if (len && payload) {
		memcpy(&out[4], payload, len);
	}

	const uint8_t chk = UC_CalculateChecksum(&out[2], (uint8_t)(len + 2u));
	out[4u + len] = chk;
	out[5u + len] = UC_FRAME_TAIL;
	out[6u + len] = UC_FRAME_TAIL;

	return (uint16_t)(7u + len); /* total bytes */
}

/* ============================== API (header) =============================== */

/* Configure USART2 RXNE interrupt and NVIC. */
void USART_ConfigInterrupt(void)
{
	/* Enable RXNE interrupt on USART2 */
	USART_ITConfig(UC_USARTx, UC_USART_IT_RXNE, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* NVIC configuration for USART2 IRQ */
	NVIC_InitStructure.NVIC_IRQChannel = UC_USARTx_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/* Send speed telemetry frame with command and float speed (little-endian). */
void SendSpeedData(uint8_t command, float speed)
{
	uint8_t frame[11]; /* 2H + CMD + LEN(4) + 4B + CHK + 2T = 11 */
	frame[0] = UC_FRAME_HEAD;
	frame[1] = UC_FRAME_HEAD;
	frame[2] = command;
	frame[3] = 4u;

	memcpy(&frame[4], &speed, sizeof(float));

	frame[8] = UC_CalculateChecksum(&frame[2], 6u);
	frame[9] = UC_FRAME_TAIL;
	frame[10] = UC_FRAME_TAIL;

	UC_SendBytes(frame, (uint16_t)sizeof(frame));
}

/* Reference speed (rps -> rpm) */
void SendReferenceSpeed(float referenceSpeed)
{
	/* Ghidra logic: reference speed scaled by 60 (rps->rpm) */
	const float rpm = referenceSpeed * 60.0f;
	SendSpeedData(0xA0u, rpm);
}

/* Actual speed (rad/s -> rpm) */
void SendActualSpeed(float actualSpeed)
{
	const float rpm = (actualSpeed * 60.0f) / MATH_2PI;
	SendSpeedData(0xA1u, rpm);
}

/* Send waveform packet with 4 floats; periodically append 2 x int16 (ref, fb). */
void SendWaveformData(float wave1, float wave2, float wave3, float wave4,
											float referenceSpeed, float feedbackSpeed)
{
	/* Triple-head waveform frame: 0xAA 0xAA 0xAA */
	uint8_t frame[3 + 1 + UC_WAVE_EXT_LEN + 1 + 2]; /* head3 + len + payload + chk + tail2 */

	frame[0] = UC_FRAME_HEAD;
	frame[1] = UC_FRAME_HEAD;
	frame[2] = UC_FRAME_HEAD;

	/* Decide payload length: base 16 bytes or extended 20 bytes */
	uint8_t payloadLen = UC_WAVE_BASE_LEN;
	s_waveCounter++;
	if (s_waveCounter >= UC_WAVE_EXT_PERIOD) {
		payloadLen = UC_WAVE_EXT_LEN;
		s_waveCounter = 0;
	}

	frame[3] = payloadLen;

	/* Payload: 4 floats */
	uint8_t* p = &frame[4];
	memcpy(p + 0u,  &wave1, sizeof(float));
	memcpy(p + 4u,  &wave2, sizeof(float));
	memcpy(p + 8u,  &wave3, sizeof(float));
	memcpy(p + 12u, &wave4, sizeof(float));

	if (payloadLen == UC_WAVE_EXT_LEN) {
		/* Append two int16_t values (saturated cast) */
		int32_t r_int = (int32_t)(referenceSpeed);
		int32_t f_int = (int32_t)(feedbackSpeed);
		if (r_int > 32767) r_int = 32767; else if (r_int < -32768) r_int = -32768;
		if (f_int > 32767) f_int = 32767; else if (f_int < -32768) f_int = -32768;
		int16_t r16 = (int16_t)r_int;
		int16_t f16 = (int16_t)f_int;
		memcpy(p + 16u, &r16, sizeof(int16_t));
		memcpy(p + 18u, &f16, sizeof(int16_t));
	}

	/* Checksum: 0xAA + sum(payload) */
	uint8_t sum = UC_FRAME_HEAD;
	for (uint8_t i = 0; i < payloadLen; i++) {
		sum = (uint8_t)(sum + p[i]);
	}

	frame[4u + payloadLen] = sum;
	frame[5u + payloadLen] = UC_FRAME_TAIL;
	frame[6u + payloadLen] = UC_FRAME_TAIL;

	UC_SendBytes(frame, (uint16_t)(7u + payloadLen));
}

/* ============================ USART2 IRQ handler =========================== */
/* RX state machine for frames: 0xAA 0xAA [CMD] [LEN] [PAYLOAD...] [CHK] 0x55 0x55 */
void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(UC_USARTx, UC_USART_IT_RXNE) != RESET)
	{
		const uint8_t data = (uint8_t)USART_ReceiveData(UC_USARTx);

		if (!s_frameStarted)
		{
			/* Looking for 0xAA 0xAA */
			if (data == UC_FRAME_HEAD && s_rxIndex == 0)
			{
				s_rxIndex = 1;
			}
			else if (data == UC_FRAME_HEAD && s_rxIndex == 1)
			{
				s_frameStarted = 1;
				s_rxIndex = 0; /* next: CMD */
			}
			else
			{
				s_rxIndex = 0;
			}
		}
		else
		{
			/* Build frame fields in order: CMD, LEN, PAYLOAD..., CHK, 0x55, 0x55 */
			if (s_rxIndex == 0)
			{
				s_cmd = data;
			}
			else if (s_rxIndex == 1)
			{
				s_len = data;
				if (s_len > UC_MAX_PAYLOAD)
				{
					/* invalid length: reset */
					s_frameStarted = 0;
					s_rxIndex = 0;
					USART_ClearITPendingBit(UC_USARTx, UC_USART_IT_RXNE);
					return;
				}
			}
			else if (s_rxIndex >= 2 && s_rxIndex < (uint8_t)(2u + s_len))
			{
				s_payload[s_rxIndex - 2u] = data;
			}
			else if (s_rxIndex == (uint8_t)(2u + s_len))
			{
				s_checksum = data;
			}
			else if (s_rxIndex == (uint8_t)(3u + s_len))
			{
				/* First tail byte */
				if (data != UC_FRAME_TAIL)
				{
					s_frameStarted = 0;
					s_rxIndex = 0;
					USART_ClearITPendingBit(UC_USARTx, UC_USART_IT_RXNE);
					return;
				}
			}
			else if (s_rxIndex == (uint8_t)(4u + s_len))
			{
				/* Second tail byte -> complete frame */
				if (data == UC_FRAME_TAIL)
				{
					/* Verify checksum */
					uint8_t hdrAndLen[2];
					hdrAndLen[0] = s_cmd;
					hdrAndLen[1] = s_len;

					uint8_t calc = UC_CalculateChecksum(hdrAndLen, 2u);
					if (s_len > 0)
					{
						calc = (uint8_t)(calc + UC_CalculateChecksum((const uint8_t*)s_payload, s_len));
					}

					if (calc == s_checksum)
					{
						/* Process command and respond */
						uint8_t respBuf[4 + UC_MAX_PAYLOAD + 3];
						uint8_t respLen = 0;
						uint8_t outLen = 0;

						switch (s_cmd)
						{
							case 0x00u:
								/* Read MCU Unique ID */
								UC_ReadMCUID(mcu_uid);
								respLen = 12u;
								outLen = (uint8_t)UC_PackResponseFrame(respBuf, s_cmd, mcu_uid, respLen);
								break;

							case 0x01u: /* key1: press */
								key1_flag = 0;
								key1_press_flag = 1;
								respLen = s_len;
								outLen = (uint8_t)UC_PackResponseFrame(respBuf, s_cmd, (const uint8_t*)s_payload, respLen);
								break;

							case 0x02u: /* key2 */
								key2_flag = 1;
								respLen = s_len;
								outLen = (uint8_t)UC_PackResponseFrame(respBuf, s_cmd, (const uint8_t*)s_payload, respLen);
								break;

							case 0x03u: /* key3 */
								key3_flag = 1;
								respLen = s_len;
								outLen = (uint8_t)UC_PackResponseFrame(respBuf, s_cmd, (const uint8_t*)s_payload, respLen);
								break;

							case 0x04u: /* reverse direction (stop -> flip -> start) */
								motor_stop();
								motor_direction = -motor_direction;
								motor_start();
								respLen = s_len;
								outLen = (uint8_t)UC_PackResponseFrame(respBuf, s_cmd, (const uint8_t*)s_payload, respLen);
								break;

							default:
								/* Unknown command: echo payload */
								respLen = s_len;
								outLen = (uint8_t)UC_PackResponseFrame(respBuf, s_cmd, (const uint8_t*)s_payload, respLen);
								break;
						}

						/* Send response (bounded loop) */
						if (outLen > 0)
						{
							UC_SendBytes(respBuf, outLen);
						}
					}
				}
				/* Reset frame state after completion (success or failure) */
				s_frameStarted = 0;
				s_rxIndex = 0;
				USART_ClearITPendingBit(UC_USARTx, UC_USART_IT_RXNE);
				return;
			}

			s_rxIndex++;
		}

		USART_ClearITPendingBit(UC_USARTx, UC_USART_IT_RXNE);
	}
}
