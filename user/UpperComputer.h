#ifndef __UPPERCOMPUTER_H__
#define __UPPERCOMPUTER_H__

void USART_ConfigInterrupt(void);
void SendWaveformData(float wave1, float wave2, float wave3, float wave4, float referenceSpeed, float feedbackSpeed);
void SendSpeedData(uint8_t command, float speed);
void SendReferenceSpeed(float referenceSpeed);
void SendActualSpeed(float actualSpeed);

#endif
