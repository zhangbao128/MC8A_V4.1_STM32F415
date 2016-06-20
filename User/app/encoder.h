#ifndef __ENCODER_H__
#define __ENCODER_H__
#include "stm32f4xx.h"                  // Device header
#include "motionTypeDef.h"

void InitEncoderGpio(void);
void InitEncoderTimer(void);
int GetEncoderPos(void);
void SetEncoderPos(int32_t pos);

#endif

