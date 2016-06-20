#ifndef __SYSTICK_H__
#define __SYSTICK_H__
#include "stm32f4xx.h"

void SysTick_Init(void);
void StartDelay(int timerid,uint32_t ms);
int IsTimeUp(int timerid);


#endif


