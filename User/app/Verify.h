#ifndef __VERIFY_H_
#define __VERIFY_H_

#include <stdint.h>
#include "stm32f4xx_can.h"
#include "stm32f4xx.h"
int CheckVerifyCode(void);
unsigned int GetVerifyCode(unsigned int CardId,unsigned int axisid,unsigned int cmd,uint8_t data[]);

#endif
