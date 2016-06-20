#ifndef __WINWATCHDOG_H_
#define __WINWATCHDOG_H_

#include <stdint.h>
#include "stm32f4xx_wwdg.h"
#include "stm32f4xx.h"
#include "motionTypeDef.h"

int WWatchDog_Config(void);
void WWatch_Refresh(unsigned char val);

#endif
