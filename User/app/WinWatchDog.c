#include "motor.h"
#include "WinWatchDog.h"

//看门狗计数周期 = 4096 * 8 / 42M = 780.2us
//780.2 * (127 - 100) < 刷新时间 < 780.2 * (127 - 64)
//21ms < 刷新时间 < 49ms
int WWatchDog_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	unsigned int wwdgcfr = 0;
  //NVIC_PriorityGroupConfig(PriorityGroup); 
  NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = WWdgPreemptionPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = WWdgSubPriority;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

	wwdgcfr  = 1 << 9 | 3 << 7 | 100;
	//打开WWDG时钟
	RCC->APB1ENR |= 1 << 11;
	//设置WWDG窗口值,时基预分频
	WWDG->CFR = wwdgcfr;
	//使能WWDG,刷新计数值
	WWDG->CR = 0xff;
	//清楚标志位
	WWDG->SR = 0;	
	return 0;
}

void WWatch_Refresh(unsigned char val)
{
	WWDG->CR = 0x80 | (val & 0x7f);
}

void WWDG_IRQHandler(void)
{
	MotorPulseDisable();
	MotorEnable(DISABLE);
}
