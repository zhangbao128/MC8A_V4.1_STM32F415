#include "SysTick.h"
#include "motionTypeDef.h"
#include "encoder.h"
#include "motor.h"

static uint32_t DelayTemp[4];
static uint32_t DelayCounter[4];
extern Motion_CtrlTypeDef motor1;
int CurrentEncoderPos = 0;
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms中断一次
	 * SystemFrequency / 100000	 10us中断一次
	 * SystemFrequency / 1000000 1us中断一次
	 */
	if (SysTick_Config(SystemCoreClock / 40000))//25us中断一次 
	{
		while (1);
	}
}

void StartDelay(int timerid,uint32_t ms)
{
	DelayTemp[timerid] = ms;	
	DelayCounter[timerid] = 0;
}
int IsTimeUp(int timerid)
{
	if(DelayCounter[timerid] >= DelayTemp[timerid])
	{
		return 1;
	}
	return 0;
}

void SysTick_Handler(void)
{
	DelayCounter[0]++;
	DelayCounter[1]++;
	DelayCounter[2]++;
	DelayCounter[3]++;
	
	if(motor1.TriggerPeriod > 0)
	{
		if(motor1.TriggerDir == motor1.Dir)
		{
			CurrentEncoderPos = GetEncoderPos();
			if(!motor1.TriggerEnable)
			{
				if(CurrentEncoderPos >= motor1.TriggerPos)
				{
					SetCameraTriggerPin();
					motor1.TriggerEnable = 1;
				}
			}
			if(!motor1.LightEnable)
			{
				if(motor1.TriggerDir == PLUS ? (CurrentEncoderPos >= (motor1.TriggerPos - motor1.LightPreStep)) : (CurrentEncoderPos <= (motor1.TriggerPos + motor1.LightPreStep)))
				{
					SetLightTriggerPin();
					motor1.LightEnable = 1;
				}
			}
		}
	}
	
	
}
/*********************************************END OF FILE**********************/











