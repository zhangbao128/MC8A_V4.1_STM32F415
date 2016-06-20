 
#include "stm32f4xx.h"
#include "can.h"
#include "Verify.h"
#include "cmd.h"
#include "motor.h"
#include "motionCtrl_S.h"
#include "encoder.h"
#include "SysTick.h"
//#include "WinWatchDog.h"


extern CanTxMsg TxMsg;	
extern CanRxMsg RxMsg; 

extern unsigned int Rev_flag;
extern unsigned int CardId;
extern unsigned int AxisId;
extern unsigned int Recv_Finished;
extern Motion_CtrlTypeDef motor1;

extern uint32_t RevRIR;
extern uint32_t RevRDTR;
extern uint32_t RevRDLR;
extern uint32_t RevRDHR;

int triggerflag = 1;
int lightflag = 1;
int ledflag = 1;
int wwdg_feed = 1;


void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	//RCC system reset(for debug purpose) //
	RCC_DeInit();
	//RCC_HSICmd(ENABLE);				
  //Enable HSE //
	RCC_HSEConfig(RCC_HSE_ON);
	//Wait till HSE is ready
 	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if (HSEStartUpStatus == SUCCESS)
	{
		////Enable PLL 
    RCC_PLLCmd(DISABLE);
		
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		
    RCC_HCLKConfig(RCC_SYSCLK_Div1);//HCLK = SYSCLK
    
    RCC_PCLK2Config(RCC_HCLK_Div2);//PCLK2 = HCLK / 2 = 168 / 2M = 84M
    
    RCC_PCLK1Config(RCC_HCLK_Div4);//PCLK1 = HCLK / 4 = 168 / 4M = 42M
		
    //PLLCLK = (8MHz/5 * 210)/2 = 168 MHz 
    RCC_PLLConfig(RCC_PLLSource_HSE , 5,210,2,4);
    //Enable PLL
    RCC_PLLCmd(ENABLE);
    //Wait till PLL is ready
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	}
}

void CameraTrigger(void)
{
	switch(triggerflag)
	{
		case 1:
		{
			if(motor1.TriggerEnable)
			{
				//SetCameraTriggerPin();
				StartDelay(0,motor1.TriggerPeriod * 40);
				triggerflag = 2;
			}
		}break;
		case 2:
		{
			if(IsTimeUp(0))
			{
				ClearCameraTriggerPin();
				//motor1.TriggerEnable = 0;
				triggerflag = 1;
			}
		}break;
	}
}

void LightTrigger(void)
{
	switch(lightflag)
	{
		case 1:
		{
			if(motor1.LightEnable)
			{
				//SetLightTriggerPin();
				StartDelay(1,motor1.LightPeriod * 40);
				lightflag = 2;
			}
		}break;
		case 2:
		{
			if(IsTimeUp(1))
			{
				ClearLightTriggerPin();
				//motor1.LightEnable = 0;
				lightflag = 1;
			}
		}break;
	}
}
void LedBlink(void)
{
	switch(ledflag)
	{
		case 1:
		{
			LED_ON();
			StartDelay(2,20000);
			ledflag = 2;
		}break;
		case 2:
		{
			if(IsTimeUp(2))
			{
				LED_OFF();
				StartDelay(2,20000);
				ledflag = 3;
			}
		}break;
		case 3:
		{
			if(IsTimeUp(2))
			{
				ledflag = 1;
			}
		}
	}
}
//void WWatchDogFeed(void)
//{
//	switch(wwdg_feed)
//	{
//		case 1:
//		{
//			StartDelay(3,30);
//			wwdg_feed = 2;
//		}break;
//		case 2:
//		{
//			if(IsTimeUp(3))
//			{
//				WWatch_Refresh(0x7f);
//				wwdg_feed = 1;
//			}
//		}break;
//	}
//}
int main(void)
{	
//	int mtrsta = 0;
	RCC_Configuration();
	NVIC_PriorityGroupConfig(PriorityGroup);
	SysTick_Init();
	InitMotorGpio();
	
//	InitMotorTimer();
//	InitEncoderGpio();
//	InitEncoderTimer();
//	Motor_Data_Init();
//	Init_Default_S();
	
	CAN_Config();	
	
	//WWatchDog_Config();

	while(1)
	{
		//WWatchDogFeed();
		
		CameraTrigger();
		LightTrigger();
		LedBlink();
		
		
		/*硬限位*/
		if(((GPIOC->IDR >> 5 	& 1) || (GPIOB->IDR & 1)) && motor1.IsRunning == RUNNING)
		{
			motor1.LimitFlag = 1;
			if(motor1.RevStopcmd != 1 && motor1.IsRunning != STOPPED && motor1.CurrentIndex <= motor1.ReversId)
			{/*如果已经运行到减速阶段，忽略停止指令*/
					motor1.RevStopcmd = 1;
					motor1.StopIndex = motor1.CurrentIndex;
			}
		}
		/*软限位*/
		if(motor1.Dir == PLUS)
		{
			if(motor1.CurrentPos >= (motor1.pLimit - 100))
			{
				MotorPulseDisable();
				motor1.RevStopcmd = 0;
				motor1.RevChangeRept = 0;
				motor1.IsRunning = STOPPED;
			}
		}
		if(motor1.Dir == REDUCE)
		{
			if(motor1.CurrentPos <= (motor1.nLimit + 100))
			{
				MotorPulseDisable();
				motor1.RevStopcmd = 0;
				motor1.RevChangeRept = 0;
				motor1.IsRunning = STOPPED;
			}
		}


		
		if(Rev_flag)
		{
			Rev_flag = 0;	
			
			RxMsg.IDE = (uint8_t)0x04 & RevRIR;
			RxMsg.ExtId = (uint32_t)0x1FFFFFFF & (RevRIR >> 3);
			RxMsg.RTR = (uint8_t)0x02 & RevRIR;
			RxMsg.DLC = (uint8_t)0x0F & RevRDTR;
			RxMsg.FMI = (uint8_t)0xFF & (RevRDTR >> 8);
			RxMsg.Data[0] = (uint8_t)0xFF & RevRDLR;
			RxMsg.Data[1] = (uint8_t)0xFF & (RevRDLR >> 8);
			RxMsg.Data[2] = (uint8_t)0xFF & (RevRDLR >> 16);
			RxMsg.Data[3] = (uint8_t)0xFF & (RevRDLR >> 24);
			RxMsg.Data[4] = (uint8_t)0xFF & RevRDHR;
			RxMsg.Data[5] = (uint8_t)0xFF & (RevRDHR >> 8);
			RxMsg.Data[6] = (uint8_t)0xFF & (RevRDHR >> 16);
			RxMsg.Data[7] = (uint8_t)0xFF & (RevRDHR >> 24);	
			
			if(CheckVerifyCode() == 0)
			{
				Recv_Finished = 1;
			}
			CAN1->RF0R |= CAN_RF0R_RFOM0;
			CAN1->IER |= CAN_IT_FMP0;			
		}
		if(!Recv_Finished)continue;
		Deal_Cmd();
		Recv_Finished = 0;
	}
}

/*********************************************END OF FILE**********************/


