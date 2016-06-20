
#include "motor.h"
#include "motionCtrl_S.h"
#include "motionCtrl_T.h"
#include "encoder.h"


extern Motion_TableTypeDef motion_table_S[SPEED_LEVEL][TOTAL_POINTS];/*S型曲线运行参数*/
extern Speed_LevelTypeDef speed_table_S[SPEED_LEVEL];/*S型曲线速度参数*/
extern uint32_t MinSteps_S[SPEED_LEVEL];/**/
extern double MinTimes_S[SPEED_LEVEL];
extern uint32_t RefDistance_S[SPEED_LEVEL][TOTAL_POINTS];
extern uint32_t RefDistance_T[SPEED_LEVEL][TOTAL_POINTS];
/*匀速运动数据*/
Motion_TableTypeDef motion_table_Cnt[TOTAL_POINTS];
Speed_LevelTypeDef speed_Cnt;

Motion_CtrlTypeDef motor1;
uint32_t pwm1_temp = 0;
uint32_t pwm1_periodtemp = 0;
uint32_t pwm_cnt = 0;


/***********change move************/
uint16_t MirrorIndex = 0;		//减速点对应的加速点
int32_t pwm_cnt_change = 0;

extern int steps;
uint8_t djustcmd = 0;

int StartPfMove(Motion_CtrlTypeDef *p ,uint32_t *pwm_temp);
int StartCntPfMove(Motion_CtrlTypeDef *p ,uint32_t *pwm_temp);

//extern int CurrentEncoderPos;
void Motor_Data_Init(void)
{
	motor1.DstPulse = 0;
	motor1.DstPos = 0;
	motor1.CurrentPos = 0;
	
	motor1.MinSteps = 0;
	motor1.CurrentIndex = 0;
	motor1.StopIndex = 0;
	motor1.PwmCntMul = 0;
	motor1.PwmCntRem = 0;
	
	motor1.MinTimes = 0;
	motor1.Time_Cost_Act = 0;
	motor1.table = 0;
	motor1.speed = 0;
	motor1.Dir = PLUS;
	motor1.IsRunning = STOPPED;
	motor1.RevStopcmd = 0;
	motor1.Enable = 0;
	motor1.pf = 0;
	motor1.CntSpeed = 0;
	motor1.MovingStyle = 0;
	
	motor1.RevChangeRept = 0;
	motor1.RevChangeCmd = 0;
	motor1.RevChangeIndex = 0;
	motor1.RemainPulse = 0;
	
	motor1.TriggerDir = PLUS;
	motor1.TriggerPos = 0;
	motor1.TriggerPeriod = 0;
	motor1.TriggerEnable = 0;

	motor1.LightPreStep = 0;
	motor1.LightPeriod = 0;
	motor1.LightEnable = 0;
	
	motor1.pLimit = 0x7FFFFFFF;
	motor1.nLimit = 0x80000000;
	
	motor1.LimitFlag = 0;
	
	motor1.SearchEnable = 0;
	motor1.SearchStatus = 0;
	
//	motor1.EncoderEnable = 1;
}
void InitMotorGpio(void)
{
    /*
     * GPIOA4 --> DIR       (output)
     * GPIOA5 --> EN        (output)
     * GPIOA6 --> TRIGGER   (output)
     * GPIOA7 --> LIGHT     (output)
		 * GPIOC11 --> LED			(output)
		 * GPIOA8 --> Pulse			(output)
	
		 * GPIOA3 ---> STOP 		(input)
		 * GPIOC4 --> Alarm			(input)
		 * GPIOC5 --> LimitDown	(input)
		 * GPIOB0 --> lIMITuP		(input)
		 * GPIOB1 --> Home      (input)
     * */
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC,ENABLE);
	
	//Pulse
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;    
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
//	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	
	//Light,Triggle,Enable,Dir
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);
	GPIO_ResetBits(GPIOA,GPIO_Pin_7);
	
	//LED
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_SetBits(GPIOC,GPIO_Pin_11);
	
	//STOP
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//Alarm,Limitdown
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	//limitup,home
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void InitMasterTimer_1(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	//1.使能TIM1,RCC时钟
	RCC->APB2ENR |= 1;
	//2.禁能TIM1计数,使能ARR自动重装,计数模式边沿对称,主模式Updateas trigger output (TRGO),then be used as a prescaler for a slave timer.
	TIM1->CR1 &= ~1;
	TIM1->CR1 |= 1 << 7 ;//| 0x6 << 5;
	//PWM1输出比较模式,使能perload
	TIM1->CCMR1 = 0x6 << 4 | 1 << 3;
	//使能TIM_CH1输出,低电平有效，互补输出使能，高电平有效
	TIM1->CCER = 0x7;
	//主输出使能,运行模式输出使能,空闲模式输出使能
	TIM1->BDTR = 1 << 15 | 3 << 10;
	//主模式选择Updata模式 
	TIM1->CR2 = 0x2 << 4;
	//使能Update中断
	TIM1->DIER = 1;
	
	
	TIM1->PSC = 0;	//主定时器以168M的平率计数
	TIM1->CNT = 0;
	TIM1->ARR = 13;
	TIM1->CCR1 = 7;

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;//TIM8_UP_TIM13_IRQn;                 //更新事件
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =Motor1PreemptionPriority;        //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =Motor1SubPriority;              //响应优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //允许中断
	NVIC_Init(&NVIC_InitStructure);
}
void InitMotorTimer(void)
{
		InitMasterTimer_1();
//		InitSlaveTimer_8();
}
int32_t GetMotor_Pos(void)
{
   return motor1.CurrentPos;
}

//bit5				bit4				bit3				bit2				bit1				bit0
//stop				IsRunning		home				Alarm				Limit1			Limit2
uint16_t GetMotor_Status(void)
{
	uint16_t status = 0;
	status += (GPIOA->IDR >> 3 	& 1) 	<< 5;		//stop
	status += motor1.IsRunning 				<< 4;		//IsRunning
	
	status += (GPIOB->IDR >> 1 	& 1) 	<< 3;		//home
	status += (GPIOC->IDR >> 4 	& 1)  << 2;		//Alarm
	status += (GPIOB->IDR 			& 1) 	<< 1;		//Limit1
	status += (GPIOC->IDR >> 5 	& 1) 	<< 0;		//Limit2
	
  return status;
}

static void MotorEvent(Motion_CtrlTypeDef *p,uint32_t *pwm_periodtemp,uint32_t *pwm_temp)
{
	p->Dir == PLUS ? p->CurrentPos++: p->CurrentPos--;
	
//	/*伺服报警*/
//	if((GPIOC->IDR & 0x10) == 0)
//	{
//			MotorPulseDisable();
//			motor1.RevStopcmd = 0;
//			motor1.RevChangeRept = 0;
//			motor1.IsRunning = STOPPED;
//			return;
//	}

	/*零点搜索*/
	if(p->SearchEnable)
	{
		if(((GPIOB->IDR >> 1) & 1) == p->SearchStatus)
		{			
			MotorPulseDisable();
			p->SearchEnable = 0;
			p->IsRunning = STOPPED;
			return;
		}
	}
	
	/*匀速运动*/
	if(p->MovingStyle == CNT_MOVING)
	{
		if(p->RevStopcmd)
		{
			MotorPulseDisable();
			p->RevStopcmd = 0;
			p->RevChangeRept = 0;
			p->IsRunning = STOPPED;
			return ;
		}
		if(p->Dir == PLUS ? (p->CurrentPos >=  p->DstPos) : (p->DstPos >= p->CurrentPos))
		{
			MotorPulseDisable();
			p->RevChangeRept = 0;
			p->IsRunning = STOPPED;
		}
		return;
	}
	
	/*曲线运动*/
	pwm1_periodtemp++;
	djustcmd = (p->CurrentIndex != p->ReversId) ? (pwm1_periodtemp >= p->table[p->CurrentIndex].Pwm_Counter ? 1 : 0) : (pwm1_periodtemp >= pwm1_temp ? 1 : 0);
	
	if(djustcmd)
	{
		pwm1_periodtemp = 0;
		if(p->RevStopcmd)
		{
			//对称减速阶段
			p->RevStopcmd = 0;
			p->CurrentIndex = (START_POINTS << 1) - p->StopIndex + 1;
		}
		
		switch(p->RevChangeCmd)
		{
			case 0://正常pfmove
			{
				if(p->CurrentIndex == p->ReversId)
				{
					if(p->PwmCntMul > 0)
					{
						pwm1_temp = CNTPWM_SIZE_MAX;
						p->PwmCntMul--;
					}
					else if(p->PwmCntRem > 0)
					{
						pwm1_temp = p->PwmCntRem;
						p->PwmCntRem = 0;
					}
					else 
					{
						pwm1_temp = 0;
						p->CurrentIndex = (START_POINTS << 1) - p->ReversId + 1;
						if(p->CurrentIndex >= TOTAL_POINTS)
						{
							MotorPulseDisable();
							p->RevChangeRept = 0;
							p->IsRunning = STOPPED;
							break;
						}
						SetMotorPulseCycle(p->table[p->CurrentIndex].Pwm_Cycle);
					}
				}
				else 
				{
					p->CurrentIndex ++;
					if(p->CurrentIndex == p->ReversId && pwm1_temp == 0)p->CurrentIndex = (START_POINTS << 1) - p->ReversId + 1;//匀速点脉冲是0跳过匀速点
					if(p->CurrentIndex >= TOTAL_POINTS)
					{
						MotorPulseDisable();
						p->RevChangeRept = 0;
						p->IsRunning = STOPPED;
						break;
					}
					SetMotorPulseCycle(p->table[p->CurrentIndex].Pwm_Cycle);
				}
			}break;
			case 1://加速阶段，同向Change
			{
				p->RevChangeCmd = 0;
//				p->PwmCntMul += (steps + pwm1_temp) / CNTPWM_SIZE_MAX;
//				p->PwmCntRem += (steps + pwm1_temp) % CNTPWM_SIZE_MAX;
				p->PwmCntMul += (steps + pwm1_temp) >> 10;
				p->PwmCntRem += (steps + pwm1_temp) & 0x3ff;
				if(p->PwmCntRem >= CNTPWM_SIZE_MAX)
				{
					p->PwmCntMul ++;
					p->PwmCntRem %=  CNTPWM_SIZE_MAX;
				}
				if(p->PwmCntMul > 0)
				{
					pwm1_temp = CNTPWM_SIZE_MAX;
					p->PwmCntMul --;
				}
				else if(p->PwmCntRem > 0)
				{
					pwm1_temp = p->PwmCntRem;
					p->PwmCntRem = 0;
				}
				else 
				{
					pwm1_temp = 0;
				}
				p->CurrentIndex ++;
				if(p->CurrentIndex == p->ReversId && pwm1_temp == 0)p->CurrentIndex = (START_POINTS << 1) - p->ReversId + 1;//匀速点脉冲是0跳过匀速点
				SetMotorPulseCycle(p->table[p->CurrentIndex].Pwm_Cycle);
			}break;
			case 4://匀速阶段,同向change
			{
				p->RevChangeCmd = 0;
//				p->PwmCntMul += steps / CNTPWM_SIZE_MAX;
//				p->PwmCntRem += steps % CNTPWM_SIZE_MAX;
				
				p->PwmCntMul += steps >> 10;
				p->PwmCntRem += steps & 0x3ff;
				if(p->PwmCntRem >= CNTPWM_SIZE_MAX)
				{
					p->PwmCntMul ++;
					p->PwmCntRem %=  CNTPWM_SIZE_MAX;
				}
				if(p->PwmCntMul > 0)
				{
					pwm1_temp = CNTPWM_SIZE_MAX;
					p->PwmCntMul--;
				}
				else if(p->PwmCntRem > 0)
				{
					pwm1_temp = p->PwmCntRem;
					p->PwmCntRem = 0;
				}
				else
				{
					pwm1_temp = 0;
					p->CurrentIndex = (START_POINTS << 1) - p->ReversId + 1;//匀速点脉冲是0跳过匀速点
					SetMotorPulseCycle(p->table[p->CurrentIndex].Pwm_Cycle);
				}
			}break;
			case 2://减速阶段,同向Change
			{
				p->RevChangeCmd = 3;
				p->RevChangeIndex = p->CurrentIndex;
				p->CurrentIndex = p->ReversId;
				pwm_cnt_change = steps;
//				p->PwmCntMul = pwm_cnt_change / CNTPWM_SIZE_MAX;
//				p->PwmCntRem = pwm_cnt_change % CNTPWM_SIZE_MAX;     
				p->PwmCntMul = pwm_cnt_change >> 10;
				p->PwmCntRem = pwm_cnt_change & 0x3ff;
				
				if(p->PwmCntRem >= CNTPWM_SIZE_MAX)
				{
					p->PwmCntMul ++;
					p->PwmCntRem %=  CNTPWM_SIZE_MAX;
				}
				if(p->PwmCntMul > 0)
				{
					pwm1_temp = CNTPWM_SIZE_MAX;
					p->PwmCntMul--;
				}
				else if(p->PwmCntRem > 0)
				{
					pwm1_temp = p->PwmCntRem;
					p->PwmCntRem = 0;
				}
			}break;
			case 3://减速阶段,同向Change
			{
				if(p->PwmCntMul > 0)
				{
					pwm1_temp = CNTPWM_SIZE_MAX;
					p->PwmCntMul--;
				}
				else if(p->PwmCntRem > 0)
				{
					pwm1_temp = p->PwmCntRem;
					p->PwmCntRem = 0;
				}
				else
				{
					pwm1_temp = 0;
					p->RevChangeCmd = 0;
					p->CurrentIndex  =  p->RevChangeIndex + 1;
					SetMotorPulseCycle(p->table[p->CurrentIndex].Pwm_Cycle);
				}
			}break;
			case 10://加速阶段,反向Change
			{
				int pwm_cntdev = 0;
				pwm_cnt_change = (p->PwmCntMul << 10) + p->PwmCntRem + pwm1_temp;//p->PwmCntMul * CNTPWM_SIZE_MAX + p->PwmCntRem + pwm1_temp;
				pwm_cntdev = pwm_cnt_change - steps;
				if(pwm_cntdev >= 0)
				{
					p->RevChangeCmd = 0;
//					p->PwmCntMul = pwm_cntdev / CNTPWM_SIZE_MAX;
//					p->PwmCntRem = pwm_cntdev % CNTPWM_SIZE_MAX;
					p->PwmCntMul = pwm_cntdev >> 10;
					p->PwmCntRem = pwm_cntdev & 0x3ff;

					if(p->PwmCntMul > 0)
					{
						pwm1_temp = CNTPWM_SIZE_MAX;
						p->PwmCntMul--;
					}
					else if(p->PwmCntRem > 0)
					{
						pwm1_temp = p->PwmCntRem;
						p->PwmCntRem = 0;
					}
					else
					{
						pwm1_temp = 0;
					}
					p->CurrentIndex ++;
					if(p->CurrentIndex == p->ReversId && pwm1_temp == 0)p->CurrentIndex  = (START_POINTS << 1) - p->ReversId + 1;
					SetMotorPulseCycle(p->table[p->CurrentIndex].Pwm_Cycle);
				}
				else 
				{
					p->RevChangeCmd = 11;
					p->PwmCntMul = 0;
					p->PwmCntRem = 0;
				  pwm1_temp = 0;
					p->CurrentIndex ++;
					if(p->CurrentIndex == p->ReversId && pwm1_temp == 0)p->CurrentIndex = (START_POINTS << 1) - p->ReversId + 1;
					SetMotorPulseCycle(p->table[p->CurrentIndex].Pwm_Cycle);
				}
			}break;
			
			case 11:
			{				
				p->CurrentIndex ++;
				if(p->CurrentIndex == p->ReversId && pwm1_temp == 0)p->CurrentIndex = (START_POINTS << 1) - p->ReversId + 1;//匀速点脉冲是0跳过匀速点
					
				if(p->CurrentIndex >= TOTAL_POINTS)
				{
					p->RevChangeCmd = 0;
					p->MovingStyle = PF_MOVING;
					p->DstPulse = steps - pwm_cnt_change;
					p->Dir = (p->Dir == PLUS ? REDUCE : PLUS);
					StartMotor(p,&pwm1_temp);
					break;
				}
				SetMotorPulseCycle(p->table[p->CurrentIndex].Pwm_Cycle);
			}break;
			case 12://匀速阶段,反向Change
			{
				int pwm_cntdev = 0;
				pwm_cnt_change = (p->PwmCntMul << 10) + p->PwmCntRem;//p->PwmCntMul* CNTPWM_SIZE_MAX + p->PwmCntRem;
				pwm_cntdev = pwm_cnt_change - steps;
				if(pwm_cntdev >= 0)
				{
					p->RevChangeCmd = 0;
//					p->PwmCntMul = pwm_cntdev / CNTPWM_SIZE_MAX;
//					p->PwmCntRem = pwm_cntdev % CNTPWM_SIZE_MAX;
					p->PwmCntMul = pwm_cntdev >> 10;
					p->PwmCntRem = pwm_cntdev & 0x3ff;
					if(p->PwmCntMul > 0)
					{
						pwm1_temp = CNTPWM_SIZE_MAX;
						p->PwmCntMul--;
					}
					else if(p->PwmCntRem > 0)
					{
						pwm1_temp = p->PwmCntRem;
						p->PwmCntRem = 0;
					}
					else
					{
						pwm1_temp = 0;
						p->CurrentIndex = (START_POINTS << 1) - p->ReversId + 1;
						SetMotorPulseCycle(p->table[p->CurrentIndex].Pwm_Cycle);
					}
				}
				else
				{
					p->RevChangeCmd = 11;
					p->PwmCntMul = 0;
					p->PwmCntRem = 0;
				  pwm1_temp = 0;
					p->CurrentIndex = (START_POINTS << 1) - p->ReversId + 1;
					SetMotorPulseCycle(p->table[p->CurrentIndex].Pwm_Cycle);
				}
			}break;
			case 13://减速阶段,反向change
			{
				p->CurrentIndex++;
				if(p->CurrentIndex >= TOTAL_POINTS)
				{
					p->MovingStyle = PF_MOVING;
					p->DstPulse = (uint32_t)abs(steps);
					p->Dir = (p->Dir == PLUS ? REDUCE : PLUS);
					StartMotor(p,&pwm1_temp);
					p->RevChangeCmd = 0;
					break;
				}
				SetMotorPulseCycle(p->table[p->CurrentIndex].Pwm_Cycle);
			}break;
			default:break;
		}
	}
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    MotorEvent(&motor1,&pwm1_periodtemp,&pwm1_temp);
		TIM1->SR = (u16)~TIM_FLAG_Update;  
}

int StartMotor(Motion_CtrlTypeDef *p ,uint32_t *pwm_temp)
{
	if(p->DstPulse <= 0){return 0;}
	
	switch(p->MovingStyle)
	{
		case CNT_PF_MOVING:
		{
			StartCntPfMove(p,pwm_temp);
		}break;
		case CNT_MOVING:
		{
			p->PwmCntMul = 0;
			p->PwmCntRem = 0;
			p->CurrentIndex = 0;
			pwm1_periodtemp = 0;
			pwm1_temp = 0;
			SetMotorDir(p->Dir);
			p->ReversId = 0;
			TIM1->PSC = 167;		
			SetMotorPulseCycle(p->CntSpeed);
			MotorPulseEnable();
			p->IsRunning = RUNNING;
		}break;
		case PF_MOVING:
		{
			StartPfMove(p,pwm_temp);
		}break;
	}
	return 0;
}

int StartPfMove(Motion_CtrlTypeDef *p ,uint32_t *pwm_temp)
{
	int i = 0;
	double halfpos = 0;
	int mirrorIndex = 0;
	p->PwmCntMul = 0;
	p->PwmCntRem = 0;
	p->CurrentIndex = 0;
	pwm1_periodtemp = 0;
	pwm1_temp = 0;
	SetMotorDir(p->Dir);
	p->ReversId = 0;
	TIM1->PSC = 0;
	
	if(p->DstPulse < MinSteps_S[p->pf])
	{ 
		if(p->DstPulse <= RefDistance_S[p->pf][1] + (MinSteps_S[p->pf] - RefDistance_S[p->pf][TOTAL_POINTS - 3]))
		{
			p->MovingStyle = CNT_MOVING;
			SetMotorPulseCycle(p->table[0].Pwm_Cycle);
			MotorPulseEnable();
			p->IsRunning = RUNNING;
			return 0;
		}
		
		halfpos = (double)p->DstPulse / 2.0;
		for(i = START_POINTS;i > 0; i-- )
		{
			 if(RefDistance_S[p->pf][i] < halfpos)break;
		}
		p->ReversId = i;
		mirrorIndex = START_POINTS + START_POINTS - (p->ReversId - 1);
		p->speed = speed_table_S + p->pf;
		p->table = (Motion_TableTypeDef*)motion_table_S[p->pf];
		pwm_cnt = p->DstPulse - RefDistance_S[p->pf][p->ReversId - 1] - (MinSteps_S[p->pf] - RefDistance_S[p->pf][mirrorIndex - 1]);
		
//		p->PwmCntMul = pwm_cnt / CNTPWM_SIZE_MAX;
//		p->PwmCntRem = pwm_cnt % CNTPWM_SIZE_MAX;
		p->PwmCntMul = pwm_cnt >> 10;
		p->PwmCntRem = pwm_cnt & 0x3ff;
		if(p->PwmCntMul > 0)
		{
			*pwm_temp = CNTPWM_SIZE_MAX;
			p->PwmCntMul--;
		}
		else if(p->PwmCntRem > 0)
		{
			*pwm_temp = p->PwmCntRem;
			p->PwmCntRem = 0;
		}
		else 
		{
			*pwm_temp = 0;
		}
		SetMotorPulseCycle(p->table[0].Pwm_Cycle);
		MotorPulseEnable();
		p->IsRunning = RUNNING;
	}
	else 
	{
		p->ReversId = START_POINTS;
		p->speed = speed_table_S + p->pf;
		p->table = (Motion_TableTypeDef*)motion_table_S[p->pf];			
		p->MinSteps = MinSteps_S[p->pf];
		pwm_cnt = p->DstPulse - p->MinSteps + p->table[START_POINTS].Pwm_Counter;
//		p->PwmCntMul = pwm_cnt / CNTPWM_SIZE_MAX;
//		p->PwmCntRem = pwm_cnt % CNTPWM_SIZE_MAX;
		p->PwmCntMul = pwm_cnt >> 10;
		p->PwmCntRem = pwm_cnt & 0x3ff;
		if(p->PwmCntMul > 0)
		{
			*pwm_temp = CNTPWM_SIZE_MAX;
			p->PwmCntMul--;
		}
		else if(p->PwmCntRem > 0)
		{
			*pwm_temp = p->PwmCntRem;
			p->PwmCntRem = 0;
		}
		else 
		{
			*pwm_temp = 0;
		}
		SetMotorPulseCycle(p->table[0].Pwm_Cycle);
		MotorPulseEnable();
		p->IsRunning = RUNNING;
	}
	
	return 0;
	
}

int StartCntPfMove(Motion_CtrlTypeDef *p ,uint32_t *pwm_temp) 
{
	int i = 0;
	double halfpos = 0;
	int mirrorIndex = 0;
	p->PwmCntMul = 0;
	p->PwmCntRem = 0;
	p->CurrentIndex = 0;
	pwm1_periodtemp = 0;
	pwm1_temp = 0;
	SetMotorDir(p->Dir);
	p->ReversId = 0;
	TIM1->PSC = 0;

	Init_Motion_Table_T((Motion_TableTypeDef (*)[TOTAL_POINTS])motion_table_Cnt,p->speed,&p->MinSteps,&p->MinTimes);
	p->table = motion_table_Cnt;
	if(p->MinSteps > p->DstPulse)
	{
		if(p->DstPulse <= RefDistance_S[p->pf][1] + (MinSteps_S[p->pf] - RefDistance_S[p->pf][TOTAL_POINTS - 3]))
		{
			p->MovingStyle = CNT_MOVING;
			SetMotorPulseCycle(p->table[0].Pwm_Cycle);
			MotorPulseEnable();
			p->IsRunning = RUNNING;
			return 0;
		}
		
		halfpos = (double)p->DstPulse / 2.0;
		for(i = START_POINTS;i > 0; i-- )
		{
			 if(RefDistance_T[p->pf][i] < halfpos)break;
		}
		p->ReversId = i;
		mirrorIndex = START_POINTS + START_POINTS - (p->ReversId - 1);
		pwm_cnt = p->DstPulse - RefDistance_T[p->pf][p->ReversId - 1] - (MinSteps_S[p->pf] - RefDistance_T[p->pf][mirrorIndex - 1]);
		
//		p->PwmCntMul = pwm_cnt / CNTPWM_SIZE_MAX;
//		p->PwmCntRem = pwm_cnt % CNTPWM_SIZE_MAX;
		p->PwmCntMul = pwm_cnt >> 10;
		p->PwmCntRem = pwm_cnt & 0x3ff;
		if(p->PwmCntMul > 0)
		{
			*pwm_temp = CNTPWM_SIZE_MAX;
			p->PwmCntMul--;
		}
		else if(p->PwmCntRem > 0)
		{
			*pwm_temp = p->PwmCntRem;
			p->PwmCntRem = 0;
		}
		else 
		{
			*pwm_temp = 0;
		}
		SetMotorPulseCycle(p->table[0].Pwm_Cycle);
		MotorPulseEnable();
		p->IsRunning = RUNNING;
	}
	else 
	{
		p->ReversId = START_POINTS;
		pwm_cnt = p->DstPulse - p->MinSteps + p->table[START_POINTS].Pwm_Counter;
//		p->PwmCntMul = pwm_cnt / CNTPWM_SIZE_MAX;
//		p->PwmCntRem = pwm_cnt % CNTPWM_SIZE_MAX;
		p->PwmCntMul = pwm_cnt >> 10;
		p->PwmCntRem = pwm_cnt & 0x3ff;
		if(p->PwmCntMul > 0)
		{
				*pwm_temp = CNTPWM_SIZE_MAX;
				p->PwmCntMul--;
		}
		else if(p->PwmCntRem > 0)
		{
				*pwm_temp = p->PwmCntRem;
				p->PwmCntRem = 0;
		}
		else 
		{
			*pwm_temp = 0;
		}
		SetMotorPulseCycle(p->table[0].Pwm_Cycle);
		MotorPulseEnable();
		p->IsRunning = RUNNING;
	}
	return 0;
}



