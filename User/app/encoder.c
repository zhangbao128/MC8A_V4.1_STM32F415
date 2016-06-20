#include "encoder.h"

//Encoder_TypeDef encoder1;
int32_t EncMul = 0;
void InitEncoderGpio(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	//EncoderA
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;    
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);

	//EncoderB
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;    
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);
}

/*
TIMx_CCMR1:
	[CC1S = 01],TI1FP1 mapped on TI1
TIMx_CCMR2:
	[CC2S = 01], TI2FP2 mapped on TI2
TIMx_CCER	
	[CC1P = 0,CC1NP = 0,IC1F = 0], TI1FP1 noninverted,TI1FP1=TI1
TIMx_CCER
	[CC2P = 0,CC2NP = 0,IC2F = 0,]TI2FP2 noninverted,TI2FP2=TI2
TIMx_SMCR
	[SMS = 011,]both inputs are active on both rising and fallingedges
TIMx_CR1
	[CEN = 1]Counter is enabled
*/
void InitEncoderTimer(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	//使能时钟
	RCC->APB1ENR |= 1 << 3;
	
	TIM5->SMCR &= ~0x7;
	TIM5->SMCR |= 0x3;
	
	TIM5->CCMR1 &= ~0x3;
	TIM5->CCMR1 |= 0x1;
	
	TIM5->CCMR1 &= ~(0x3 << 8);
	TIM5->CCMR1 |= 0x1 << 8;
	
	TIM5->CCMR1 |= (12 << 4 | 12 << 12);
	TIM5->CCMR2 |= (12 << 4 | 12 << 12);
	//TIM5->CCER = 1 << 1;
	
	TIM5->CR1 |= 0x1;
	TIM5->ARR = 0xffff;
	TIM5->PSC = 0;
	TIM5->CNT = 0;
	TIM5->DIER = 0x1;
	
//	encoder1.CurrentEncPos = 0;
	EncMul = 0;
//	encoder1.EncRem = 0;

	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;									//更新事件
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =Encoder1PreemptionPriority;        //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =Encoder1SubPriority;              	//响应优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //允许中断
	NVIC_Init(&NVIC_InitStructure);
}

int GetEncoderPos(void)
{
	return (EncMul * 65536 + TIM5->CNT);
}
void SetEncoderPos(int32_t pos)
{
	TIM5->CNT = pos % 65536;
	EncMul = pos / 65536;
}

void TIM5_IRQHandler(void)
{
	if(TIM5->SR & 1)
	{
		TIM5->SR &= ~1;
		// 0 upcounter
		// 1 downcounter
		if(TIM5->CR1 & 0x10)
		{
			EncMul --;
		}
		else 
		{
			EncMul ++;
		}
	}
}

