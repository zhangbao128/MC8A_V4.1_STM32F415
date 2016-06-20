#ifndef USR_INC_MOTOR_H_
#define USR_INC_MOTOR_H_
#include "motionTypeDef.h"

#define MotorPulseEnable()		\
		TIM1->CR1 |= 1
#define MotorPulseDisable()		\
		TIM1->CR1 &= ~1

#define LED_ON()							\
		GPIO_ResetBits(GPIOC,GPIO_Pin_11)
#define LED_OFF()							\
		GPIO_SetBits(GPIOC,GPIO_Pin_11)
		
#define SetMotorPulseCycle(cycle)	\
		TIM1->CNT = 0;							\
		TIM1->ARR = cycle;					\
		TIM1->CCR1 = cycle >> 1
		
#define SetMotorDir(dir)				\
		motor1.Dir = dir ;					\
		dir == REDUCE ? GPIO_SetBits(GPIOA,GPIO_Pin_4) : GPIO_ResetBits(GPIOA,GPIO_Pin_4)
		
#define MotorEnable(status)			\
		motor1.Enable = status;			\
		status == ENABLE ? GPIO_SetBits(GPIOA,GPIO_Pin_5):GPIO_ResetBits(GPIOA,GPIO_Pin_5)
		
#define SetCameraTriggerPin()		\
		GPIOA->ODR |= 1 << 6

#define ClearCameraTriggerPin()	\
		GPIOA->ODR &= ~(1 << 6)

#define SetLightTriggerPin()		\
		GPIOA->ODR |= 1 << 7

#define ClearLightTriggerPin()	\
		GPIOA->ODR &= ~(1 << 7)

#define ClearLimitSensor()			\
		motor1.LimitFlag = 0


void Motor_Data_Init(void);
void InitMotorGpio(void);
void InitMotorTimer(void);
int32_t GetMotor_Pos(void);
uint16_t GetMotor_Status(void);
int StartMotor(Motion_CtrlTypeDef *pmotor,uint32_t *pwm_temp);



#endif 
