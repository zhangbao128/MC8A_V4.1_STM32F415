#ifndef MOTIONTYPEDEF_H__
#define MOTIONTYPEDEF_H__

#include <stdint.h>
//#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32f4xx.h"
//几个阶段的时间百分比
#define AA_PERC							0.2		/*加加速，减加速阶段的时间百分比*/
#define CA_PERC							0.6		/*匀加速，匀减速阶段的时间百分比*/
#define RA_PERC							0.2		/*减加速，减减速阶段的时间百分比*/

//几个阶段的离散化点数			
#define AA_POINTS						45
#define CA_POINTS						60
#define RA_POINTS						30
#define AR_POINTS						30
#define CR_POINTS						60
#define RR_POINTS						45

#define TOTAL_POINTS					(AA_POINTS + CA_POINTS + RA_POINTS + AR_POINTS + CR_POINTS + RR_POINTS + 1)
#define START_POINTS					(AA_POINTS + CA_POINTS + RA_POINTS)
#define STOP_POINTS						(AR_POINTS + CR_POINTS + RR_POINTS)

#define SPEED_LEVEL						20
#define PSC_CLK							168000000 //主定时器以14M的平率计数
#define CNTPWM_SIZE_MAX 				1024

#define SLOW_CNT_CLK 			1000000

//为了确保发出的脉冲精度，初始化速度与加速时间有一定的关系
//速度初始化时的初始速度与加速时间的关系
//2 * START_POINTS / acct < init < 3 * START_POINTS / acct ==>
//2 * START_POINTS < init * acct < 3 * START_POINTS ==>
//
#define S_MODEL                         1
//#define T_MODEL                         1
#define CNT_T_MODEL                     1
#define MAX_SPEED                       500000
#define ACC_TIME                        0.1
#define CNT_ACC_TIME                    0.1
#define ACC_SPEED 											8000000

//抢占时优先级2位(0--3)，从优先级2位(0--3)
#define PriorityGroup 									NVIC_PriorityGroup_0

#define WWdgPreemptionPriority 					0
#define WWdgSubPriority 								3

#define Motor1PreemptionPriority 				0
#define Motor1SubPriority 							0

#define Encoder1PreemptionPriority 			0
#define Encoder1SubPriority 						1

#define CANRxPreemptionPriority 				0
#define CANRxSubPriority 								2

/*error*/
typedef enum{
	NORMAL 													= 0,
	PF_SETPU_ERROR 									= 10,
	AXIS_OVERANGE 									= 11,
	MOTOR_IS_DISABLE 								= 12,
	PULS_LESSTHAN_2 								= 13,
	POINTER_2_NULL 									= 14,
	MOTOR_ISRUNNING 								= 15,
	CMD_ERROR 											= 16,
	SPEED_SETUP_ERROR 							= 17,
	MOTION_TABLE_ERROR 							= 18,
	MOTOR_IS_IN_CHANGE_MOVING 			= 19,
	MOTOR_IS_IN_CNT_MOVING 					= 20,
	BEYOND_THE_LIMIT 								= 21,
	LIMIT_SENSOR_INDUCTED 					= 22,
	ENCODER_WAS_DISABLE 						= 23,
	SERVO_ALARM 										= 24
} ErrorCode;			
			
typedef enum{			
	CNT_MOVING 											= 1,
	CNT_PF_MOVING 									= 2,
	PF_MOVING 											= 3,
	CHANGE_MOVING 									= 4,
	STOP_MOVING											= 5,
}MovingStyle;

typedef enum {RUNNING = 1,STOPPED = 0} MtrStatus;
typedef enum {PLUS = 0,REDUCE = 1} MtrDir;
typedef FunctionalState MtrEnable;
typedef enum {OPEN = 0,CLOSE = 1} MtrPulse;
//typedef enum {ON = 0,OFF = 1} SensorSta;
//typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
/*pwm某离散时间段内的周期*/
typedef struct{
	uint16_t Pwm_Cycle;
	uint16_t Pwm_Counter;
} Motion_TableTypeDef;

typedef struct{
	uint32_t IniSp;/*初始速度*/
	uint32_t StoSp;/*停止速度*/
	uint32_t MaxSp;/*最大速度*/
	uint32_t pf;
	
	/*加速度的范围比较大可以使用_iq2的格式*/
	double AASp;/*加加速度*/
	double DASp;/*减加速度*/
	/*加速时间的范围0--1可以使用_iq30的格式*/
	double AAT;
	double CAT;
	double RAT;
	double ART;
	double CRT;
	double RRT;
} Speed_LevelTypeDef;

typedef struct{
	uint32_t DstPulse;						/*当前需要产生的pwm脉冲数*/
//	uint32_t OutPulse;						/*已经输出的pwm脉冲数*/
	int32_t DstPos;
	int32_t CurrentPos;					    /*当前位置*/
//	int32_t CurrentEncPos;
	uint32_t MinSteps;						/*完成某次S型曲线运动，需要的最少pwm脉冲数*/
	uint16_t CurrentIndex;		            /*当前执行到的table索引*/
	uint16_t StopIndex;						/*接收到停止指令时的执行到的table索引*/
	uint32_t PwmCntMul;						/*匀速阶段的pwm倍数*/
	uint16_t PwmCntRem;						/*匀速阶段的pwm余数*/
	double MinTimes;						/*完成某次S型曲线运动，需要的最少时间*/
	double Time_Cost_Act;					/*完成某次S型曲线运动，实际消耗的最少时间*/
	Speed_LevelTypeDef *speed;
	Motion_TableTypeDef *table;
	MtrDir Dir;								/*电机运动方向*/
	MtrStatus IsRunning;					/*电机运动状态*/
//	uint16_t IsCntMoving;					/*当前是否匀速运行标示*/
//	uint16_t MotorId;						/*电机轴Id 1...*/
	uint16_t RevStopcmd;                    /*1：接收到停止指令，0：没有接收到停止指令*/
	uint16_t Enable;						/*电机使能*/
	uint16_t pf;							/*电机当前运动速度等级 0...*/
//	uint16_t CntFre;
	uint16_t CntSpeed;
	uint16_t MovingStyle;			/*cntmove,pfmove,stop,changemove*/


	int32_t pLimit;						/*正向软限位*/
	int32_t nLimit;						/*反向软限位*/	
//	uint32_t ChangeIndex ;
//	uint32_t ChangeFlag ;     /*1.加速阶段收到change指令，且未进入匀速阶段
//															2.减速阶段收到change指令*/
	uint8_t RevChangeRept;
	uint8_t RevChangeCmd;
	uint16_t RevChangeIndex;
	uint32_t RemainPulse;
	
	int32_t TriggerPos;
	MtrDir TriggerDir;
	uint32_t TriggerPeriod;			/*相机触发电平持续时间 ms*/
	uint8_t TriggerEnable;
	
	int32_t LightPreStep;			/*光源提前打开Step 大于0提前，小于0滞后*/
	uint32_t LightPeriod;				/*光源出发电平持续时间 ms*/
	uint8_t LightEnable;
	
	uint8_t LimitFlag;
	
	uint8_t SearchEnable;
	uint8_t SearchStatus;
	uint8_t EncoderEnable;
	
	int ReversId;
	
//	uint8_t SearchSpeed;
//	MtrDir SearchDir;
	
} Motion_CtrlTypeDef;


#endif




