#include "cmd.h"
#include "motionTypeDef.h"
#include "can.h"
#include "motor.h"

#include "motionCtrl_S.h"
#include "motionCtrl_T.h"
#include "Verify.h"
#include "encoder.h"

extern CanTxMsg TxMsg;
extern CanRxMsg RxMsg;
extern Motion_CtrlTypeDef motor1;


extern Motion_TableTypeDef motion_table_S[SPEED_LEVEL][TOTAL_POINTS];/*S型曲线运行参数*/
extern Speed_LevelTypeDef speed_table_S[SPEED_LEVEL];/*S型曲线速度参数*/
extern uint32_t RefDistance[SPEED_LEVEL][TOTAL_POINTS];
extern uint32_t MinSteps_S[SPEED_LEVEL];/**/
extern double MinTimes_S[SPEED_LEVEL];
extern unsigned char Recv_Finished;


/*匀速运动数据*/
extern Motion_TableTypeDef motion_table_Cnt[TOTAL_POINTS];
extern Speed_LevelTypeDef speed_Cnt;
extern uint32_t pwm1_temp;
//extern uint32_t pwm2_temp;
extern uint16_t CardId;
extern uint16_t AxisId;

double temp_accspeed = 0;
uint32_t temp_maxsp = 0;
double temp_acctimes = 0.0;

int transmitMailid = 0;
uint16_t cmd = 0xff;
uint16_t pf = 0xff;
uint16_t errorCode = NORMAL;
uint32_t pos = 0;
uint16_t verifyCode = 0;

//int devPos = 0;
//unsigned int *pwm_tmep = NULL;
unsigned int cnt_Speed = 0;
int steps = 0;
short rtn = 0;
Motion_CtrlTypeDef *pmotor = NULL;


void Deal_Cmd(void)
{
    pf = 0xff;
    errorCode = NORMAL;
    pos = 0;
    verifyCode = 0;
		cnt_Speed = 0;
    cmd = (RxMsg.ExtId >> 8) & 0xff;
    switch(cmd){
        case CMD_PF_MOVE:
				{
					pmotor = &motor1;
					
					if(pmotor == NULL)								{errorCode = POINTER_2_NULL;break;}
					if(pmotor->Enable == DISABLE)			{errorCode = MOTOR_IS_DISABLE;break;}
					if(pmotor->LimitFlag)							{errorCode = LIMIT_SENSOR_INDUCTED;break;}
					if(pmotor->IsRunning == RUNNING)	{errorCode = MOTOR_ISRUNNING;break;}
					if((GPIOC->IDR & 0x10) == 0)			{errorCode = SERVO_ALARM;break;}
						
					pf = RxMsg.Data[1];
					pos = RxMsg.Data[7] * 16777216;
					pos += RxMsg.Data[6] * 65536;
					pos += RxMsg.Data[5] * 256;
					pos += RxMsg.Data[4];
					
					pmotor->DstPos = (int32_t)pos;
					if(pmotor->CurrentPos != GetEncoderPos() && pmotor->EncoderEnable){pmotor->CurrentPos = GetEncoderPos();}	
					steps = pmotor->DstPos - pmotor->CurrentPos;
					pmotor->pf = pf;
					pmotor->MovingStyle = PF_MOVING;
					pmotor->DstPulse = (uint32_t)abs(steps);
					if(steps >= 0){pmotor->Dir = PLUS;}
					else {pmotor->Dir = REDUCE;}

					rtn = StartMotor(pmotor,&pwm1_temp);
					if(rtn){errorCode = rtn;}
        }break;
				case CMD_CHANGE_PF_MOVE:
				{
					pmotor = &motor1;
					
					if(pmotor == NULL){errorCode = POINTER_2_NULL;break;}
					if(pmotor->Enable == DISABLE){errorCode = MOTOR_IS_DISABLE;break;}
					if(pmotor->LimitFlag){errorCode = LIMIT_SENSOR_INDUCTED;break;}
					if(pmotor->IsRunning == RUNNING && pmotor->RevChangeRept){errorCode = MOTOR_IS_IN_CHANGE_MOVING ;break;}
					if((GPIOC->IDR & 0x10) == 0)			{errorCode = SERVO_ALARM;break;}
					
					pf = RxMsg.Data[1];
					pos = RxMsg.Data[7] * 16777216;
					pos += RxMsg.Data[6] * 65536;
					pos += RxMsg.Data[5] * 256;
					pos += RxMsg.Data[4];
					steps = (int32_t)pos - pmotor->DstPos;
					if(steps == 0){break;}
					pmotor->DstPos = (int32_t)pos;
//					if(pmotor->DstPos > pmotor->pLimit || pmotor->DstPos < pmotor->nLimit){errorCode = BEYOND_THE_LIMIT;break;}
					pmotor->DstPulse += (uint32_t)abs(steps);
					
					pmotor->RevChangeRept = 1;
					
					if(pmotor->IsRunning != RUNNING)
					{
						if(steps >= 0){pmotor->Dir = PLUS;}
						else {pmotor->Dir = REDUCE;}
						pmotor->pf = pf;
						pmotor->MovingStyle = PF_MOVING;
						pmotor->DstPulse = (uint32_t)abs(steps);
						rtn = StartMotor(pmotor,&pwm1_temp);
						if(rtn){errorCode = rtn;}
						TxMsg.Data[1] = 0;
						break;
					}
					
					//向位移增加方向change
					if((pmotor->Dir == PLUS && steps > 0) || 
						 (pmotor->Dir == REDUCE && steps < 0))
					{
						steps = abs(steps);
						//加速阶段,收到Change指令
						if(pmotor->CurrentIndex < pmotor->ReversId)
						{
							pmotor->RevChangeCmd = 1;
						}
						//匀速阶段,接收到change指令
						else if(pmotor->CurrentIndex == pmotor->ReversId)
						{
							pmotor->RevChangeCmd = 4;
						}
						//减速阶段,收到Change指令
						else 
						{
							pmotor->RevChangeCmd = 2;
						}
					}
					//向位移减少方向change
					else
					{
						steps = abs(steps);
						if(pmotor->CurrentIndex < pmotor->ReversId)
						{
							pmotor->RevChangeCmd = 10;
						}
						else if(pmotor->CurrentIndex == pmotor->ReversId)
						{
							pmotor->RevChangeCmd = 12;
						}
						else 
						{
							pmotor->RevChangeCmd = 13;
						}
					}
					TxMsg.Data[1] = pmotor->RevChangeCmd;
				}break;
        case CMD_INITIALIZE_CARD:
				{
					pmotor = &motor1;
					Motor_Data_Init();
					InitMotorTimer();
					MotorPulseDisable();
					if(pmotor->EncoderEnable)
					{
						InitEncoderGpio();
						InitEncoderTimer();
					}
					
#if S_MODEL
            Init_Default_S();
#else
            Init_Default_T();
#endif
        }break;
        case CMD_GET_POS:
				{
					pos = GetMotor_Pos();
					TxMsg.Data[1] = pos & 0xff;
					TxMsg.Data[2] = (pos >> 8) & 0xff;
					TxMsg.Data[3] = (pos >> 16) & 0xff;
					TxMsg.Data[4] = (pos >> 24) & 0xff;
        }break;
        case CMD_SET_POS:
				{
					pmotor = &motor1;
					if(pmotor->IsRunning != STOPPED){errorCode = MOTOR_ISRUNNING;break;}
					pos = RxMsg.Data[4] * 16777216;
					pos += RxMsg.Data[3] * 65536;
					pos += RxMsg.Data[2] * 256;
					pos += RxMsg.Data[1];
					pmotor->CurrentPos = pos;
					pmotor->DstPos = pos;
        }break;
				case CMD_SET_ENCPOS:
				{
					pmotor = &motor1;
					if(pmotor->IsRunning != STOPPED){errorCode = MOTOR_ISRUNNING;break;}
					if(!pmotor->EncoderEnable){errorCode = ENCODER_WAS_DISABLE;break;}
					pos = RxMsg.Data[4] * 16777216;
					pos += RxMsg.Data[3] * 65536;
					pos += RxMsg.Data[2] * 256;
					pos += RxMsg.Data[1];
					SetEncoderPos((int32_t)pos);
				}break;
				case CMD_GET_ENCPOS:
				{
					pos = 0;
					if(pmotor->EncoderEnable){pos = GetEncoderPos();}
					TxMsg.Data[1] = pos & 0xff;
					TxMsg.Data[2] = (pos >> 8) & 0xff;
					TxMsg.Data[3] = (pos >> 16) & 0xff;
					TxMsg.Data[4] = (pos >> 24) & 0xff;
				}break;
        case CMD_PF_SET_STEP1:
				{
//					temp_accspeed = RxMsg.Data[4];
//					temp_accspeed += RxMsg.Data[5] << 8;
//					temp_accspeed += RxMsg.Data[6] << 16;
//					temp_accspeed += RxMsg.Data[7] << 24;
					temp_acctimes = RxMsg.Data[2];
					temp_acctimes += RxMsg.Data[3] << 8;
					temp_acctimes /= 1000.0;
        }break;
        case CMD_PF_SET_STEP2:
				{
					pf = RxMsg.Data[1];
					temp_maxsp = RxMsg.Data[2];
					temp_maxsp += RxMsg.Data[3] << 8;
					temp_maxsp += RxMsg.Data[4] << 16;
					temp_maxsp += RxMsg.Data[5] << 24;
//#if S_MODEL
					if(Speed_Setup_S(&speed_table_S[pf],pf,temp_maxsp,temp_acctimes))
					{
//#else
//					if(Speed_Setup_T(&speed_table_S[pf],pf,temp_maxsp,temp_acctimes,temp_acctimes))
//					{
//#endif
							errorCode = PF_SETPU_ERROR;break;
					}
					else 
					{
//#if S_MODEL
							if(Init_Motion_Table_S((Motion_TableTypeDef (*)[TOTAL_POINTS])motion_table_S[pf],&speed_table_S[pf],&MinSteps_S[pf],&MinTimes_S[pf]))
							{
//#else
//							if(Init_Motion_Table_T((Motion_TableTypeDef (*)[TOTAL_POINTS])motion_table_S[pf],&speed_table_S[pf],&MinSteps_S[pf],&MinTimes_S[pf]))
//							{
//#endif
									errorCode = MOTION_TABLE_ERROR;break;
							}
					}
        }break;
        case CMD_GET_MOTOR_STATUS:
				{
            TxMsg.Data[1] = GetMotor_Status();
        }break;
        case CMD_ENABLE:
				{
                MotorEnable(ENABLE);
        }break;
        case CMD_DISENABLE:
				{
								MotorEnable(DISABLE);
        }break;
        case CMD_PF_STOP:
				{
					pmotor = &motor1;
					if(pmotor->RevStopcmd != 1 && pmotor->IsRunning != STOPPED &&
						 pmotor->CurrentIndex <= pmotor->ReversId)//如果已经运行到减速阶段，忽略停止指令
					{
							pmotor->RevStopcmd = 1;
							pmotor->StopIndex = pmotor->CurrentIndex;
					}
        }break;
				//匀速运动时发送的运动距离必须大于某值
        case CMD_CNT_MOVE:
				{
					pmotor = &motor1;
					if(pmotor == NULL)								{errorCode = POINTER_2_NULL;break;}
					if(pmotor->Enable == DISABLE)			{errorCode = MOTOR_IS_DISABLE;break;}
					if(pmotor->LimitFlag)							{errorCode = LIMIT_SENSOR_INDUCTED;break;}
					if(pmotor->IsRunning == RUNNING)	{errorCode = MOTOR_ISRUNNING;break;}
					if((GPIOC->IDR & 0x10) == 0)			{errorCode = SERVO_ALARM;break;}
					
					cnt_Speed = RxMsg.Data[1];
					cnt_Speed += RxMsg.Data[2] * 256;
					cnt_Speed += RxMsg.Data[3] * 65536;

					pos =  RxMsg.Data[7] * 16777216;
					pos += RxMsg.Data[6] * 65536;
					pos += RxMsg.Data[5] * 256;
					pos += RxMsg.Data[4];
					pmotor->DstPos = (int32_t)pos;
					if(pmotor->CurrentPos != GetEncoderPos() && pmotor->EncoderEnable){pmotor->CurrentPos = GetEncoderPos();}
					steps = pmotor->DstPos - pmotor->CurrentPos;
					if(steps >= 0){pmotor->Dir = PLUS;}
					else {pmotor->Dir = REDUCE;}
					pmotor->pf = 0;
					pmotor->MovingStyle = 0;
					pmotor->DstPulse = (uint32_t)fabs(steps);
					//可以曲线加速的速度设定
					if(!Speed_Setup_T(&speed_Cnt,0,cnt_Speed,CNT_ACC_TIME,CNT_ACC_TIME))
					{
							pmotor->MovingStyle = CNT_PF_MOVING;//设置为非恒定速度运行标识
							pmotor->speed = &speed_Cnt;
							rtn = StartMotor(pmotor,&pwm1_temp);
							if(rtn){errorCode = rtn;}
					}
					else
					{
							//不能跑去曲线的速度设定								
							uint32_t cnt_speed = SLOW_CNT_CLK / cnt_Speed;							
							if(cnt_speed < 65535)
							{
									pmotor->MovingStyle = CNT_MOVING;
									pmotor->CntSpeed = SLOW_CNT_CLK / cnt_Speed;
									rtn = StartMotor(pmotor,&pwm1_temp);
									if(rtn){errorCode = rtn;}
							}
							else 
							{
									errorCode = CMD_ERROR;
							}
					}
        }break;
				case CMD_SET_TRIGGER_POS:
				{
					pmotor = &motor1;
					pos = RxMsg.Data[7] * 16777216;
					pos += RxMsg.Data[6] * 65536;
					pos += RxMsg.Data[5] * 256;
					pos += RxMsg.Data[4];
					pmotor->TriggerPos = (int32_t)pos;
					pmotor->TriggerDir = (RxMsg.Data[3] == PLUS) ?PLUS : REDUCE;
					
				}break;
				case CMD_SET_TRIGGER_PERIOD:
				{
					pmotor = &motor1;
					pmotor->TriggerPeriod = 0;
					
					pmotor->TriggerPeriod = RxMsg.Data[7] * 16777216;
					pmotor->TriggerPeriod += RxMsg.Data[6] * 65536;
					pmotor->TriggerPeriod += RxMsg.Data[5] * 256;
					pmotor->TriggerPeriod += RxMsg.Data[4];
				}break;
				case CMD_CLEAR_TRIGGER:
				{
					pmotor = &motor1;
					pmotor->TriggerDir = PLUS;
					pmotor->TriggerPos = 0;
					pmotor->TriggerPeriod = 0;
					pmotor->TriggerEnable = 0;

					pmotor->LightPreStep = 0;
					pmotor->LightPeriod = 0;
					pmotor->LightEnable = 0;

				}break;
				case CMD_SET_LIGHT_PERIOD:
				{
					pmotor = &motor1;
					pos = RxMsg.Data[3] * 16777216;
					pos += RxMsg.Data[2] * 65536;
					pos += RxMsg.Data[1] * 256;
					pos += RxMsg.Data[0];
					pmotor->LightPreStep = (int32_t)pos;

					pmotor->LightPeriod = 0;
					pmotor->LightPeriod = RxMsg.Data[7] * 16777216;
					pmotor->LightPeriod += RxMsg.Data[6] * 65536;
					pmotor->LightPeriod += RxMsg.Data[5] * 256;
					pmotor->LightPeriod += RxMsg.Data[4];
					
				}break;
				case CMD_SET_PLIMIT:
				{
					pmotor = &motor1;
					pos = RxMsg.Data[7] * 16777216;
					pos += RxMsg.Data[6] * 65536;
					pos += RxMsg.Data[5] * 256;
					pos += RxMsg.Data[4];
					pmotor->pLimit = (int32_t)pos;
					
				}break;
				case CMD_SET_NLIMIT:
				{
					pmotor = &motor1;
					pos = RxMsg.Data[7] * 16777216;
					pos += RxMsg.Data[6] * 65536;
					pos += RxMsg.Data[5] * 256;
					pos += RxMsg.Data[4];
					pmotor->nLimit = (int32_t)pos;
				}break;
				case CMD_CLEAR_LIMIT:
				{
					ClearLimitSensor();
				}break;
				case CMD_SEARCH_SENSOR:
				{
					pmotor = &motor1;
					if(pmotor == NULL)								{errorCode = POINTER_2_NULL;break;}
					if(pmotor->Enable == DISABLE)			{errorCode = MOTOR_IS_DISABLE;break;}
					if(pmotor->LimitFlag)							{errorCode = LIMIT_SENSOR_INDUCTED;break;}
					if(pmotor->IsRunning == RUNNING)	{errorCode = MOTOR_ISRUNNING;break;}
					if((GPIOC->IDR & 0x10) == 0)			{errorCode = SERVO_ALARM;break;}
					
					pmotor->SearchStatus = RxMsg.Data[0];
					
					cnt_Speed = RxMsg.Data[3] << 16;
					cnt_Speed += RxMsg.Data[2] << 8;
					cnt_Speed += RxMsg.Data[1];			
					
					pos =  RxMsg.Data[7] << 24;
					pos += RxMsg.Data[6] << 16;
					pos += RxMsg.Data[5] << 8;
					pos += RxMsg.Data[4];
					
					pmotor->SearchEnable = 1;
					
					pmotor->DstPos = pmotor->CurrentPos + (int32_t)pos;
					
					if(pmotor->CurrentPos != GetEncoderPos() && pmotor->EncoderEnable){pmotor->CurrentPos = GetEncoderPos();}
					steps = pmotor->DstPos - pmotor->CurrentPos;
					if(steps >= 0){pmotor->Dir = PLUS;}
					else {pmotor->Dir = REDUCE;}
					pmotor->pf = 0;
					pmotor->MovingStyle = 0;
					pmotor->DstPulse = (uint32_t)fabs(steps);
					//可以曲线加速的速度设定
					if(!Speed_Setup_T(&speed_Cnt,0,cnt_Speed,CNT_ACC_TIME,CNT_ACC_TIME))
					{
							pmotor->MovingStyle = CNT_PF_MOVING;//设置为非恒定速度运行标识
							pmotor->speed = &speed_Cnt;
							rtn = StartMotor(pmotor,&pwm1_temp);
							if(rtn){errorCode = rtn;}
					}
					else
					{
							//不能跑去曲线的速度设定								
							uint32_t cnt_speed = SLOW_CNT_CLK / cnt_Speed;							
							if(cnt_speed < 65535)
							{
									pmotor->MovingStyle = CNT_MOVING;
									pmotor->CntSpeed = SLOW_CNT_CLK / cnt_Speed;
									rtn = StartMotor(pmotor,&pwm1_temp);
									if(rtn){errorCode = rtn;}
							}
							else 
							{
									errorCode = CMD_ERROR;
							}
					}
				}break;
				case CMD_ENABLE_ENCODER:
				{
					pmotor = &motor1;
					pmotor->EncoderEnable = 1;
				}break;
				case CMD_DISABLE_ENCODER:
				{
					pmotor = &motor1;
					pmotor->EncoderEnable = 0;
				}break;
        default: {errorCode = CMD_ERROR;}break;
    }
    TxMsg.IDE = CAN_ID_EXT;
    TxMsg.RTR = CAN_RTR_DATA;
    TxMsg.DLC = 8;
    TxMsg.Data[0] = errorCode;
    verifyCode = GetVerifyCode(CardId,AxisId,cmd,TxMsg.Data);
    TxMsg.ExtId = (uint32_t)((CardId * 16 + AxisId) * 65536 + cmd * 256 + (verifyCode & 0xff));
		transmitMailid = CAN_Transmit(CAN1,&TxMsg);
}
