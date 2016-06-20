#include "motionCtrl_T.h"

extern Motion_TableTypeDef motion_table_S[SPEED_LEVEL][TOTAL_POINTS];/*S型曲线运行参数*/
extern Speed_LevelTypeDef speed_table_S[SPEED_LEVEL];/*S型曲线速度参数*/
extern uint32_t MinSteps_S[SPEED_LEVEL];/**/
extern double MinTimes_S[SPEED_LEVEL];
uint32_t RefDistance_T[SPEED_LEVEL][TOTAL_POINTS];

int Init_Default_T(void){
	int i =0;
	if(Init_Speed_Table_T()){return -1;}
	for(i = 0;i < SPEED_LEVEL;i ++){
        if(Init_Motion_Table_T(
        (Motion_TableTypeDef (*)[TOTAL_POINTS])motion_table_S[i],
        &speed_table_S[i],
        &MinSteps_S[i],
        &MinTimes_S[i])){return -1;}
	}
	return 0;
}
int Init_Speed_Table_T(void){
	uint16_t i = 0;
	for(i =0;i < SPEED_LEVEL;i ++){
		if(Speed_Setup_T(&speed_table_S[i],i,(uint32_t)(MAX_SPEED / SPEED_LEVEL) * (i + 1),ACC_TIME,ACC_TIME) != 0){return -1;}
	}
	return 0;
}
int Speed_Setup_T(Speed_LevelTypeDef *speed,uint16_t pf,uint32_t maxsp,double acct,double dect){
	uint32_t initsp,stopsp;
//	if((0.05 > acct) || (acct > 2.5) || (0.05 > dect) || (dect > 2.5)){
//		return -1;
//	}
	initsp = (2.5 * START_POINTS / acct);
	stopsp = (2.5 * START_POINTS / dect);
	if(maxsp < initsp){
		return -1;
	}
	speed->pf = pf;
	speed->AASp = (maxsp - initsp) / acct;
	speed->DASp = (maxsp - stopsp) / dect;
	speed->IniSp = initsp;
	speed->StoSp = stopsp;
	speed->MaxSp = maxsp;
	speed->AAT = acct;
	speed->CAT = 0;
	speed->RAT = 0;

	speed->ART = dect;
	speed->CRT = 0;
	speed->RRT = 0;
	return 0;
}
int Init_Motion_Table_T(Motion_TableTypeDef (*table)[TOTAL_POINTS],const Speed_LevelTypeDef *pSpeed,uint32_t *sumSteps,double *sumTimes){
	int i = 0,pwm_cycle = 0,pwm_count = 0;//,k = 0;
	double fi = 0;
	Motion_TableTypeDef *p = (Motion_TableTypeDef *)table;
	*sumSteps = 0;
	*sumTimes = 0;
	for(i = 0;i < START_POINTS;i ++){
		fi = GetSpeed_ByTicks_T(pSpeed,(pSpeed->AAT / START_POINTS) * i);
		pwm_cycle = PSC_CLK / fi;
		pwm_count = (pSpeed->AAT / START_POINTS) * fi;
		p[i].Pwm_Counter = pwm_count;
		p[i].Pwm_Cycle = pwm_cycle;
		*sumSteps +=	pwm_count;
		*sumTimes +=	pwm_count / fi;
		RefDistance_T[pSpeed->pf][i] = *sumSteps;
	}
	
	  fi = GetSpeed_ByTicks_T(pSpeed,pSpeed->AAT);
		pwm_cycle = PSC_CLK / fi;
		pwm_count = (pSpeed->AAT / START_POINTS) * fi;
		p[START_POINTS].Pwm_Counter = pwm_count;
		p[START_POINTS].Pwm_Cycle = pwm_cycle;
		*sumSteps +=	pwm_count;
		*sumTimes +=	pwm_count / fi;
		RefDistance_T[pSpeed->pf][START_POINTS] = *sumSteps;
	
  for(i = 0;i < STOP_POINTS;i ++){
		fi = GetSpeed_ByTicks_T(pSpeed,pSpeed->AAT + (pSpeed->ART / STOP_POINTS) * i);
		pwm_cycle = PSC_CLK / fi;
		pwm_count = (pSpeed->ART / STOP_POINTS) * fi;
		p[START_POINTS + 1 + i].Pwm_Counter = pwm_count;
		p[START_POINTS + 1 + i].Pwm_Cycle = pwm_cycle;
		*sumSteps +=	pwm_count;
		*sumTimes +=	pwm_count / fi;
		RefDistance_T[pSpeed->pf][START_POINTS + 1 + i] = *sumSteps;
	}
	return 0;
}
/*梯形曲线时间与速度的对应关系*/
double GetSpeed_ByTicks_T(const Speed_LevelTypeDef *speed,double t){
    double fi = 0;
	if(0 <= t && t < speed->AAT){
			fi = speed->IniSp + speed->AASp * t;
	}
	else if(speed->AAT <= t && t < speed->AAT + speed->ART){
			fi = speed->IniSp + speed->AASp * speed->AAT - speed->DASp * (t - speed->AAT);
	}
	return fi;
}




