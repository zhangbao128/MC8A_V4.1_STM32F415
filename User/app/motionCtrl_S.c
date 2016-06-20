#include "motionCtrl_S.h"


Motion_TableTypeDef motion_table_S[SPEED_LEVEL][TOTAL_POINTS];/*S型曲线运行参数 占用空间21600Byte*/
Speed_LevelTypeDef speed_table_S[SPEED_LEVEL];/*S型曲线速度参数 960Byte*/
uint32_t RefDistance_S[SPEED_LEVEL][TOTAL_POINTS];
uint32_t MinSteps_S[SPEED_LEVEL];/*80Byte*/
double MinTimes_S[SPEED_LEVEL];/*80Byte*/

int Init_Default_S(void){
	int i =0;
	if(Init_Speed_Table_S()){return -1;}
	for(i = 0;i < SPEED_LEVEL;i ++){
        if(Init_Motion_Table_S(
        (Motion_TableTypeDef (*)[TOTAL_POINTS])motion_table_S[i],
        &speed_table_S[i],
        &MinSteps_S[i],
        &MinTimes_S[i])){return -1;}
	}
	return 0;
}
int Init_Speed_Table_S(void){
	int i = 0;
	for(i = 0;i < SPEED_LEVEL;i ++){
		if(Speed_Setup_S(&speed_table_S[i],i,(uint32_t)(MAX_SPEED / SPEED_LEVEL) * (i + 1),ACC_TIME)){return -1;}
	}
	return 0;
}
int Speed_Setup_S(Speed_LevelTypeDef *speed,int pf,uint32_t maxsp,double acctime){
  double aa = 0,taa = 0,tca = 0,tra = 0;
	uint32_t initsp = 0;
	taa = AA_PERC * acctime;
	tca = CA_PERC * acctime;
	tra = RA_PERC * acctime;
	initsp = (2 * AA_POINTS) / taa;
	if(maxsp < initsp)
	{
		return -1;
	}
//	initsp = (2.5 * AA_POINTS) / taa;
	aa = (maxsp - initsp) / (taa * tca + taa * tra);
	speed->pf 			= pf;
	speed->IniSp 		= initsp;
	speed->StoSp 		= initsp;
	speed->MaxSp 		= maxsp;
	speed->AASp 			= aa;
	speed->DASp 			= aa;
	speed->AAT 			= taa;
	speed->CAT 			= tca;
	speed->RAT 			= tra;
	speed->ART 			= taa;
	speed->CRT 			= tca;
	speed->RRT 			= tra;
	
	////////////////////////////////////////////////////////////
	
	return 0;
}
/*S型曲线时间与速度的对应关系*/
double GetSpeed_ByTicks_S(const Speed_LevelTypeDef *pSpeed,double t)
{
    double sp = 0;

    if(0 <= t && t < pSpeed->AAT){
        sp = pSpeed->IniSp+0.5*pSpeed->AASp*t*t;
    }
    else if(pSpeed->AAT <= t && t < pSpeed->AAT + pSpeed->CAT){
        sp = pSpeed->IniSp + 0.5 * pSpeed->AASp * pSpeed->AAT * pSpeed->AAT + (t-pSpeed->AAT) * pSpeed->AASp * pSpeed->AAT;
    }
    else if(pSpeed->AAT + pSpeed->CAT <= t && t <= pSpeed->AAT + pSpeed->CAT + pSpeed->RAT){
        sp = pSpeed->IniSp+0.5*pSpeed->AASp*pSpeed->AAT*pSpeed->AAT+pSpeed->CAT*pSpeed->AASp*pSpeed->AAT+
        pSpeed->AASp*pSpeed->AAT*(t-pSpeed->AAT-pSpeed->CAT) - 0.5*pSpeed->AASp*(t-pSpeed->AAT-pSpeed->CAT)*(t-pSpeed->AAT-pSpeed->CAT);
    }

	return sp;
}

int Init_Motion_Table_S(Motion_TableTypeDef (*table)[TOTAL_POINTS],const Speed_LevelTypeDef *pSpeed,uint32_t *sumSteps,double *sumTimes){
int k = 0,pwm_count=0,pwm_cycle = 0;
double fi = 0;
	Motion_TableTypeDef *p = (Motion_TableTypeDef *)table;
	*sumSteps = 0;
	*sumTimes = 0;

	/*加加速阶段*/
	for(k = 0;k < AA_POINTS;k ++){
		fi = GetSpeed_ByTicks_S(pSpeed,pSpeed->AAT / AA_POINTS * k);
		pwm_cycle = PSC_CLK / fi;
		pwm_count = (pSpeed->AAT / AA_POINTS) * fi;
		p[k].Pwm_Cycle = pwm_cycle;
		p[k].Pwm_Counter = pwm_count;
		*sumSteps += pwm_count;
		*sumTimes += pwm_count / fi;
		RefDistance_S[pSpeed->pf][k] = *sumSteps;
	}
	/*匀加速阶段*/
	for(k = 0;k < CA_POINTS;k ++){
		fi= GetSpeed_ByTicks_S(pSpeed,pSpeed->AAT + pSpeed->CAT/ CA_POINTS * k);
		pwm_cycle = PSC_CLK / fi;
		pwm_count = (pSpeed->CAT/ CA_POINTS) * fi;
		p[AA_POINTS + k].Pwm_Cycle = pwm_cycle;
		p[AA_POINTS + k].Pwm_Counter = pwm_count;
		*sumSteps += pwm_count;
		*sumTimes += pwm_count / fi;
		RefDistance_S[pSpeed->pf][AA_POINTS + k] = *sumSteps;
	}
	/*减加速阶段*/
	for(k = 0;k < RA_POINTS;k ++){
		fi = GetSpeed_ByTicks_S(pSpeed,pSpeed->AAT + pSpeed->CAT + pSpeed->RAT/ RA_POINTS * k);
		pwm_cycle = PSC_CLK / fi;
		pwm_count = (pSpeed->RAT/ RA_POINTS) * fi;
		p[AA_POINTS + CA_POINTS + k].Pwm_Cycle = pwm_cycle;
		p[AA_POINTS + CA_POINTS + k].Pwm_Counter = pwm_count;
		*sumSteps += pwm_count;
		*sumTimes += pwm_count / fi;
		RefDistance_S[pSpeed->pf][AA_POINTS + CA_POINTS + k] = *sumSteps;
	}
	
		fi = GetSpeed_ByTicks_S(pSpeed,pSpeed->AAT + pSpeed->CAT + pSpeed->RAT);
		pwm_cycle = PSC_CLK / fi;
		pwm_count = (pSpeed->RAT/ RA_POINTS) * fi;
		p[AA_POINTS + CA_POINTS + RA_POINTS].Pwm_Cycle = pwm_cycle;
		p[AA_POINTS + CA_POINTS + RA_POINTS].Pwm_Counter = pwm_count;
		*sumSteps += pwm_count;
		*sumTimes += pwm_count / fi;
		RefDistance_S[pSpeed->pf][AA_POINTS + CA_POINTS + RA_POINTS] = *sumSteps;
	
	
	/*加减速阶段*/
	for(k = 0;k < AR_POINTS;k ++){
		fi = /*spmax - */GetSpeed_ByTicks_S(pSpeed,pSpeed->AAT + pSpeed->CAT + pSpeed->RAT - pSpeed->ART / AR_POINTS * k);
		pwm_cycle = PSC_CLK / fi;
		pwm_count = (pSpeed->ART / AR_POINTS) * fi;
		p[AA_POINTS + CA_POINTS + RA_POINTS + 1 + k].Pwm_Cycle = pwm_cycle;
		p[AA_POINTS + CA_POINTS + RA_POINTS + 1 + k].Pwm_Counter = pwm_count;
		*sumSteps += pwm_count;
		*sumTimes += pwm_count / fi;
		RefDistance_S[pSpeed->pf][AA_POINTS + CA_POINTS + RA_POINTS + 1 + k] = *sumSteps;
	}
	/*匀减速阶段*/
	for(k = 0;k < CR_POINTS;k ++){
		fi = /*spmax - */GetSpeed_ByTicks_S(pSpeed,pSpeed->AAT + pSpeed->CAT /* AA_TIME + CA_TIME*/ - pSpeed->CRT / CR_POINTS * k);
		pwm_cycle = PSC_CLK / fi;
		pwm_count = (pSpeed->CRT / CR_POINTS) * fi;
		p[AA_POINTS + CA_POINTS + RA_POINTS + AR_POINTS + 1 + k].Pwm_Cycle = pwm_cycle;
		p[AA_POINTS + CA_POINTS + RA_POINTS + AR_POINTS + 1 + k].Pwm_Counter = pwm_count;
		*sumSteps += pwm_count;
		*sumTimes += pwm_count / fi;
		RefDistance_S[pSpeed->pf][AA_POINTS + CA_POINTS + RA_POINTS + AR_POINTS + 1 + k] = *sumSteps;
	}
	/*减减速阶段*/
	for(k = 0;k < RR_POINTS;k ++){
		fi = /*spmax - */GetSpeed_ByTicks_S(pSpeed,pSpeed->AAT /* AA_TIME*/ - pSpeed->RRT / RR_POINTS * k);
		pwm_cycle = PSC_CLK / fi;
		pwm_count = (pSpeed->RRT / RR_POINTS) * fi;
		p[AA_POINTS + CA_POINTS + RA_POINTS + AR_POINTS + CR_POINTS + 1 + k].Pwm_Cycle = pwm_cycle;
		p[AA_POINTS + CA_POINTS + RA_POINTS + AR_POINTS + CR_POINTS + 1 + k].Pwm_Counter = pwm_count;
		*sumSteps += pwm_count;
		*sumTimes += pwm_count / fi;
		RefDistance_S[pSpeed->pf][AA_POINTS + CA_POINTS + RA_POINTS + AR_POINTS + CR_POINTS + 1 + k] = *sumSteps;
	}
	return 0;
}
















