#ifndef MOTIONCTRL_S_H
#define MOTIONCTRL_S_H

#include "stdint.h"
//#include "stdbool.h"
#include "math.h"
#include "motionTypeDef.h"

int Init_Default_S(void);
int Init_Speed_Table_S(void);
int Speed_Setup_S(Speed_LevelTypeDef *speed,int pf,uint32_t maxsp,double acctime);
//int Speed_Setup_S(Speed_LevelTypeDef *speed,int pf,/*uint32_t initsp,*/uint32_t maxsp,double acctime,double accsp);
int Init_Motion_Table_S(Motion_TableTypeDef (*table)[TOTAL_POINTS],const Speed_LevelTypeDef *pSpeed,uint32_t *sumSteps,double *sumTimes);
double GetSpeed_ByTicks_S(const Speed_LevelTypeDef *pSpeed,double t);

#endif

