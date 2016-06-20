#ifndef MOTIONCTRL_T_H
#define MOTIONCTRL_T_H

#include "motionTypeDef.h"

int Init_Default_T(void);
int Init_Speed_Table_T(void);
int Speed_Setup_T(Speed_LevelTypeDef *speed,uint16_t pf,/*uint32_t initsp,*/uint32_t maxsp,/*uint32_t stopsp,*/double acct,double dect);
int Init_Motion_Table_T(Motion_TableTypeDef (*table)[TOTAL_POINTS],const Speed_LevelTypeDef *pSpeed,uint32_t *sumSteps,double *sumTimes);
double GetSpeed_ByTicks_T(const Speed_LevelTypeDef *speed,double t);

#endif

