#ifndef __SERVE_H
#define __SERVE_H
#include "sys.h"

#define EN_SERVE PCout(12)
#define psc  83  //预分频器
#define arr  19999 //自动重装载值
//角度控制精度为0.1度
#define CH1_Angle_Set(angle) TIM_SetCompare1(TIM3,500+angle/180*2000)
#define CH2_Angle_Set(angle) TIM_SetCompare2(TIM3,500+angle/180*2000)
#define CH3_Angle_Set(angle) TIM_SetCompare3(TIM3,500+angle/180*2000)
#define CH4_Angle_Set(angle) TIM_SetCompare4(TIM3,500+angle/180*2000)

void SERVE_Configution(FunctionalState SERVE_State);

#endif
