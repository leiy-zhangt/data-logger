#ifndef __SERVE_H
#define __SERVE_H
#include "sys.h"

#define EN_SERVE PCout(12)
#define psc  83  //Ԥ��Ƶ��
#define arr  4999 //�Զ���װ��ֵ
//�Ƕȿ��ƾ���Ϊ0.1��
#define CH1_Angle_Set(angle) TIM_SetCompare1(TIM3,430+angle/180.0*2000.0)
#define CH2_Angle_Set(angle) TIM_SetCompare2(TIM3,470+angle/180.0*2000.0)
#define CH3_Angle_Set(angle) TIM_SetCompare3(TIM3,520+angle/180.0*2000.0)
#define CH4_Angle_Set(angle) TIM_SetCompare4(TIM3,525+angle/180.0*2000.0)

void SERVE_Configution(FunctionalState SERVE_State);

#endif
