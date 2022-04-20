#ifndef __BMI055_H
#define __BMI055_H

#include "sys.h"

#define ACC_CS PCout(0)
#define GYR_CS PCout(2) 
#define ACC_INT_Cmd DISABLE
#define GYR_INT_Cmd DISABLE
#define ACC_Port GPIOC
#define ACC_CS_Pin GPIO_Pin_0
#define ACC_INT_Pin GPIO_Pin_1
#define GYR_Port GPIOC
#define GYR_CS_Pin GPIO_Pin_2
#define GYR_INT_Pin GPIO_Pin_3

void BMI055_Configuration(void);

#endif
