#ifndef __BMI055_H
#define __BMI055_H

#include "sys.h"
#include "delay.h"
#include "spi.h"

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

#define ACC_Range_2g 0X03
#define ACC_Range_4g 0X05
#define ACC_Range_8g 0X08
#define ACC_Range_16g 0X0C

typedef enum {ACC_Choose=0,GYR_Choose=1}IMU_Choose;

void BMI055_Configuration(void);
void BMI055_SendData(IMU_Choose IMU,u8 addr,u8 data); //BMI055发送数据函数
u8 BMI055_ReadData(IMU_Choose IMU,u8 addr);
void BMI055_ReadBuffer(IMU_Choose IMU,u8 addr,u8* buffer,u8 length);

#endif
