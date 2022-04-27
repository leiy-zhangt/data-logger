#ifndef __BMI055_H
#define __BMI055_H

#include "sys.h"
#include "delay.h"
#include "spi.h"
#include "usart.h"
#include "led.h"

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

#define GYR_Range_125 0X04
#define GYR_Range_250 0X03
#define GYR_Range_500 0X02
#define GYR_Range_1000 0X01
#define GYR_Range_2000 0X00

#define ODR_100Hz_32BD 0X07
#define ODR_200Hz_65BD 0X06
#define ODR_100Hz_12BD 0X05
#define ODR_200Hz_23BD 0X04
#define ODR_400Hz_47BD 0X03
#define ODR_1000Hz_116BD 0X02
#define ODR_2000Hz_230BD 0X01
#define ODR_2000Hz_523BD 0X10

#define BMI_ReadCmd(status) TIM_Cmd(TIM4,status) 

typedef enum {ACC_Choose=0,GYR_Choose=1}IMU_Choose;
typedef enum {BMI_Frequence_10Hz = 4999,BMI_Frequence_20Hz = 2499,BMI_Frequence_50Hz = 999}BMI_Frequence;

static u8 bmi_data[12],n;
static int16_t acc_16,gyr_16;
static double acc,gyr;
    
void BMI055_Configuration(BMI_Frequence frequence); //MI055初始化函数
void BMI055_SendData(IMU_Choose IMU,u8 addr,u8 data); //BMI055发送数据函数
u8 BMI055_ReadData(IMU_Choose IMU,u8 addr); //BMI055单字节接收
void BMI055_ReadBuffer(IMU_Choose IMU,u8 addr,u8* buffer,u8 length); //BMI055数组接收

#endif
