#ifndef __BMI055_H
#define __BMI055_H

#include "sys.h"
#include "delay.h"
#include "spi.h"
#include "usart.h"
#include "led.h"
#include "computation.h"

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

//#define ACC_Range_2g 0X03
//#define ACC_Range_4g 0X05
//#define ACC_Range_8g 0X08
//#define ACC_Range_16g 0X0C

//#define GYR_Range_125 0X04
//#define GYR_Range_250 0X03
//#define GYR_Range_500 0X02
//#define GYR_Range_1000 0X01
//#define GYR_Range_2000 0X00

#define ODR_100Hz_32BD 0X07
#define ODR_200Hz_65BD 0X06
#define ODR_100Hz_12BD 0X05
#define ODR_200Hz_23BD 0X04
#define ODR_400Hz_47BD 0X03
#define ODR_1000Hz_116BD 0X02
#define ODR_2000Hz_230BD 0X01
#define ODR_2000Hz_523BD 0X10

#define BMI_ReadCmd(status) TIM_Cmd(TIM4,status) 
#define Start_Page 1

typedef enum {ACC_Choose=0,GYR_Choose=1}IMU_Choose;
typedef enum {BMI_Frequence_10Hz = 4999,BMI_Frequence_20Hz = 2499,BMI_Frequence_50Hz = 999}BMI_Frequence;
typedef enum {ACC_Range_2g = 0X03,ACC_Range_4g = 0X05,ACC_Range_8g = 0X08,ACC_Range_16g = 0X0C}ACC_Range_Choose;
typedef enum {GYR_Range_125 = 0X04,GYR_Range_250 = 0X03,GYR_Range_500 = 0X02,GYR_Range_1000 = 0X01,GYR_Range_2000 = 0X00}GYR_Range_Choose;

extern u8 location,bmi_buffer[2048],n;
extern int16_t acc_16,gyr_16;
extern double acc,gyr;
extern uint32_t data_number,final_number; //数据的数量 
extern double dt;
extern int ACC_Range,GYR_Range;  //IMU量程选择

void BMI055_Configuration(ACC_Range_Choose acc_range,GYR_Range_Choose gyr_range,BMI_Frequence frequence); //MI055初始化函数
void BMI055_SendData(IMU_Choose IMU,u8 addr,u8 data); //BMI055发送数据函数
u8 BMI055_ReadData(IMU_Choose IMU,u8 addr); //BMI055单字节接收
void BMI055_ReadBuffer(IMU_Choose IMU,u8 addr,u8* buffer,u8 length); //BMI055数组接收
int16_t BMI055_DataTransform(IMU_Choose IMU,u8 data_l,u8 data_h); //数据转换函数

#endif
