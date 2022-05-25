#ifndef __COMMAND_H
#define __COMMAND_H

#include "sys.h"
#include "w25n.h"
#include "usart.h"
#include "bmi055.h"
#include "computation.h"

#define acc_g 9.8

extern u8 Command_Flag;//命令标志位

void Command_Execute(USART_TypeDef* USARTx);
void Commanad_ChipErase(void);
void Command_Bmi055StartWork(void);
void Command_Bmi055StopWork(void);
void Command_DataOutput(void);
void Command_StatusCheck(void);
void Command_BMI055_DataStorage(void); //BMI055数据存储操作函数
void Command_BMI055_DataDisplay(void);//BMI055原始数据显示函数
void Command_AttitudeSolution(void); //BMI055姿态解算函数  
void Command_BMI055_OFFSET(void);//BMI055偏移量设置
void Command_Q_Init(double *q);//四元数Q初始化
void Command_AccelerationDisplay(void);//加速度原始值显示
void Command_VelocityDisplay(void);//速度值显示
void Command_PositionDisplay(void);//位置值显示

#endif
