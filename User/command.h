#ifndef __COMMAND_H
#define __COMMAND_H

#include "sys.h"
#include "w25n.h"
#include "usart.h"
#include "bmi055.h"
#include "computation.h"

#define acc_g 9.7911

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
#endif
