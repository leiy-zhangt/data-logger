#ifndef __COMMAND_H
#define __COMMAND_H

#include "sys.h"
#include "w25n.h"
#include "usart.h"
#include "bmi055.h"
#include "computation.h"

#define acc_g 9.8

extern u8 Command_Flag;//�����־λ

void Command_Execute(USART_TypeDef* USARTx);
void Commanad_ChipErase(void);
void Command_Bmi055StartWork(void);
void Command_Bmi055StopWork(void);
void Command_DataOutput(void);
void Command_StatusCheck(void);
void Command_BMI055_DataStorage(void); //BMI055���ݴ洢��������
void Command_BMI055_DataDisplay(void);//BMI055ԭʼ������ʾ����
void Command_AttitudeSolution(void); //BMI055��̬���㺯��  
void Command_BMI055_OFFSET(void);//BMI055ƫ��������
void Command_Q_Init(double *q);//��Ԫ��Q��ʼ��
void Command_AccelerationDisplay(void);//���ٶ�ԭʼֵ��ʾ
void Command_VelocityDisplay(void);//�ٶ�ֵ��ʾ
void Command_PositionDisplay(void);//λ��ֵ��ʾ

#endif
