#ifndef __USART_H
#define __USART_H
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "stdio.h"	
#include "sys.h" 
#include "w25n.h"
#include "bmi055.h"

#define USART_n		USART1  //����ʹ��printf�����Ĵ��ڣ���������Ҫʹ��USART_printfר�ú�������

#define USART1_REC_LEN  			20  	//����USART1�������ֽ���
#define USART2_REC_LEN  			20  	//����USART2�������ֽ���
#define USART3_REC_LEN  			20  	//����USART3�������ֽ���

//��ʹ��ĳ������ʱҪ��ֹ�˴��ڣ��Լ��ٱ�����
#define EN_USART1 			1		//ʹ�ܣ�1��/��ֹ��0������1
#define EN_USART2 			1		//ʹ�ܣ�1��/��ֹ��0������2
#define EN_USART3 			1		//ʹ�ܣ�1��/��ֹ��0������3

typedef enum{Command_Receice = 1,Transfer = 2}USART_MODE_Selection;
	  	
extern u8  USART1_RX_BUF[USART1_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u8  USART2_RX_BUF[USART2_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern u8  USART3_RX_BUF[USART3_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
 
extern u16 USART1_RX_STA;         		//����״̬���	
extern u16 USART2_RX_STA;         		//����״̬���	
extern u16 USART3_RX_STA;         		//����״̬���	

extern USART_MODE_Selection USART_MODE;

//��������
void USART1_Configuration(u32 bound,FunctionalState ITStatus);//����1��ʼ��������
void USART2_Configuration(u32 bound,FunctionalState ITStatus);//����2��ʼ��������
void USART3_Configuration(u32 bound,FunctionalState ITStatus);//����3��ʼ��������
void USART1_printf(char* fmt,...); //����1��ר��printf����
void USART2_printf(char* fmt,...); //����2��ר��printf����
void USART3_printf(char* fmt,...); //����3��ר��printf����
void Command_Execute(USART_TypeDef* USARTx);
#endif


