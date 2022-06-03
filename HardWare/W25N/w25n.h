#ifndef __W25N_H
#define __W25N_H			    
#include "sys.h" 
#include "spi.h"
#include "delay.h"	

#define Start_Page 65 //���ݳ�ʼ����ҳ

#define W25N_Port  GPIOA
#define W25N_CS_Pin  GPIO_Pin_11
#define W25N_WP_Pin  GPIO_Pin_12
#define W25N_HOLD_Pin  GPIO_Pin_15

#define	W25N_CS PAout(11)  	//W25Q��Ƭѡ�ź�
#define	W25N_WP PAout(12)  	//W25Q��д�뱣��
#define	W25N_HOLD PAout(15)  	//W25Q�ı���

extern uint16_t page; 

void W25N_Configuration(void);
void W25N_WriteEnable(void);  //дʹ��
FlagStatus W25N_CheckBusy(void); //���BUSYλ
void W25N_WaitBusy(void);  //�ȴ�BUSYλ
void W25N_BlockErase(uint16_t block); //����0~1023��  Bloc = Page/64
void W25N_ChipErase(void); //��������оƬ
void W25N_DataWrirte(u8 *buffer,uint16_t page); //��Pageд������
void W25N_SendData(u8 addr,u8 data); //��Ĵ���д������
void  W25N_DataReceive(u8 *buffer,uint16_t page); //��Page�ж�ȡ����
#endif
















