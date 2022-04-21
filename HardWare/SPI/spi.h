#ifndef __SPI_H
#define __SPI_H
#include "sys.h"

#define EN_SPI1 1
#define EN_SPI2 0

#define SPI1_Port GPIOA
#define SPI1_GPIO_CLK RCC_AHB1Periph_GPIOA 
#define SPI1_SCK GPIO_Pin_5
#define SPI1_MISO GPIO_Pin_6
#define SPI1_MOSI GPIO_Pin_7
#define SPI1_Pinsourse_SCK GPIO_PinSource5
#define SPI1_Pinsourse_MISO GPIO_PinSource6
#define SPI1_Pinsourse_MOSI GPIO_PinSource7

#define SPI2_Port GPIOB
#define SPI2_GPIO_CLK RCC_AHB1Periph_GPIOB 
#define SPI2_SCK GPIO_Pin_13
#define SPI2_MISO GPIO_Pin_14
#define SPI2_MOSI GPIO_Pin_15
#define SPI2_Pinsourse_SCK GPIO_PinSource13
#define SPI2_Pinsourse_MISO GPIO_PinSource14
#define SPI2_Pinsourse_MOSI GPIO_PinSource15

void SPI1_Configuration(void);			//��ʼ��SPI1�� 
void SPI2_Configuration(void);          //��ʼ��SPI2�� 
u8 SPI_ReadWriteByte(SPI_TypeDef* SPI, u8 Data);//SPI1���߶�дһ���ֽ�
		 
#endif

