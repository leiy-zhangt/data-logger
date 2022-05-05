/*
ָ���
CE:W25N����
BE:BMI055��ʼ��ȡ���ݲ�����
BD��BMI055ֹͣ����
DR:��ȡ����
DD:��ʾԭʼ����
AS:��ʾ��������̬��
BO:����ƫ����
*/
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "iic.h"
#include "led.h"  
#include "bmi055.h"
#include "w25n.h"
#include "serve.h"
#include "buzzer.h"
#include "computation.h"

int main(void)
{ 
    RCC_Configuration();
    NVIC_Configuration();
	SPI1_Configuration();
    USART1_Configuration(115200,ENABLE);
    delay_ms(1000);
    BMI055_Configuration(ACC_Range_2g,GYR_Range_250,BMI_Frequence_50Hz);
    LED_Init();
    W25N_Configuration();
    SERVE_Configution(DISABLE);
    BUZZER_Configuration();
    LED=1; 
    printf("DATA LOGGER has read\r\n");
//    BMI_ReadCmd(ENABLE);
    while(1)
    {

    }
}

