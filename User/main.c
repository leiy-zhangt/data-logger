/*
ָ���
CE:W25N����
BE:BMI055��ʼ��ȡ���ݲ�����
BD��BMI055ֹͣ����
DR:��ȡ����
DD:��ʾԭʼ����
AS:��ʾ��������̬��
BO:����ƫ����
QI����Ԫ����ʼ��
AD:���Կռ���ٶ���ʾ
VD:���Կռ��ٶ���ʾ
PD:���Կռ�λ����ʾ
*/
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "iic.h"
#include "led.h"  
#include "bmi055.h"
#include "w25n.h"
#include "bmp388.h"
#include "qmc5883l.h"
#include "serve.h"
#include "buzzer.h"
#include "atgm336h.h"
#include "computation.h"

int main(void)
{ 
    RCC_Configuration();
    NVIC_Configuration();
	SPI1_Configuration();
    I2C1_Configuration();
    I2C2_Configuration();
    USART1_Configuration(115200,ENABLE);
    USART2_Configuration(9600,ENABLE);
    delay_ms(1000);
    BMI055_Configuration(ACC_Range_2g,GYR_Range_125,BMI_Frequence_50Hz);
    LED_Init();
    W25N_Configuration();
    SERVE_Configution(DISABLE);
    BUZZER_Configuration();
    BMP388_Configuration();
    QMC5883L_Configuration();
    ATGM336H_Configuration();
    printf("DATA LOGGER has read\r\n");
    LED = 1;
    while(1)
    {
        
    }
}

