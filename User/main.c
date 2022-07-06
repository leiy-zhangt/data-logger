/*
指令集：
CE:W25N擦除
BE:BMI055开始读取数据并保存
BD：BMI055停止工作
DR:读取数据
DD:显示原始数据
AS:显示解算后的姿态角
BO:消除偏移量
QI：四元数初始化
AD:惯性空间加速度显示
VD:惯性空间速度显示
PD:惯性空间位置显示
EC:使能控制功能
DE:失能控制功能
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
#include "adi16448.h"
#include "fuse.h"

int main(void)
{ 
    RCC_Configuration();
    NVIC_Configuration();
	SPI1_Configuration();
//	SPI2_Configuration();
    I2C1_Configuration();
//    I2C2_Configuration();
    USART1_Configuration(115200,ENABLE);
    USART2_Configuration(9600,DISABLE);
    delay_ms(1000);
    W25N_Configuration();
    BMI055_Configuration(ACC_Range_16g,GYR_Range_1000,BMI_Frequence_100Hz);
    LED_Init();
	ADI16448_Configuration();
    SERVE_Configution(ENABLE);
    BUZZER_Configuration();
    BMP388_Configuration();
	FUSE_Configuration();
//    QMC5883L_Configuration();
    ATGM336H_Configuration(); 
    printf("DATA LOGGER has read\r\n");
    LED = 0;
    while(1)
    {
       
    }
}

