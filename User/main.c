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
AD:加速度显示
FU:解锁
FL:上锁
FS:立即点火
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
#include "serve.h"
#include "buzzer.h"
#include "computation.h"

int main(void)
{ 
    RCC_Configuration();
    NVIC_Configuration();
	SPI1_Configuration();
    USART1_Configuration(115200,ENABLE);
    BMI055_Configuration(ACC_Range_8g,GYR_Range_125,BMI_Frequence_50Hz);
    LED_Init();
    W25N_Configuration();
    SERVE_Configution(DISABLE);
    BUZZER_Configuration();
    BMP388_Configuration();
    delay_ms(1000);
    printf("DATA LOGGER has read\r\n");
    while(1)
    {
        while(FIRE_Flag == 0);
        if(FIRE_Flag == 1)
        {
            LED = 1;
            time_count = 0;
            while(time_count<325); 
            EN_SERVE = 1;
            Command_Bmi055StopWork();
            delay_ms(500);
            printf("logger has fired!\r\n");
            FIRE_Flag = 0;
            LED = 0;
        }
        while(acceleration_test == 0);
    }
}

