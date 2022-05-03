/*
指令集：
CE:W25N擦除
BE:BMI055开始读取数据
BD：BMI055停止工作
DR:读取数据
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
    BMI055_Configuration(ACC_Range_4g,GYR_Range_125,BMI_Frequence_10Hz);
    LED_Init();
    W25N_Configuration();
    SERVE_Configution(DISABLE);
    BUZZER_Configuration();
    LED=1; 
    printf("has read\r\n");
    BMI_ReadCmd(DISABLE);
    while(1)
    {

    }
}

