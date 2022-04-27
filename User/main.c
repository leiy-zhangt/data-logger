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

int main(void)
{ 
    __IO u8 res[2048]={0},page;
    __IO uint16_t n;
    RCC_Configuration();
    NVIC_Configuration();
	SPI1_Configuration();
    BMI055_Configuration();
    LED_Init();
    USART1_Configuration(115200,DISABLE);
    W25N_Configuration();
    SERVE_Configution(DISABLE);
    BUZZER_Configuration();
    
    for(n=0;n<2048;n++)
    {
        buffer[n]=n%255;
    }
    W25N_DataWrirte(buffer,0X00ff); 
    W25N_DataReceive(res,0X00ff);
    W25N_BlockErase(0x00ff/64);
    W25N_DataReceive(res,0X00ff);  
}

