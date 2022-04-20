#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
//#include "w25qxx.h"
#include "iic.h"
#include "led.h"  

int main(void)
{ 
    RCC_Configuration();
    NVIC_Configuration();
	SPI1_Configuration();
    LED_Init();
    USART1_Configuration(115200,ENABLE);
    while(1)
    {
        
    }
}

