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
    delay_init(168);
    LED_Init();
    uart_init(115200);
    while(1)
    {
        USART_SendData(USART1,'a');
        LED=!LED;
        delay_ms(1000);
    }
}

