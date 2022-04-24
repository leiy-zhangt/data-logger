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
    RCC_Configuration();
    NVIC_Configuration();
	SPI1_Configuration();
    BMI055_Configuration();
    LED_Init();
    USART1_Configuration(115200,DISABLE);
    W25N_Configuration();
    SERVE_Configution(ENABLE);
    BUZZER_Configuration();
    while(1)
    {
        
    }
}

