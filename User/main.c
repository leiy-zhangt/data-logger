#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "iic.h"
#include "led.h"  
#include "bmi055.h"
#include "w25n.h"
#include "serve.h"

int main(void)
{ 
    RCC_Configuration();
    NVIC_Configuration();
	SPI1_Configuration();
    BMI055_Configuration();
    LED_Init();
    USART1_Configuration(115200,DISABLE);
    W25N_Configuration();
    SERVE_Configution();
    EN_SERVE = 1;
    while(1)
    {
        TIM_SetCompare1(TIM3,500);
        TIM_SetCompare2(TIM3,500);
        TIM_SetCompare3(TIM3,500);
        TIM_SetCompare4(TIM3,500);
        LED=!LED;
        delay_ms(1000);
        TIM_SetCompare1(TIM3,2500);
        TIM_SetCompare2(TIM3,2500);
        TIM_SetCompare3(TIM3,2500);
        TIM_SetCompare4(TIM3,2500);
        LED=!LED;
        delay_ms(1000);
    }
}

