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
    USART1_Configuration(256000,DISABLE);
    BMI055_Configuration(BMI_Frequence_10Hz);
    LED_Init();
    W25N_Configuration();
    SERVE_Configution(DISABLE);
    BUZZER_Configuration();
    delay_ms(2000);
    printf("acc_x  acc_y  acc_z  gyr_x  gyr_y  gyr_z\r\n");
    delay_ms(2000);
    BMI_ReadCmd(ENABLE);
    while(1)
    {
//        LED = !LED;
    }
}

