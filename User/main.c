#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "iic.h"
#include "led.h"  
#include "bmi055.h"

int main(void)
{ 
    RCC_Configuration();
    NVIC_Configuration();
	SPI1_Configuration();
    BMI055_Configuration();
    LED_Init();
    USART1_Configuration(115200,DISABLE);
    while(1)
    {
        __IO u8 data[2],id;
        __IO int16_t tem_16;
        __IO float tem;
//        id = BMI055_ReadData(ACC_Choose,0X00);
//        BMI055_ReadBuffer(ACC_Choose,0X06,data,2);
//        tem_16=((int16_t)data[1])<<8;
//        tem_16 = tem_16|data[0];
//        tem_16 = tem_16>>4;
////        tem = (tem_16>>4);
//        tem=tem_16/4096.00*32;
//        printf("g:%0.4f    acc:%d\r\n",tem,tem_16);
//        delay_ms(5);
        
        BMI055_ReadBuffer(GYR_Choose,0X02,data,2);
        tem_16=((int16_t)data[1])<<8;
        tem_16 = tem_16|data[0];
        tem=tem_16/65536.00*250;
        printf("W:%0.4f\r\n",tem);
        delay_ms(3);
    }
}

