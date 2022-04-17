#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "w25qxx.h"
#include "myiic.h"
#include "iic.h"
//#include "key.h"  

int main(void)
{ 
	I2C1_Configuration();
    I2C_SendByte(I2C1,0X77<<1,0X00,0X00);
}

