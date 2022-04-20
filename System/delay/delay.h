#ifndef __DELAY_H
#define __DELAY_H 			   
#include <sys.h>	  

#define SysTick_CTRL_ENABLE_MASK 0X01
#define SysTick_CTRL_DISABLE_MASK 0X00000000

void delay_ms(u16 nms); //����ӳ�Ϊ65535ms
void delay_us(u32 nus); //����ӳ�Ϊ79815us

#endif





























