#ifndef __W25N_H
#define __W25N_H			    
#include "sys.h" 
#include "spi.h"
#include "delay.h"	

#define W25N_Port  GPIOA
#define W25N_CS_Pin  GPIO_Pin_11
#define W25N_WP_Pin  GPIO_Pin_12
#define W25N_HOLD_Pin  GPIO_Pin_15

#define	W25N_CS PAout(11)  	//W25Q的片选信号
#define	W25N_WP PAout(12)  	//W25Q的写入保护
#define	W25N_HOLD PAout(15)  	//W25Q的保持

void W25N_Configuration(void);

#endif
















