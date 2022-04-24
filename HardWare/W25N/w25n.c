#include "w25n.h"    

////4Kbytes为一个Sector
////16个扇区为1个Block
////W25128
////容量为16M字节,共有128个Block,4096个Sector 
//													 
//初始化SPI FLASH的IO口
void W25N_Configuration(void)
{ 
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
    GPIO_InitStructure.GPIO_Pin = W25N_CS_Pin|W25N_WP_Pin|W25N_HOLD_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(W25N_Port,&GPIO_InitStructure);//初始化
    W25N_CS = 1;
    W25N_HOLD = 1;
    W25N_WP;
}  


























