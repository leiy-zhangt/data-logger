#include "w25n.h"    

////4KbytesΪһ��Sector
////16������Ϊ1��Block
////W25128
////����Ϊ16M�ֽ�,����128��Block,4096��Sector 
//													 
//��ʼ��SPI FLASH��IO��
void W25N_Configuration(void)
{ 
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
    GPIO_InitStructure.GPIO_Pin = W25N_CS_Pin|W25N_WP_Pin|W25N_HOLD_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(W25N_Port,&GPIO_InitStructure);//��ʼ��
    W25N_CS = 1;
    W25N_HOLD = 1;
    W25N_WP;
}  


























