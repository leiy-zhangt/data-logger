#include "adi16448.h"

void ADI16448_Configuration(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOA,GPIOEʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //KEY0 KEY1 KEY2��Ӧ����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ����ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOA0
    ADI16448_CS = 1;
}