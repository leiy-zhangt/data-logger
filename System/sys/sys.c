#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 

//ϵͳʱ�ӳ�ʼ��	
//����ʱ������/�жϹ���/GPIO���õ�

//////////////////////////////////////////////////////////////////////////////////  


//THUMBָ�֧�ֻ������
//�������·���ʵ��ִ�л��ָ��WFI  
__asm void WFI_SET(void)
{
	WFI;		  
}
//�ر������ж�(���ǲ�����fault��NMI�ж�)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR	  
}
//���������ж�
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR  
}
//����ջ����ַ
//addr:ջ����ַ
__asm void MSR_MSP(u32 addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}
//�����жϷ���
void NVIC_Configuration(void)  
{ 
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
}

void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON); /* Enable HSE ʹ���ⲿ���پ���*/   
	HSEStartUpStatus = RCC_WaitForHSEStartUp(); /* Wait till HSE is ready �ȴ��ⲿ���پ���ʹ�����*/   
	if(HSEStartUpStatus == SUCCESS){   
		/*����PLLʱ��Դ����Ƶϵ��*/   
		RCC_PLLConfig(RCC_PLLSource_HSE,8,336,2,7); //����PLLʱ��Ƶ�� 
		/*����AHBʱ�ӣ�HCLK��*/   
		RCC_HCLKConfig(RCC_SYSCLK_Div1); //RCC_SYSCLK_Div1����AHBʱ�� = ϵͳʱ��(SYSCLK) = 168MHZ���ⲿ����8HMZ��   
		/*ע��˴������ã����ʹ��SYSTICK����ʱ���򣬴�ʱSYSTICK(Cortex System timer)=HCLK/8=9MHZ*/   
		RCC_PCLK1Config(RCC_HCLK_Div4); //���õ���AHBʱ�ӣ�PCLK1��,RCC_HCLK_Div4����APB1ʱ�� = HCLK/4 = 42MHZ���ⲿ����8HMZ��   
		RCC_PCLK2Config(RCC_HCLK_Div2); //���ø���AHBʱ�ӣ�PCLK2��,RCC_HCLK_Div1����APB2ʱ�� = HCLK = 84MHZ���ⲿ����8HMZ��   
		/*ע��AHB��Ҫ�����ⲿ�洢��ʱ�ӡ�APB2����AD��I/O���߼�TIM������1��APB1����DA��USB��SPI��I2C��CAN������2��3��4��5����ͨTIM */  
		FLASH_SetLatency(FLASH_Latency_5); //����FLASH�洢����ʱʱ��������     
		FLASH_PrefetchBufferCmd(ENABLE); //ѡ��FLASHԤȡָ�����ģʽ��Ԥȡָ����ʹ�� 
        //FLASH�ӳٲ�����Ҫ�ο���ѹ��ϵͳƵ��
		RCC_PLLCmd(ENABLE);	//ʹ��PLL
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); //�ȴ�PLL����ȶ�   
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //ѡ��SYSCLKʱ��ԴΪPLL
		while(RCC_GetSYSCLKSource() != 0x08); //�ȴ�PLL��ΪSYSCLKʱ��Դ 
    }
}












