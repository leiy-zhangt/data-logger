#include "serve.h"

void SERVE_Configution(FunctionalState SERVE_State)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	//����ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM14ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//ʹ��PORTBʱ��	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��PORTCʱ��
    //���ö��ʹ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;      
	GPIO_Init(GPIOC,&GPIO_InitStructure);  
    if(SERVE_State == ENABLE) EN_SERVE = 1;
    else EN_SERVE = 0;
    //���ö���������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;      
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;      
	GPIO_Init(GPIOB,&GPIO_InitStructure); 
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3);
    TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//��ʼ����
    //���ͨ��ʹ��
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 4OC1
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 4OC2
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 4OC3
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 4OC4
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ��� 
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ��� 
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR3�ϵ�Ԥװ�ؼĴ��� 
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR4�ϵ�Ԥװ�ؼĴ��� 
    TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPEʹ�� 	
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
    CH1_Angle_Set(90);
	CH2_Angle_Set(90);
	CH3_Angle_Set(90);
	CH4_Angle_Set(90);
}
