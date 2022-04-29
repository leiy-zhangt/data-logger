#include "bmi055.h"
/*
1����ȡ���ݱ����LSB��ʼ����ʱMSB�б���
2��BMI055ΪMSBģʽ���ҵ�ַ��0��д�룬1������
3�����ݸ�ʽ  addr_addr_dataX12_0X00_0X00 ��16Byte*128
*/

u8 location=0,bmi_buffer[2048],n;
int16_t acc_16,gyr_16;
double acc,gyr;
uint32_t data_number=0,final_number; //���ݵ����� 

void BMI055_Configuration(BMI_Frequence frequence)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
    //����Ƭѡ����
    GPIO_InitStructure.GPIO_Pin = ACC_CS_Pin|GYR_CS_Pin; //Ƭѡ��������Ϊ�������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��Ƭѡ����
    ACC_CS = 1;
    GYR_CS = 1;
    //�����ж�����
    GPIO_InitStructure.GPIO_Pin = ACC_INT_Pin|GYR_INT_Pin; //Ƭѡ��������Ϊ�������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��Ƭѡ����
    //�����ⲿ�ж���
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource3);
    //EXTI����
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;//LINE1
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //�����ش��� 
    EXTI_InitStructure.EXTI_LineCmd = ACC_INT_Cmd;//ʹ��LINE1
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;//LINE3
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //�����ش��� 
    EXTI_InitStructure.EXTI_LineCmd = GYR_INT_Cmd;//ʹ��LINE3
    EXTI_Init(&EXTI_InitStructure);
    //NVIC����
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//�ⲿ�ж�0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//�����ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);//����
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//�ⲿ�ж�0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//�����ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);//����
    //����BMI055
    BMI055_SendData(ACC_Choose,0X0F,ACC_Range_4g);//���ٶȼ�����ѡ��
    BMI055_SendData(ACC_Choose,0X10,0X0C); //�˲�������ѡ��
    BMI055_SendData(GYR_Choose,0X0F,GYR_Range_125);//�����Ǽ�����ѡ��
    BMI055_SendData(GYR_Choose,0X10,ODR_100Hz_32BD);//����������������� 
    //BMI055��ȡ��������
    TIM_TimeBaseStructure.TIM_Prescaler=1679;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=BMI_Frequence_50Hz;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//��ʼ����ʱ��
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update); 
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
    TIM_SetCounter(TIM4,0X00);
    TIM_Cmd(TIM4,DISABLE);
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//�����ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);//����
}



void BMI055_SendData(IMU_Choose IMU,u8 addr,u8 data)
{
    if(IMU == ACC_Choose) ACC_CS = 0;
    else  GYR_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,addr&0X7f);
    SPI_ReadWriteByte(SPI1,data);
    delay_us(3);
    if(IMU == ACC_Choose) ACC_CS = 1;
    else  GYR_CS = 1;
}

void BMI055_ReadBuffer(IMU_Choose IMU,u8 addr,u8* buffer,u8 length)
{
    if(IMU == ACC_Choose) ACC_CS = 0;
    else  GYR_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,addr|0X80);
    for(;length>0;length--)
    {
        *buffer = SPI_ReadWriteByte(SPI1,0X00);
        buffer++;
    }
    if(IMU == ACC_Choose) ACC_CS = 1;
    else  GYR_CS = 1;
}

u8 BMI055_ReadData(IMU_Choose IMU,u8 addr)
{
    u8 res;
    if(IMU == ACC_Choose) ACC_CS = 0;
    else  GYR_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,addr|0X80);
    res = SPI_ReadWriteByte(SPI1,0X00);
    if(IMU == ACC_Choose) ACC_CS = 1;
    else  GYR_CS = 1;
    return res;
}

int16_t BMI055_DataTransform(IMU_Choose IMU,u8 data_l,u8 data_h)
{
    static int16_t data;
    data=(((int16_t)data_h)<<8)|data_l;
    if(IMU == ACC_Choose) return data>>4;
    else return data;
}

void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1))
    {
        
    }
    EXTI_ClearITPendingBit(EXTI_Line1);
}

void EXTI3_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line3))
    {
        
    }
    EXTI_ClearITPendingBit(EXTI_Line3);
}

void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4,TIM_IT_Update))
    {
        bmi_buffer[16*location]=data_number>>24;
        bmi_buffer[16*location+1]=data_number>>16;
        bmi_buffer[16*location+2]=data_number>>8;
        bmi_buffer[16*location+3]=data_number;
        data_number++;
        BMI055_ReadBuffer(ACC_Choose,0X02,bmi_buffer+2+16*location,6);
        delay_us(3);
        BMI055_ReadBuffer(GYR_Choose,0X02,bmi_buffer+8+16*location,6);
        location++;
        if(location==128)
        {
            location = 0;
            W25N_DataWrirte(bmi_buffer,page);
            page++;
        }
    }
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update); 
}

