#include "bmi055.h"
/*
1、读取数据必须从LSB开始，此时MSB有保护
2、BMI055为MSB模式，且地址项0：写入，1：读出

*/

void BMI055_Configuration(BMI_Frequence frequence)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
    //配置片选引脚
    GPIO_InitStructure.GPIO_Pin = ACC_CS_Pin|GYR_CS_Pin; //片选引脚配置为推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化片选引脚
    ACC_CS = 1;
    GYR_CS = 1;
    //配置中断引脚
    GPIO_InitStructure.GPIO_Pin = ACC_INT_Pin|GYR_INT_Pin; //片选引脚配置为推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化片选引脚
    //连接外部中断线
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource3);
    //EXTI配置
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;//LINE1
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
    EXTI_InitStructure.EXTI_LineCmd = ACC_INT_Cmd;//使能LINE1
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;//LINE3
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
    EXTI_InitStructure.EXTI_LineCmd = GYR_INT_Cmd;//使能LINE3
    EXTI_Init(&EXTI_InitStructure);
    //NVIC配置
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//外部中断0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);//配置
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//外部中断0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);//配置
    //配置BMI055
    BMI055_SendData(ACC_Choose,0X0F,ACC_Range_4g);//加速度计量程选择
    BMI055_SendData(ACC_Choose,0X10,0X0C); //滤波器带宽选择
    BMI055_SendData(GYR_Choose,0X0F,GYR_Range_125);//陀螺仪计量程选择
    BMI055_SendData(GYR_Choose,0X10,ODR_100Hz_32BD);//陀螺仪输出速率设置 
    //BMI055读取速率设置
    TIM_TimeBaseStructure.TIM_Prescaler=1679;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=BMI_Frequence_10Hz;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
    TIM_SetCounter(TIM4,0X00);
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update); 
    TIM_Cmd(TIM4,DISABLE);
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);//配置
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

int16_t BMI_DataTransform(IMU_Choose IMU,u8 data_l,u8 data_h)
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
    LED = !LED;
    if(TIM_GetITStatus(TIM4,TIM_IT_Update))
    {
        BMI055_ReadBuffer(ACC_Choose,0X02,bmi_data,6);
        delay_us(3);
        BMI055_ReadBuffer(GYR_Choose,0X02,bmi_data+6,6);
    }
    for(n=0;n<6;n++)
    {
        if(n<3)
        {
            acc_16 = BMI_DataTransform(ACC_Choose,bmi_data[2*n],bmi_data[2*n+1]);
            acc = acc_16/4096.00*8*9.8;
            printf("%+0.4f  ",acc);
        }
        else
        {
            gyr_16 = BMI_DataTransform(GYR_Choose,bmi_data[2*n],bmi_data[2*n+1]);
            gyr = gyr_16/65536.00*250;
            printf("%+0.4f  ",gyr);
            if(n==5) printf("\r\n");
        }
    }
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update); 
}
