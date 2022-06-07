#include "spi.h"

#ifdef EN_SPI1
void SPI1_Configuration(void)
{	 
    GPIO_InitTypeDef  GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
    //����ʱ��
    RCC_AHB1PeriphClockCmd(SPI1_GPIO_CLK, ENABLE);//ʹ��GPIOBʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//ʹ��SPI1ʱ��
    //GPIO��ʼ������
    GPIO_InitStructure.GPIO_Pin = SPI1_SCK|SPI1_MISO|SPI1_MOSI;//PB3~5���ù������	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(SPI1_Port, &GPIO_InitStructure);//��ʼ��

    GPIO_PinAFConfig(SPI1_Port,SPI1_Pinsourse_SCK,GPIO_AF_SPI1); //PB3����Ϊ SPI1
    GPIO_PinAFConfig(SPI1_Port,SPI1_Pinsourse_MISO,GPIO_AF_SPI1); //PB4����Ϊ SPI1
    GPIO_PinAFConfig(SPI1_Port,SPI1_Pinsourse_MOSI,GPIO_AF_SPI1); //PB5����Ϊ SPI1

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//����ͬ��ʱ�ӵĵ�һ�������أ��������½������ݱ�����
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
    SPI_Init(SPI1, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
    SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����		 
}   
#endif

#ifdef EN_SPI2
void SPI2_Configuration(void)
{	 
    GPIO_InitTypeDef  GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
    //����ʱ��
    RCC_AHB1PeriphClockCmd(SPI2_GPIO_CLK, ENABLE);//ʹ��GPIOBʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//ʹ��SPI1ʱ��
    //GPIO��ʼ������
    GPIO_InitStructure.GPIO_Pin = SPI2_SCK|SPI2_MISO|SPI2_MOSI;//PB3~5���ù������	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(SPI2_Port, &GPIO_InitStructure);//��ʼ��

    GPIO_PinAFConfig(SPI2_Port,SPI2_Pinsourse_SCK,GPIO_AF_SPI2); //PB3����Ϊ SPI1
    GPIO_PinAFConfig(SPI2_Port,SPI2_Pinsourse_MISO,GPIO_AF_SPI2); //PB4����Ϊ SPI1
    GPIO_PinAFConfig(SPI2_Port,SPI2_Pinsourse_MOSI,GPIO_AF_SPI2); //PB5����Ϊ SPI1

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
    SPI_Init(SPI2, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
    SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����		 
}   
#endif

u16 SPI_ReadWriteByte(SPI_TypeDef*SPI,u16 Data)
{		 			 
    while (SPI_I2S_GetFlagStatus(SPI,SPI_I2S_FLAG_TXE) == RESET){}//�ȴ���������  
    SPI_I2S_SendData(SPI, Data); //ͨ������SPIx����һ��byte  ����	
    while (SPI_I2S_GetFlagStatus(SPI,SPI_I2S_FLAG_RXNE) == RESET){} //�ȴ�������һ��byte  
    return SPI_I2S_ReceiveData(SPI); //����ͨ��SPIx������յ�����			    
}








