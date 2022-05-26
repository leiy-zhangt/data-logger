#include "iic.h"

#ifdef EN_I2C1

void I2C1_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	//����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Pin = I2C1_SCL|I2C1_SDA;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(I2C1_Port,&GPIO_InitStructure);
	//���ø���ģʽ
	GPIO_PinAFConfig(I2C1_Port,I2C1_Pinsourse_SCL,GPIO_AF_I2C1);
	GPIO_PinAFConfig(I2C1_Port,I2C1_Pinsourse_SDA,GPIO_AF_I2C1);
	//����I2C����
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C1_Speed;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_OwnAddress1 = 0X11;
    I2C_Init(I2C1,&I2C_InitStructure);
    I2C_Cmd(I2C1,ENABLE);
}
#endif

#ifdef EN_I2C2

void I2C2_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	//����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
	//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Pin = I2C2_SCL|I2C2_SDA;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(I2C2_Port,&GPIO_InitStructure);
	//���ø���ģʽ
	GPIO_PinAFConfig(I2C2_Port,I2C2_Pinsourse_SCL,GPIO_AF_I2C2);
	GPIO_PinAFConfig(I2C2_Port,I2C2_Pinsourse_SDA,GPIO_AF_I2C2);
	//����I2C����
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C2_Speed;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_OwnAddress1 = 0X11;
    I2C_Init(I2C2,&I2C_InitStructure);
    I2C_Cmd(I2C2,ENABLE);
}

#endif


void I2C_SendByte(I2C_TypeDef* I2C,u8 SlaveAddr,u8 WriteAddr,u8 Data)
{
    I2C_GenerateSTART(I2C,ENABLE);
    while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C,SlaveAddr,I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2C,WriteAddr);
    while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED));    
    I2C_SendData(I2C,Data);
    while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_GenerateSTOP(I2C,ENABLE);
}

void I2C_SendBuffer(I2C_TypeDef* I2C,u8 SlaveAddr,u8 WriteAddr,u8 *buffer,u16 length)
{
    while(I2C_GetFlagStatus(I2C,I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2C,ENABLE);
    while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_MODE_SELECT));	//��� EV5
	I2C_Send7bitAddress(I2C,SlaveAddr, I2C_Direction_Transmitter); //д��������ַ
	while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//��� EV6
    I2C_SendData(I2C,WriteAddr); //�ڲ����ܵ�ַ
	while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED));//��λ�Ĵ����ǿգ����ݼĴ����ѿգ�����EV8���������ݵ�DR��������¼�
	while(length--){ //ѭ����������	
		I2C_SendData(I2C,*buffer); //��������
		buffer++; //����ָ����λ
		while (!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED));//���EV8
	}
	I2C_GenerateSTOP(I2C,ENABLE);//����ֹͣ�ź�
}

u8 I2C_ReadByte(I2C_TypeDef* I2C,u8 SlaveAddr,u8 ReadAddr)      //I2C��ȡһ���ֽ�
{ 
	while(I2C_GetFlagStatus(I2C,I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C,ENABLE);
	while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C,SlaveAddr,I2C_Direction_Transmitter); 
	while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C,ReadAddr);
	while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTART(I2C,ENABLE);
	while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C,SlaveAddr,I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	I2C_AcknowledgeConfig(I2C,DISABLE); //�����һ������ʱ�ر�Ӧ��λ
	I2C_GenerateSTOP(I2C,ENABLE);	//���һ������ʱʹ��ֹͣλ
    I2C_AcknowledgeConfig(I2C,ENABLE);
    while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_RECEIVED));
	return I2C_ReceiveData(I2C);
}

void I2C_ReadBuffer(I2C_TypeDef* I2C,u8 SlaveAddr,u8 ReadAddr,u8* buffer,u16 length)
{ //I2C��ȡ���ݴ���������ַ���Ĵ������ڲ���ַ��������
    while(I2C_GetFlagStatus(I2C,I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2C,ENABLE);//�����ź�
    while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_MODE_SELECT));	//��� EV5
    I2C_Send7bitAddress(I2C,SlaveAddr,I2C_Direction_Transmitter); //д��������ַ
    while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//��� EV6
    //	I2C_Cmd(I2C,ENABLE);
    I2C_SendData(I2C,ReadAddr); //���Ͷ��ĵ�ַ
    while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); //��� EV8
    I2C_GenerateSTART(I2C,ENABLE); //�����ź�
    while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_MODE_SELECT)); //��� EV5
    I2C_Send7bitAddress(I2C,SlaveAddr,I2C_Direction_Receiver); //��������ַ����������Ϊ��
    while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); //���EV6
    while(length){
        if(length == 1){ //ֻʣ�����һ������ʱ���� if ���
            I2C_AcknowledgeConfig(I2C,DISABLE); //�����һ������ʱ�ر�Ӧ��λ
            I2C_GenerateSTOP(I2C,ENABLE);	//���һ������ʱʹ��ֹͣλ
        }
        if(I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_RECEIVED)){ //��ȡ����
            *buffer = I2C_ReceiveData(I2C);//���ÿ⺯��������ȡ���� pBuffer
            buffer++; //ָ����λ
            length--; //�ֽ����� 1 
        }
    }
    I2C_AcknowledgeConfig(I2C,ENABLE);
}
