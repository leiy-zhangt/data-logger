#include "qmc5883l.h"

void QMC5883L_Configuration(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能时钟
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC8
    I2C_SendByte(I2C1,QMC5883L_addr,0X0A,0X80);//寄存器恢复默认值
    if(QMC5883L_Range == 8) I2C_SendByte(I2C1,QMC5883L_addr,0X09,0B11010101);
    else if(QMC5883L_Range == 2)I2C_SendByte(I2C1,QMC5883L_addr,0X09,0B11000101);
    I2C_SendByte(I2C1,QMC5883L_addr,0X0A,0B01000001);
    I2C_SendByte(I2C1,QMC5883L_addr,0X0B,0X01);
}

void QMC5883L_MagnetismRead(uint8_t *buffer,double *data)
{
    int16_t data_int16[3];
    while((I2C_ReadByte(I2C1,QMC5883L_addr,0X06)&0X01) == 0);
    if((I2C_ReadByte(I2C1,QMC5883L_addr,0X06)&0X02) == 0)
    {
        I2C_ReadBuffer(I2C1,QMC5883L_addr,0X00,buffer,6);
        data_int16[0] = ((int16_t)buffer[1]<<8)|buffer[0];
        data_int16[1] = ((int16_t)buffer[3]<<8)|buffer[2];
        data_int16[2] = ((int16_t)buffer[5]<<8)|buffer[4];
        if(QMC5883L_Range == 2)
        {
            data[0] = data_int16[0]/65536.0*4;
            data[1] = data_int16[1]/65536.0*4;
            data[2] = data_int16[2]/65536.0*4;
        }
        else if(QMC5883L_Range == 8)
        {
            data[0] = data_int16[0]/65536.0*16;
            data[1] = data_int16[1]/65536.0*16;
            data[2] = data_int16[2]/65536.0*16;
        }
    }
    else 
    {
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
    }
}

double QMC5883L_TemperatureRead(void)
{
    uint8_t buffer[2];
    int16_t data;
    I2C_ReadBuffer(I2C1,QMC5883L_addr,0X07,buffer,2);
    data = ((int16_t)buffer[1]<<8)|buffer[0];
    return data/100.00;
}




