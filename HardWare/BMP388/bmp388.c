#include "bmp388.h"

//最高位为0：写  最高位为1：读

BMP388_Calibration_Data_Struct BMP388_Calibration_Data;
BMP388_Calibration_QuantizedData_Struct BMP388_Calibration_QuantizedData;

void BMP388_Configuration(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(BMP388_CLK, ENABLE);//
    //配置片选引脚为推挽输出
    GPIO_InitStructure.GPIO_Pin = BMP388_CS_Pin; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//片选引脚配置为推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
    GPIO_Init(BMP388_Port, &GPIO_InitStructure);//初始化片选引脚
    //配置数据中断引脚为下拉输入
    GPIO_InitStructure.GPIO_Pin = BMP388_DR_Pin; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //普通输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
    GPIO_Init(BMP388_Port, &GPIO_InitStructure);//初始化片选引脚
    BMP388_Calibration(&BMP388_Calibration_Data,&BMP388_Calibration_QuantizedData);//获取校正系数
    //BMP388工作模式配置
    BMP388_SendData(0X7E,0XB6);
    BMP388_SendData(0X19,0X46);//配置中断引脚输出高电平
    BMP388_SendData(0X1C,0X03);//配置过采样
    BMP388_SendData(0X1D,0X02);//配置输出速率为50Hz
    BMP388_SendData(0X1F,0X02);//配置滤波设置
    delay_ms(50);
    BMP388_SendData(0X1B,0X33);//配置工作模式为正常工作
}

void BMP388_SendData(u8 addr,u8 data)
{
    BMP388_CmdRd;
    delay_us(1);
    BMP388_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,addr&0X7F);
    SPI_ReadWriteByte(SPI1,data);
    BMP388_CS = 1;
    delay_us(1);
}

u8 BMP388_ReadData(u8 addr)
{
    u8 res;
    BMP388_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,addr|0X80);
    SPI_ReadWriteByte(SPI1,0X00);
    res = SPI_ReadWriteByte(SPI1,0X00);
    BMP388_CS = 1;
    return res;
}

void BMP388_ReadBuffer(u8 addr,u8 *buffer,u8 length)
{
    BMP388_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,addr|0X80);
    SPI_ReadWriteByte(SPI1,0X00);
    for(;length>0;length--)
    {
        *buffer = SPI_ReadWriteByte(SPI1,0X00);
        buffer++;
    }
    BMP388_CS = 1;
}

u8 BMP388_StatusGet(void)
{
    return BMP388_ReadData(0X03);
}

void BMP388_PressureGet_char(u8 *pbuffer)
{
    BMP388_ReadBuffer(0X04,pbuffer,3);
    BMP388_ReadData(0X11);
}

void BMP388_TemperatureGet_char(u8 *tbuffer)
{
    BMP388_ReadBuffer(0X07,tbuffer,3);
    BMP388_ReadData(0X11);
}

double BMP388_TemperatureGet(void)
{
    uint8_t tem_buffer[3];
    int32_t uncomp_temp = 0;
    double partial_data1;
    double partial_data2;
    BMP388_PressureGet_char(tem_buffer);
    uncomp_temp = ((int32_t)tem_buffer[2]<<16)|((int32_t)tem_buffer[1]<<8)|((int32_t)tem_buffer[0]);
    partial_data1 = (double)(uncomp_temp - BMP388_Calibration_QuantizedData.par_t1);
    partial_data2 = (double)(partial_data1 * BMP388_Calibration_QuantizedData.par_t2);
    BMP388_Calibration_QuantizedData.t_lin = partial_data2 + (partial_data1 * partial_data1) * BMP388_Calibration_QuantizedData.par_t3;
    return BMP388_Calibration_QuantizedData.t_lin;
}

double BMP388_PressureGet(void)
{
    uint8_t pre_buffer[3];
    uint32_t uncomp_pre = 0;
    double partial_data1;
    double partial_data2;
    double partial_data3;
    double partial_data4;
    double partial_out1;
    double partial_out2;
    BMP388_TemperatureGet_char(pre_buffer);
    uncomp_pre = ((uint32_t)pre_buffer[2]<<16)|((uint32_t)pre_buffer[1]<<8)|pre_buffer[0];
    partial_data1 = BMP388_Calibration_QuantizedData.par_p6 * BMP388_Calibration_QuantizedData.t_lin;
    partial_data2 = BMP388_Calibration_QuantizedData.par_p7 * pow(BMP388_Calibration_QuantizedData.t_lin, 2);
    partial_data3 = BMP388_Calibration_QuantizedData.par_p8 * pow(BMP388_Calibration_QuantizedData.t_lin, 3);
    partial_out1 = BMP388_Calibration_QuantizedData.par_p5 + partial_data1 + partial_data2 + partial_data3;
    partial_data1 = BMP388_Calibration_QuantizedData.par_p2 * BMP388_Calibration_QuantizedData.t_lin;
    partial_data2 = BMP388_Calibration_QuantizedData.par_p3 * pow(BMP388_Calibration_QuantizedData.t_lin, 2);
    partial_data3 = BMP388_Calibration_QuantizedData.par_p4 * pow(BMP388_Calibration_QuantizedData.t_lin, 3);
    partial_out2 = uncomp_pre *(BMP388_Calibration_QuantizedData.par_p1 + partial_data1 + partial_data2 + partial_data3);
    partial_data1 = pow((double)uncomp_pre,2);
    partial_data2 = BMP388_Calibration_QuantizedData.par_p9 + BMP388_Calibration_QuantizedData.par_p10 * BMP388_Calibration_QuantizedData.t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + pow((double)uncomp_pre,3) * BMP388_Calibration_QuantizedData.par_p11;
    return partial_out1 + partial_out2 + partial_data4;
}

void BMP388_Calibration(BMP388_Calibration_Data_Struct *data_int,BMP388_Calibration_QuantizedData_Struct *data_float)
{
    uint8_t buffer[21];
    double temp_var;
    BMP388_ReadBuffer(0X31,buffer,21);
    data_int->par_t1 = (((uint16_t)buffer[1])<<8)|buffer[0];
    data_int->par_t2 = (((uint16_t)buffer[3])<<8)|buffer[2];
    data_int->par_t3 = buffer[4];
    data_int->par_p1 = (((int16_t)buffer[6])<<8)|buffer[5];
    data_int->par_p2 = (((int16_t)buffer[8])<<8)|buffer[7];
    data_int->par_p3 = buffer[9];
    data_int->par_p4 = buffer[10];
    data_int->par_p5 = (((uint16_t)buffer[12])<<8)|buffer[11];
    data_int->par_p6 = (((uint16_t)buffer[14])<<8)|buffer[13];
    data_int->par_p7 = buffer[15];
    data_int->par_p8 = buffer[16];
    data_int->par_p9 = (((int16_t)buffer[18])<<8)|buffer[17];
    data_int->par_p10 = buffer[19];
    data_int->par_p11 = buffer[20];
    
    temp_var = 0.00390625;
    data_float->par_t1 = ((double)data_int->par_t1 / temp_var);
    temp_var = 1073741824.0;
    data_float->par_t2 = ((double)data_int->par_t2 / temp_var);
    temp_var = 281474976710656.0;
    data_float->par_t3 = ((double)data_int->par_t3 / temp_var);
    temp_var = 1048576.0;
    data_float->par_p1 = ((double)(data_int->par_p1 - (16384)) / temp_var);
    temp_var = 536870912.0;
    data_float->par_p2 = ((double)(data_int->par_p2 - (16384)) / temp_var);
    temp_var = 4294967296.0;
    data_float->par_p3 = ((double)data_int->par_p3 / temp_var);
    temp_var = 137438953472.0;
    data_float->par_p4 = ((double)data_int->par_p4 / temp_var);
    temp_var = 0.125;
    data_float->par_p5 = ((double)data_int->par_p5 / temp_var);
    temp_var = 64.0;
    data_float->par_p6 = ((double)data_int->par_p6 / temp_var);
    temp_var = 256.0;
    data_float->par_p7 = ((double)data_int->par_p7 / temp_var);
    temp_var = 32768.0;
    data_float->par_p8 = ((double)data_int->par_p8 / temp_var);
    temp_var = 281474976710656.0;
    data_float->par_p9 = ((double)data_int->par_p9 / temp_var);
    temp_var = 281474976710656.0;
    data_float->par_p10 = ((double)data_int->par_p10 / temp_var);
    temp_var = 36893488147419103232.0;
    data_float->par_p11 = ((double)data_int->par_p11 / temp_var);
}

