#ifndef __BMP388_H
#define __BMP388_H

#include "sys.h"
#include "delay.h"
#include "spi.h"
#include "math.h"

#define BMP388_CS PCout(4)
#define BMP388_DR PCin(5)
#define BMP388_Port GPIOC
#define BMP388_CLK RCC_AHB1Periph_GPIOC
#define BMP388_CS_Pin GPIO_Pin_4
#define BMP388_DR_Pin GPIO_Pin_5
#define BMP388_CmdRd while((BMP388_ReadData(0X03)&0X10)==0)
    
typedef struct 
{
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
}BMP388_Calibration_Data_Struct;

typedef struct 
{
    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double t_lin;
}BMP388_Calibration_QuantizedData_Struct;

extern BMP388_Calibration_Data_Struct BMP388_Calibration_Data;
extern BMP388_Calibration_QuantizedData_Struct BMP388_Calibration_QuantizedData;


void BMP388_Configuration(void);//初始化函数
void BMP388_SendData(u8 addr,u8 data);//BMP388发送数据函数
u8 BMP388_ReadData(u8 addr);//BMP388读数据函数
void BMP388_ReadBuffer(u8 addr,u8 *buffer,u8 length);
u8 BMP388_StatusGet(void);//BMP388状态获取函数
void BMP388_PressureGet_char(u8 *pbuffer);//BMP388气压值获取（char型）
void BMP388_TemperatureGet_char(u8 *tbuffer);//BMP388温度值获取（char型）
void BMP388_Calibration(BMP388_Calibration_Data_Struct *data_int,BMP388_Calibration_QuantizedData_Struct *data_float);//BMP388校正系数获取
double BMP388_TemperatureGet(void);//BMP388温度值获取（double型）
double BMP388_PressureGet(void);//BMP388气压值获取（double型）
#endif

