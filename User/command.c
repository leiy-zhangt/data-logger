#include "command.h"

void Command_Execute(USART_TypeDef* USARTx)
{
    u8 command[2];
    if(USARTx == USART1) {command[0]=USART1_RX_BUF[0];command[1]=USART1_RX_BUF[1];}
    else if(USARTx == USART2) {command[0]=USART2_RX_BUF[0];command[1]=USART2_RX_BUF[1];}
    else if(USARTx == USART3) {command[0]=USART3_RX_BUF[0];command[1]=USART3_RX_BUF[1];}
    if(command[0]=='C'&&command[1]=='E') Commanad_ChipErase();
    else if(command[0]=='B'&&command[1]=='E') Command_Bmi055StartWork();
    else if(command[0]=='B'&&command[1]=='D') Command_Bmi055StopWork();
    else if(command[0]=='D'&&command[1]=='R') Command_DataOutput();
    else printf("Command is error!\r\n");  
}

void Commanad_ChipErase(void)
{
    W25N_ChipErase();
    printf("W25N has been erased\r\n");
}

void Command_Bmi055StartWork(void)
{
    printf("BMI055 is working!\r\n");
    location = 0;
    data_number = 0;
    page = Start_Page;
    BMI_ReadCmd(ENABLE);
}

void Command_Bmi055StopWork(void)
{
    BMI_ReadCmd(DISABLE);
    W25N_DataWrirte(bmi_buffer,page);
    final_number = data_number;
    bmi_buffer[0]=final_number>>24;
    bmi_buffer[1]=final_number>>16;
    bmi_buffer[2]=final_number>>8;
    bmi_buffer[3]=final_number;
    W25N_DataWrirte(bmi_buffer,0X0000);
    printf("BMI055 stops working!%u points has been stored!\r\n",data_number);
}

void Command_DataOutput(void)
{
    data_number = 0;
    page = Start_Page;
    final_number = 0;
    W25N_DataReceive(bmi_buffer,0X0000);
    final_number|=(bmi_buffer[0]<<24);
    final_number|=(bmi_buffer[1]<<16);
    final_number|=(bmi_buffer[2]<<8);
    final_number|=bmi_buffer[3];
    if(final_number != 0XFFFFFFFF){
        printf("acc_x   acc_y   acc_z   gry_x   gry_y   gyr_z\r\n");
        while(1)
        {
            W25N_DataReceive(bmi_buffer,page++);
            for(location=0;location<128;location++)
            {
                u8 *bmi_data = bmi_buffer+2+16*location;
                if(data_number>=final_number) break;
                printf("%d:  ",data_number);
                data_number++;
                for(n=0;n<6;n++)
                {
                    if(n<3)
                    {
                        acc_16 = BMI055_DataTransform(ACC_Choose,bmi_data[2*n],bmi_data[2*n+1]);
                        acc = acc_16/4096.00*8*9.8;
                        printf("%+0.4f  ",acc);
                    }
                    else
                    {
                        gyr_16 = BMI055_DataTransform(GYR_Choose,bmi_data[2*n],bmi_data[2*n+1]);
                        gyr = gyr_16/65536.00*250;
                        printf("%+0.4f  ",gyr);
                        if(n==5) printf("\r\n");
                    }
                }
            }
            if(data_number>=final_number) break;
        }
    }
    else printf("FLASH is empty\r\n");
}