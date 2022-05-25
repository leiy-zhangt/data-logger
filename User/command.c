#include "command.h"

u8 Command_Flag=0;

void Command_Execute(USART_TypeDef* USARTx)
{
    u8 command[2];
    if(USARTx == USART1) {command[0]=USART1_RX_BUF[0];command[1]=USART1_RX_BUF[1];}
    else if(USARTx == USART2) {command[0]=USART2_RX_BUF[0];command[1]=USART2_RX_BUF[1];}
    else if(USARTx == USART3) {command[0]=USART3_RX_BUF[0];command[1]=USART3_RX_BUF[1];}
    if(command[0]=='C'&&command[1]=='E') Commanad_ChipErase();
    else if(command[0]=='B'&&command[1]=='E') {Command_Flag = 0; Command_Bmi055StartWork();}
    else if(command[0]=='B'&&command[1]=='D') Command_Bmi055StopWork();
    else if(command[0]=='D'&&command[1]=='R') Command_DataOutput();
    else if(command[0]=='S'&&command[1]=='C') Command_StatusCheck();
    else if(command[0]=='B'&&command[1]=='O') Command_BMI055_OFFSET();
    else if(command[0]=='D'&&command[1]=='D') {Command_Flag = 1; Command_Bmi055StartWork();}
    else if(command[0]=='A'&&command[1]=='S') {Command_Flag = 2; Command_Bmi055StartWork();}
    else if(command[0]=='Q'&&command[1]=='I') Command_Q_Init(q);
    else if(command[0]=='A'&&command[1]=='D') {Command_Flag = 3; Command_Bmi055StartWork();}
    else if(command[0]=='V'&&command[1]=='D') {Command_Flag = 4; Command_Bmi055StartWork();}
    else if(command[0]=='P'&&command[1]=='D') {Command_Flag = 5; Command_Bmi055StartWork();}
    else printf("Command is error!\r\n");  
}

void Commanad_ChipErase(void)
{
    W25N_ChipErase();
    printf("W25N has been erased\r\n");
}

void Command_Bmi055StartWork(void)
{
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
    velocity_n[0] = 0;
    velocity_n[1] = 0;
    velocity_n[2] = 0;
    position_n[0] = 0;
    position_n[1] = 0;
    position_n[2] = 0;
    printf("BMI055 is working!\r\n");
    location = 0;
    data_number = 0;
    page = Start_Page;
    BMI_ReadCmd(ENABLE);
}

void Command_Bmi055StopWork(void)
{
    BMI_ReadCmd(DISABLE);
    TIM_SetCounter(TIM4,0X00);
    delay_us(5);
    printf("BMI055 stops working!");
    if(Command_Flag == 0)
    {
        W25N_DataWrirte(bmi_buffer,page);
        final_number = data_number;
        bmi_buffer[0]=final_number>>24;
        bmi_buffer[1]=final_number>>16;
        bmi_buffer[2]=final_number>>8;
        bmi_buffer[3]=final_number;
        W25N_DataWrirte(bmi_buffer,0X0000);
        printf("%u points has been stored!\r\n",data_number);
    }
}

void Command_StatusCheck(void)
{
    printf("Data logger has ready!\r\n");
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
                u8 *bmi_data = bmi_buffer+4+16*location;
                if(data_number>=final_number) break;
                printf("%d:  ",data_number);
                data_number++;
                for(n=0;n<6;n++)
                {
                    if(n<3)
                    {
                        acc_16 = BMI055_DataTransform(ACC_Choose,bmi_data[2*n],bmi_data[2*n+1]);
                        acc = acc_16/4096.00*ACC_Range*acc_g;
                        printf("%+0.4f  ",acc);
                    }
                    else
                    {
                        gyr_16 = BMI055_DataTransform(GYR_Choose,bmi_data[2*n],bmi_data[2*n+1]);
                        gyr = gyr_16/65536.00*GYR_Range;
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

void Command_BMI055_DataStorage(void)
{
    bmi_buffer[16*location]=data_number>>24;
    bmi_buffer[16*location+1]=data_number>>16;
    bmi_buffer[16*location+2]=data_number>>8;
    bmi_buffer[16*location+3]=data_number;
    data_number++;
    BMI055_ReadBuffer(ACC_Choose,0X02,bmi_buffer+4+16*location,6);
    delay_us(3);
    BMI055_ReadBuffer(GYR_Choose,0X02,bmi_buffer+10+16*location,6);
    location++;
    if(location==128)
    {
        location = 0;
        W25N_DataWrirte(bmi_buffer,page);
        page++;
        if(page == page_max)
        {
            Command_Bmi055StopWork();
            printf("FLASH is full!\r\n");
        }
    }
}

void Command_BMI055_DataDisplay(void)
{
    u8 buffer[6];
    double acc[3]; 
    double gyr[3];
    AngularVelocity_Get(buffer,gyr); 
    Acceleration_Get(buffer,acc);
    printf("acc: %+0.4f  %+0.4f  %+0.4f  ,gyr: %+0.4f  %+0.4f  %+0.4f  \r\n",acc[0],acc[1],acc[2],gyr[0],gyr[1],gyr[2]);
//    printf("%+0.4f  %+0.4f  %+0.4f  %+0.4f  %+0.4f  %+0.4f  \r\n",acc[0],acc[1],acc[2],gyr[0],gyr[1],gyr[2]);
}

void Command_AttitudeSolution(void)
{
    u8 buffer[6];
    double gyr[3];
    AngularVelocity_Get(buffer,gyr);
    AttitudeSolution(q,gyr);
    pitch = Pitch_Get(q)*180/PI;
    yaw = Yaw_Get(q)*180/PI;
    roll = Roll_Get(q)*180/PI;
    printf("pitch: %+0.4f  yaw: %+0.4f  roll: %+0.4f\r\n",pitch,yaw,roll);    
}

void Command_BMI055_OFFSET(void)
{
    uint16_t n;
    u8 buffer[6];
    double acc_offset[3],gyr_offset[3],offset[6];
    printf("BMI055 offset is starting!\r\n");
    for(n=0;n<6;n++)
    {
        bmi055_offset[n] = 0;
        offset[n] = 0;
    }
    for(n=0;n<500;n++)
    {
        Acceleration_Get(buffer,acc_offset);
        AngularVelocity_Get(buffer,gyr_offset);
        offset[0] = offset[0]+acc_offset[0];
        offset[1] = offset[1]+acc_offset[1];
        offset[2] = offset[2]+acc_offset[2];
        offset[3] = offset[3]+gyr_offset[0];
        offset[4] = offset[4]+gyr_offset[1];
        offset[5] = offset[5]+gyr_offset[2];
        delay_ms(20);
    }
    bmi055_offset[0] = offset[0]/500.0;
    bmi055_offset[1] = offset[1]/500.0;
    bmi055_offset[2] = offset[2]/500.0-acc_g;
    bmi055_offset[3] = offset[3]/500.0;
    bmi055_offset[4] = offset[4]/500.0;
    bmi055_offset[5] = offset[5]/500.0;
    printf("BMI055 offset has finished!\r\n");
}

void Command_Q_Init(double *q)
{
    double Euler[3];
    AttitudeAngle_Init(Euler);
    yaw = Euler[0]*PI/180;
    pitch = Euler[1]*PI/180;
    roll = Euler[2]*PI/180;
    T_11 = cos(roll)*cos(yaw)-sin(roll)*sin(pitch)*sin(yaw);
    T_21 = cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw);
    T_31 = -sin(roll)*-cos(pitch);
    T_12 = -cos(pitch)*sin(yaw);
    T_22 = cos(pitch)*cos(yaw);
    T_32 = sin(pitch);
    T_13 = sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw);
    T_23 = sin(roll)*sin(yaw)-cos(roll)*sin(pitch)*cos(yaw);
    T_33 = cos(roll)*cos(pitch);
    q[0] = 0.5*sqrt(1+T_11+T_22+T_33);
    q[1] = 0.5*sqrt(1+T_11-T_22-T_33);
    q[2] = 0.5*sqrt(1-T_11+T_22-T_33);
    q[3] = 0.5*sqrt(1-T_11-T_22+T_33);
    if((T_32 - T_23)<0) q[1] = -q[1];
    if((T_13 - T_31)<0) q[2] = -q[2];
    if((T_21 - T_12)<0) q[3] = -q[3];
    printf("Q initialization has finished!\r\n");
}

void Command_AccelerationDisplay(void)
{
    AccelerationSolution();
    printf("acc_x: %+0.4f   acc_y: %+0.4f   acc_z: %+0.4f\r\n",acceleration_n[0],acceleration_n[1],acceleration_n[2]);
//    printf("%+0.4f   %+0.4f   %+0.4f\r\n",acceleration_n[0],acceleration_n[1],acceleration_n[2]);
}

void Command_VelocityDisplay(void)
{
    VelociteySolution();
    printf("velocity_x: %+0.4f   velocity_y: %+0.4f   velocity_z: %+0.4f\r\n",velocity_n[0],velocity_n[1],velocity_n[2]);
}

void Command_PositionDisplay(void)
{
    PositionSolution();
    printf("position_x: %+0.4f   position_y: %+0.4f   position_z: %+0.4f\r\n",position_n[0],position_n[1],position_n[2]);
}

