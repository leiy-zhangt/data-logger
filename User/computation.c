#include "computation.h"

double dq[4],w[3],q[4],pitch,yaw,roll;

void AttitudeSolution(double *q,double *w)
{
    dq[0] = 0.5f*(-w[0]*q[1]-w[1]*q[2]-w[2]*q[3]);
    dq[1] = 0.5f*(w[0]*q[0]+w[2]*q[2]-w[1]*q[3]);
    dq[2] = 0.5f*(w[1]*q[0]-w[2]*q[1]+w[0]*q[3]);
    dq[3] = 0.5f*(w[2]*q[0]+w[1]*q[1]-w[0]*q[2]);
    q[0] = q[0] + dq[0]*dt;
    q[1] = q[1] + dq[1]*dt;
    q[2] = q[2] + dq[2]*dt;
    q[3] = q[3] + dq[3]*dt;
}

double Pitch_Get(double q[4]) 
{
//    pitch = atan2(-(pow(q[0],2)-pow(q[1],2)+pow(q[2],2)-pow(q[3],2)),2*(q[1]*q[2]+q[0]*q[3]));
    pitch = asin(2*(q[2]*q[3]+q[0]*q[1]));
    return pitch;
}

double Yaw_Get(double q[4])
{
    yaw = atan2(-(pow(q[0],2)-pow(q[1],2)+pow(q[2],2)-pow(q[3],2)),2*(q[1]*q[2]-q[0]*q[3]));
    return yaw;
}

double Roll_Get(double q[4])
{
//    roll = atan2(-2*(q[1]*q[3]+q[0]*q[2]),pow(q[0],2)-pow(q[1],2)-pow(q[2],2)+pow(q[3],2));
    roll = atan2(-2*(q[1]*q[3]-q[0]*q[2]),(pow(q[0],2)-pow(q[1],2)-pow(q[2],2)+pow(q[3],2)));
    return roll;
}

double * Acceleration_Get(u8 *buffer)
{
    static double acc[3];
    BMI055_ReadBuffer(ACC_Choose,0X02,buffer,6);
    for(n = 0;n<3;n++)
    {
        acc_16 = BMI055_DataTransform(ACC_Choose,buffer[2*n],buffer[2*n+1]);
        acc[n] = acc_16/4096.0*ACC_Range*acc_g;
    }
    return acc;
}

double * AngularVelocity_Get(u8 *buffer)
{
    static double gyr[3];
    BMI055_ReadBuffer(GYR_Choose,0X02,buffer,6);
    for(n = 0;n<3;n++)
    {
        gyr_16 = BMI055_DataTransform(GYR_Choose,buffer[2*n],buffer[2*n+1]);
        gyr[n] = gyr_16/65536.0*GYR_Range;
    }
    return gyr;
}

void BMI055_DataStorage(void)
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

void BMI055_DataDisplay(void)
{
    double *acc = Acceleration_Get(bmi_buffer);
    double *gyr = AngularVelocity_Get(bmi_buffer+6);
    printf("acc:%+0.4f  %+0.4f  %+0.4f  ,gyr:%+0.4f  %+0.4f  %+0.4f  \r\n",acc[0],acc[1],acc[2],gyr[0],gyr[1],gyr[2]);
}
