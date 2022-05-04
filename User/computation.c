#include "computation.h"

double dq[4],w[3],q[4],pitch,yaw,roll;

void AttitudeSolution(double *q,double *w)
{
    w[0] = w[0]*PI/180;
    w[1] = w[1]*PI/180;
    w[2] = w[2]*PI/180;
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
//    yaw = atan2((pow(q[0],2)-pow(q[1],2)+pow(q[2],2)-pow(q[3],2)),2*(q[1]*q[2]-q[0]*q[3]));
    yaw = atan2(-2*(q[1]*q[2]-q[0]*q[3]),(pow(q[0],2)-pow(q[1],2)-pow(q[2],2)+pow(q[3],2)));
    return yaw;
}

double Roll_Get(double q[4])
{
//    roll = atan2(-2*(q[1]*q[3]+q[0]*q[2]),pow(q[0],2)-pow(q[1],2)-pow(q[2],2)+pow(q[3],2));
    roll = atan2(-2*(q[1]*q[3]-q[0]*q[2]),(pow(q[0],2)-pow(q[1],2)+pow(q[2],2)-pow(q[3],2)));
    return roll;
}

double * Acceleration_Get(u8 *buffer)
{
    static double acc[3];
    BMI055_ReadBuffer(ACC_Choose,0X02,buffer,6);
    for(n = 0;n<3;n++)
    {
        acc_16 = BMI055_DataTransform(ACC_Choose,buffer[2*n],buffer[2*n+1]);
        acc[n] = acc_16/4096.0*ACC_Range*acc_g-bmi055_offset[n];
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
        gyr[n] = gyr_16/65536.0*GYR_Range-bmi055_offset[n+3];
    }
    return gyr;
}




