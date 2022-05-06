#include "computation.h"

double dq[4],w[3],q[4],pitch,yaw,roll;
double T_11,T_12,T_13,T_21,T_22,T_23,T_31,T_32,T_33;//b->n×ª»»¾ØÕó
double acceleration_n[3],velocity_n[3],position_n[3];

void AttitudeSolution(double *q,double *w)
{
    double q_norm;
    w[0] = w[0]*PI/180;
    w[1] = w[1]*PI/180;
    w[2] = w[2]*PI/180;
    //±ê×¼¾ØÕó
//    dq[0] = 0.5*(-w[0]*q[1]-w[1]*q[2]-w[2]*q[3]);
//    dq[1] = 0.5*(w[0]*q[0]+w[2]*q[2]-w[1]*q[3]);
//    dq[2] = 0.5*(w[1]*q[0]-w[2]*q[1]+w[0]*q[3]);
//    dq[3] = 0.5*(w[2]*q[0]+w[1]*q[1]-w[0]*q[2]);
    //½ÇËÙ¶È±ä»»ºóµÄ¾ØÕó
    dq[0] = 0.5*(w[1]*q[1]-w[0]*q[2]-w[2]*q[3]);
    dq[1] = 0.5*(-w[1]*q[0]+w[2]*q[2]-w[0]*q[3]);
    dq[2] = 0.5*(w[0]*q[0]-w[2]*q[1]-w[1]*q[3]);
    dq[3] = 0.5*(w[2]*q[0]+w[0]*q[1]+w[1]*q[2]);
    q[0] = q[0] + dq[0]*dt;
    q[1] = q[1] + dq[1]*dt;
    q[2] = q[2] + dq[2]*dt;
    q[3] = q[3] + dq[3]*dt;
    q_norm = q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
    q_norm = sqrt(q_norm);
    q[0] = q[0]/q_norm;
    q[1] = q[1]/q_norm;
    q[2] = q[2]/q_norm;
    q[3] = q[3]/q_norm;
}

double Pitch_Get(double q[4]) 
{
    pitch = asin(2*(q[0]*q[1]+q[2]*q[3]));//³õÊ¼¾ØÕó¸©Ñö½Ç
//    pitch = asin(2*(q[0]*q[1]+q[2]*q[3]));//×ª»»¾ØÕóÆ«º½½Ç
    return pitch;
}

double Yaw_Get(double q[4])
{
    yaw = atan2(-2*(q[1]*q[2]-q[0]*q[3]),(pow(q[0],2)-pow(q[1],2)+pow(q[2],2)-pow(q[3],2)));//³õÊ¼¾ØÕóÆ«º½½Ç
//    yaw = atan2((pow(q[0],2)-pow(q[1],2)+pow(q[2],2)-pow(q[3],2)),2*(q[1]*q[2]-q[0]*q[3]));//×ª»»¾ØÕóÆ«º½½Ç
    return yaw;
}

double Roll_Get(double q[4])
{
    roll = atan2(-2*(q[1]*q[3]-q[0]*q[2]),pow(q[0],2)-pow(q[1],2)-pow(q[2],2)+pow(q[3],2));//³õÊ¼¾ØÕó¹ö×ª½Ç
//    roll = atan2(-2*(q[1]*q[3]-q[0]*q[2]),pow(q[0],2)-pow(q[1],2)-pow(q[2],2)+pow(q[3],2));//×ª»»¾ØÕóÆ«º½½Ç
    return roll;
}

void Acceleration_Get(u8 *buffer,double *acc)
{
    BMI055_ReadBuffer(ACC_Choose,0X02,buffer,6);
    for(n = 0;n<3;n++)
    {
        acc_16 = BMI055_DataTransform(ACC_Choose,buffer[2*n],buffer[2*n+1]);
        acc[n] = acc_16/4096.0*ACC_Range*acc_g-bmi055_offset[n];
    }
}

void AngularVelocity_Get(u8 *buffer,double *gyr)
{
    BMI055_ReadBuffer(GYR_Choose,0X02,buffer,6);
    for(n = 0;n<3;n++)
    {
        gyr_16 = BMI055_DataTransform(GYR_Choose,buffer[2*n],buffer[2*n+1]);
        gyr[n] = gyr_16/65536.0*GYR_Range-bmi055_offset[n+3];
    }
}

void AttitudeAngle_Init(double *Euler)
{
    Euler[0] = 0;
    Euler[1] = 0;
    Euler[2] = 0;
}

void AccelerationSolution(void)
{
    u8 buffer[6];
    double acc_b[3];
    double gyr[3];
    AngularVelocity_Get(buffer,gyr);
    Acceleration_Get(buffer,acc_b);
    AttitudeSolution(q,gyr);
    yaw = Yaw_Get(q);
    pitch = Pitch_Get(q);
    roll = Roll_Get(q);
    T_11 = cos(roll)*cos(yaw)-sin(roll)*sin(pitch)*sin(yaw);
    T_21 = cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw);
    T_31 = -sin(roll)*-cos(pitch);
    T_12 = -cos(pitch)*sin(yaw);
    T_22 = cos(pitch)*cos(yaw);
    T_32 = sin(pitch);
    T_13 = sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw);
    T_23 = sin(roll)*sin(yaw)-cos(roll)*sin(pitch)*cos(yaw);
    T_33 = cos(roll)*cos(pitch);
    acceleration_n[0] = -T_11*acc_b[1]+T_12*acc_b[0]+T_13*acc_b[2];
    acceleration_n[1] = -T_21*acc_b[1]+T_22*acc_b[0]+T_23*acc_b[2];
    acceleration_n[2] = -T_31*acc_b[1]+T_32*acc_b[0]+T_33*acc_b[2]-acc_g;
}

void VelociteySolution(void)
{
    AccelerationSolution();
    velocity_n[0] = velocity_n[0]+acceleration_n[0]*dt;
    velocity_n[1] = velocity_n[1]+acceleration_n[1]*dt;
    velocity_n[2] = velocity_n[2]+acceleration_n[2]*dt;
}

void PositionSolution(void)
{
    VelociteySolution();
    position_n[0] = position_n[0]+velocity_n[0]*dt;
    position_n[1] = position_n[1]+velocity_n[1]*dt;
    position_n[2] = velocity_n[2]+velocity_n[2]*dt;
}
