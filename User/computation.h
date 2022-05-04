#ifndef __COMPUTATION_H
#define __COMPUTATION_H

#include "math.h"
#include "bmi055.h"
#include "command.h"

#define PI 3.1415926

extern double dq[4],w[3],q[4],pitch,yaw,roll;

void AttitudeSolution(double q[],double w[]);//��̬�����㷨�����ٶȵ�λΪ��/s
double Pitch_Get(double q[]); //�����Ǽ��㹫ʽ
double Yaw_Get(double q[]);   //ƫ���Ǽ��㹫ʽ
double Roll_Get(double q[]);  //��ת�Ǽ��㹫ʽ  
double * AngularVelocity_Get(u8 *buffer); //���ٶȻ�ȡ����
double * Acceleration_Get(u8 *buffer); //���ٶȻ�ȡ����
  
#endif

