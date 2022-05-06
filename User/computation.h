#ifndef __COMPUTATION_H
#define __COMPUTATION_H

#include "math.h"
#include "bmi055.h"
#include "command.h"

#define PI 3.1415926

extern double dq[4],w[3],q[4],pitch,yaw,roll;
extern double T_11,T_12,T_13,T_21,T_22,T_23,T_31,T_32,T_33;////b->n��̬ת������
extern double acceleration_n[3],velocity_n[3],position_n[3];//��������ϵ�µ��ٶ�

void AttitudeSolution(double q[],double w[]);//��̬�����㷨�����ٶȵ�λΪ��/s
double Pitch_Get(double q[]); //�����Ǽ��㹫ʽ,����Ϊ����
double Yaw_Get(double q[]);   //ƫ���Ǽ��㹫ʽ,����Ϊ����
double Roll_Get(double q[]);  //��ת�Ǽ��㹫ʽ,����Ϊ����
void AngularVelocity_Get(u8 *buffer,double *gyr); //���ٶȻ�ȡ����
void Acceleration_Get(u8 *buffer,double *acc); //���ٶȻ�ȡ����
void AttitudeAngle_Init(double *Euler);//��ʼŷ����,�ֱ�Ϊyaw,pitch,roll 
void AccelerationSolution(void);//���ع�������ϵ�µļ��ٶ�
void VelociteySolution(void);//���ع�������ϵ�µ��ٶȣ���λΪm/s
void PositionSolution(void);//���ع�������ϵ�µ�λ�ã���λΪm
#endif

