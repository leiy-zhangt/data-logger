#ifndef __COMPUTATION_H
#define __COMPUTATION_H

#include "math.h"
#include "bmi055.h"
#include "command.h"

#define PI 3.1415926

extern double dq[4],w[3],q[4],pitch,yaw,roll;
extern double T_11,T_12,T_13,T_21,T_22,T_23,T_31,T_32,T_33;////b->n姿态转换矩阵
extern double acceleration_n[3],velocity_n[3],position_n[3];//惯性坐标系下的速度

void AttitudeSolution(double q[],double w[]);//姿态解算算法，角速度单位为°/s
double Pitch_Get(double q[]); //俯仰角计算公式,返回为弧度
double Yaw_Get(double q[]);   //偏航角计算公式,返回为弧度
double Roll_Get(double q[]);  //滚转角计算公式,返回为弧度
void AngularVelocity_Get(u8 *buffer,double *gyr); //角速度获取函数
void Acceleration_Get(u8 *buffer,double *acc); //加速度获取函数
void AttitudeAngle_Init(double *Euler);//初始欧拉角,分别为yaw,pitch,roll 
void AccelerationSolution(void);//返回惯性坐标系下的加速度
void VelociteySolution(void);//返回惯性坐标系下的速度，单位为m/s
void PositionSolution(void);//返回惯性坐标系下的位置，单位为m
#endif

