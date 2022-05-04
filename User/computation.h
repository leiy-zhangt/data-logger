#ifndef __COMPUTATION_H
#define __COMPUTATION_H

#include "math.h"
#include "bmi055.h"
#include "command.h"

#define PI 3.1415926

extern double dq[4],w[3],q[4],pitch,yaw,roll;

void AttitudeSolution(double q[],double w[]);//姿态解算算法，角速度单位为°/s
double Pitch_Get(double q[]); //俯仰角计算公式
double Yaw_Get(double q[]);   //偏航角计算公式
double Roll_Get(double q[]);  //滚转角计算公式  
double * AngularVelocity_Get(u8 *buffer); //角速度获取函数
double * Acceleration_Get(u8 *buffer); //加速度获取函数
  
#endif

