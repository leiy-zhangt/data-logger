#ifndef __CONTROL_H
#define __CONTROL_H

#include "sys.h"
#include "command.h"

#define PITCH_P 3
#define PITCH_I 0
#define PITCH_D 0
#define ROLL_P 3
#define ROLL_I 0
#define ROLL_D 0

typedef struct
{
    double P;
    double I;
    double D;
    double bias;
    double bias_i;
    double bias_d;
    double bias_pre;
}PID_Coefficient;

extern PID_Coefficient PITCH_Coefficient;
extern PID_Coefficient ROLL_Coefficient;

extern u8 Control_Flag;

double PID_Output(PID_Coefficient *PID,double actuality,double expectation);
void PitchChannel_Output(double PID,double *serve);
void RollChannel_Output(double PID,double *serve);
    
#endif
