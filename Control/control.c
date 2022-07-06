#include "control.h"

PID_Coefficient PITCH_Coefficient = {PITCH_P,PITCH_I,PITCH_D,0,0,0,0};
PID_Coefficient ROLL_Coefficient = {ROLL_P,ROLL_I,ROLL_D,0,0,0,0};

u8 Control_Flag = 0;
double roll_init;
uint32_t fuse_counter;

double PID_Output(PID_Coefficient *PID,double actuality,double expectation)
{
    double output;
    PID->bias = expectation - actuality;
    PID->bias_i = PID->bias_i + PID->bias;
    PID->bias_d = gyr[0];
    PID->bias_pre = PID->bias;
    output = PID->P*PID->bias + PID->I*PID->bias_i + PID->D*PID->bias_d;
    return output;
}

void PitchChannel_Output(double PID,double *serve)
{
    serve[0] = PID/4.0;
    serve[1] = PID/4.0;
    serve[2] = PID/4.0;
    serve[3] = PID/4.0;
}

void RollChannel_Output(double PID,double *serve)
{
    serve[0] = PID/4.0;
    serve[1] = PID/4.0;
    serve[2] = PID/4.0;
    serve[3] = PID/4.0;
}

