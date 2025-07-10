#include "pid.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->SetPoint = 0;
    pid->LastError = 0;
    pid->PrevError = 0;
}

void PID_SetPoint(PID_Controller *pid, float setpoint) {
    pid->SetPoint = setpoint;
}

float PID_IncCalc(PID_Controller *pid, float feedback_value) {
    float error = pid->SetPoint - feedback_value;
    float increment = pid->Kp * (error - pid->LastError) +
                      pid->Ki * error +
                      pid->Kd * (error - 2 * pid->LastError + pid->PrevError);
    pid->PrevError = pid->LastError;
    pid->LastError = error;
    return increment;
}/*
 * pid.c
 *
 *  Created on: 2025年7月6日
 *      Author: 20766
 */


