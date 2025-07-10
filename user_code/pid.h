/*
 * pid.h
 *
 *  Created on: 2025年7月6日
 *      Author: 20766
 */

#ifndef USER_CODE_PID_H_
#define USER_CODE_PID_H_

typedef struct {
    float Kp, Ki, Kd;
    float SetPoint;
    float LastError;
    float PrevError;
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd);
void PID_SetPoint(PID_Controller *pid, float setpoint);
float PID_IncCalc(PID_Controller *pid, float feedback_value);


#endif /* USER_CODE_PID_H_ */
