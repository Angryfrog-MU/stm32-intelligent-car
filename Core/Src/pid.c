#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "pid.h"

// Initialize PID controller
void PID_Init(PID* pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0;
    pid->pre_error = 0.0;
    pid->output = 0.0;
}

// Update PID controller
float PID_Update(PID* pid, float setpoint, float pv, float dt) {
    // Calculate error
    float error = setpoint - pv;

    // Proportional term
    float Pout = pid->Kp * error;

    // Integral term
    pid->integral += error * dt;
    float Iout = pid->Ki * pid->integral;

    // Derivative term
    float derivative = (error - pid->pre_error) / dt;
    float Dout = pid->Kd * derivative;

    // Total output
    pid->output = Pout + Iout + Dout;

    // Save error to previous error
    pid->pre_error = error;

    return pid->output;
}
