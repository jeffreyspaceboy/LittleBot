/*---PID_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       PID.c                                                     */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-09                                                */
/*----------------------------------------------------------------------------*/
#include "../include/PID.h"

#include <stdlib.h>

PID_Controller pid_init(float P, float I, float D, float error_tolerance){
    //Example Constants: kp = 20.00 | kd = 1.55 | ki = 0.05
    PID_Controller new_pid = {
        .kp = P,
        .ki = I,
        .kd = D,
        .prev_error = 0.0,
        .error_integral = 0.0,
        .error_tolerance = error_tolerance,
        .prev_time = 0.0,
    };
    return new_pid;
}

float pid_power(PID_Controller *pid, float target, float current, uint32_t current_time){
    float dt, error, dedt;
    dt = (float)(current_time - pid->prev_time); //Time Delta
    pid->prev_time = current_time;
    error = target - current; //Error
    pid->error_integral += error*dt; //Integral
    dedt = (error - pid->prev_error)/dt; //Derivative  
    pid->prev_error = error;
    //if(abs(error) < pid->error_tolerance){ return 0; } //At Target
    return (pid->kp*error) + (pid->kd*dedt) + (pid->ki * pid->error_integral); //Control Signal
}
/*---PID_C---*/