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
        .error_tolerance = error_tolerance,
        .error = 0.0,
        .prev_error = 0.0,
        .error_integral = 0.0,
        .dedt = 0.0,
        .dt = 0.0,
        .prev_time = 0,
    };
    return new_pid;
}

float pid_power(PID_Controller *pid, float target, float current, uint32_t current_time){
    pid->dt = (float)(current_time - pid->prev_time); //Time Delta
    pid->prev_time = current_time;
    pid->error = target - current; //Error
    pid->error_integral += pid->error * pid->dt; //Integral
    pid->dedt = (pid->error - pid->prev_error) / pid->dt; //Derivative  
    pid->prev_error = pid->error;
    return (pid->kp * pid->error) + (pid->ki * pid->error_integral) + (pid->kd * pid->dedt); //Control Signal
}
/*---PID_C---*/