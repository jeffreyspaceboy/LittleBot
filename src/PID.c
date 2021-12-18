/*---PID_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       PID.c                                                     */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-11                                                */
/*----------------------------------------------------------------------------*/

/* LOCAL INCLUDES */
#include "PID.h"

/* NON-STANDARD INCLUDES */
#include <pigpio.h>

/* STANDARD INCLUDES */
#include <stdlib.h>
#include <stdio.h>


PID_Controller_t pid_init(float kP, float kI, float kD){
    //Example Constants: kp = 20.00 | kd = 1.55 | ki = 0.05
    PID_Controller_t new_pid = {
        .kp = kP,
        .ki = kI,
        .kd = kD,
        .error = 0.0,
        .prev_error = 0.0,
        .error_integral = 0.0,
        .dedt = 0.0,
        .dt = 0.0,
        .prev_time = 0,
        .enabled = false,
    };
    return new_pid;
}

int pid_start(PID_Controller_t *pid, float target, float error_tolerance){
    pid->error_tolerance = error_tolerance;
    pid->target = target;
    pid->prev_error = target;
    pid->error_integral = 0.0;
    pid->prev_time = gpioTick();
    pid->enabled = true;
    return SUCCESS;
}

float pid_power(PID_Controller_t *pid, float current){
    if(!pid->enabled){ printf("%s You should run pid_start() before calling this function.\n", WARNING_MSG); }
    uint32_t current_time = gpioTick();
    pid->dt = (float)(current_time - pid->prev_time); //Time Delta
    pid->prev_time = current_time;
    pid->error = pid->target - current; //Error
    pid->error_integral += pid->error * pid->dt; //Integral
    pid->dedt = (pid->error - pid->prev_error) / pid->dt; //Derivative  
    pid->prev_error = pid->error;
    return (pid->kp * pid->error) + (pid->ki * pid->error_integral) + (pid->kd * pid->dedt); //Control Signal
}
/*---PID_C---*/