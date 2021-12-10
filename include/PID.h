/*---PID_H---*/
#ifndef PID_H
#define PID_H
/*----------------------------------------------------------------------------*/
/*    Module:       PID.h                                                     */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-09                                                */
/*----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#include "Definitions.h"

#include <stdint.h>

typedef struct{
    float kp, ki, kd, error_tolerance;
    float error, prev_error, error_integral, dedt, dt;
    long prev_time;
} PID_Controller;

PID_Controller pid_init(float P, float I, float D, float error_tolerance);

float pid_power(PID_Controller *pid, float target, float current, uint32_t current_time);

#ifdef __cplusplus
}
#endif
#endif
/*---PID_H---*/