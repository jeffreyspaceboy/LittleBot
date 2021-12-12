/*---PID_H---*/
#ifndef PID_H
#define PID_H
/*----------------------------------------------------------------------------*/
/*    Module:       PID.h                                                     */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-11                                                */
/*----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

/* LOCAL INCLUDES */
#include "Definitions.h"

/* STANDARD INCLUDES */
#include <stdint.h>
#include <stdbool.h>


/** @brief PID_CONTROLLER TYPE - used for PID control of motors, or other feedback and control.
 * @param kp Proportional Gain Constant.
 * @param ki Integral Gain Constant.
 * @param kd Derivative Gain Constant.
 * @param target The value the PID controller is trying to reach.
 * @param error_tolerance Used externally to check if current is within the bounds of the target.
 * @param error (= target - current) Difference between your target, and your current position.
 * @param prev_error The error observed the last time the pid function was run.
 * @param error_integral (+= error * dt) The integral of error. Or the sum of all error since the loop was started.
 * @param dedt (= (error - prev_error) / dt) The derivative of error. Or the change in error over time.
 * @param dt (= current_time - prev_time) The change in time between the current loop and the previous one.
 * @param prev_time The time observed the last time the pid function was run.
 */
typedef struct PID_Controller_t{
    float kp, ki, kd;
    float target, error_tolerance, error, prev_error, error_integral, dedt, dt;
    long prev_time;
    bool enabled;
} PID_Controller_t;

/** @brief PID Initialization
 * @param kP Proportional Gain Constant
 * @param kI Integral Gain Constant
 * @param kD Derivative Gain Constant
 * @return PID_Controller */
PID_Controller_t pid_init(float kP, float kI, float kD);

/** @brief Initializes your target and gets controller ready for use. Run this right before your PID loop.
 * @param pid PID Controller to be used
 * @param target The value the controller is trying to reach
 * @param error_tolerance The tolerance for error used externally for this PID controller
 * @return int SUCCESS or FAILURE */
int pid_start(PID_Controller_t *pid, float target, float error_tolerance);

/** @brief Takes target and current values as inputs and outputs a power based on tuned PID gains.
 * @param pid PID Controller to be used
 * @param target The value the controller is trying to reach
 * @param current The value the system currently has
 * @return float - output power of the PID function */
float pid_power(PID_Controller_t *pid, float current);


#ifdef __cplusplus
}
#endif
#endif
/*---PID_H---*/