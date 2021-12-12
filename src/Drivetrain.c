/*---DRIVETRAIN_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       Drivetrain.c                                              */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-11                                                */
/*----------------------------------------------------------------------------*/

/* LOCAL INCLUDES */
#include "../include/Drivetrain.h"

/* NON-STANDARD INCLUDES */
#include <pigpio.h>

/* STANDARD INCLUDES */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

Drivetrain_t drivetrain_init(char drivetrain_name[NAME_MAX_SIZE], Motor_t *left_motor, Motor_t *right_motor){
    Drivetrain_t new_drivetrain = {
        .name = "",
        .left_motor = left_motor,
        .right_motor = right_motor,
    };
    strncpy(new_drivetrain.name, drivetrain_name, sizeof(new_drivetrain.name));
    return new_drivetrain;
}

int drivetrain_del(Drivetrain_t *drivetrain){
    return motor_del(drivetrain->left_motor) || motor_del(drivetrain->right_motor);  
}

int drivetrain_spin(Drivetrain_t *drivetrain, int left_power, int right_power){
    return motor_spin(drivetrain->left_motor, left_power) || motor_spin(drivetrain->right_motor, right_power);
}

int drivetrain_pid_distance_spin(Drivetrain_t *drivetrain, float distance){
    PID_Controller_t pid_left = pid_init(400.0, 0.00000001, 550.0);
    PID_Controller_t pid_right = pid_init(400.0, 0.00000001, 550.0);
    float tolerance = 0.08;
    int timesGood = 0, turn_bias = 50;
    bool moveComplete = false;
    float left_rotations = motor_get_rotations(drivetrain->left_motor);
    float right_rotations = motor_get_rotations(drivetrain->right_motor);
    pid_start(&pid_left, distance, tolerance);
    pid_start(&pid_right, distance, tolerance);
    while(!moveComplete && right_rotations <= distance*2.0F && left_rotations <= distance*2.0F){ 
        left_rotations = motor_get_rotations(drivetrain->left_motor);
        right_rotations = motor_get_rotations(drivetrain->right_motor);
        printf("%s (%f | %f)\n", INFO_MSG, left_rotations, right_rotations);
        if(left_rotations > right_rotations){
            drivetrain_spin(drivetrain, (int)pid_power(&pid_left, left_rotations)-turn_bias, (int)pid_power(&pid_right, right_rotations));
        }else if(right_rotations > left_rotations){
            drivetrain_spin(drivetrain, (int)pid_power(&pid_left, left_rotations), (int)pid_power(&pid_right, right_rotations)-turn_bias);
        }else{
            drivetrain_spin(drivetrain, (int)pid_power(&pid_left, left_rotations), (int)pid_power(&pid_right, right_rotations));
        }
        
        if(fabs(pid_right.error)<=pid_right.error_tolerance && fabs(pid_left.error)<=pid_left.error_tolerance){ timesGood++; }
        if(timesGood >= 1000){ moveComplete = true; }
        gpioSleep(PI_TIME_RELATIVE, 0, 1000);
    }
    return drivetrain_spin(drivetrain, 0, 0);
}

int drivetrain_pid_velocity_spin(Drivetrain_t *drivetrain, float rps){
    PID_Controller_t pid_left = pid_init(400.0, 0.000000001, 550.0);
    PID_Controller_t pid_right = pid_init(400.0, 0.000000001, 550.0);
    float tolerance = 0.01;
    int timesGood = 0, turn_bias = 0;
    bool moveComplete = false;
    float l_rps = motor_get_rps(drivetrain->left_motor);
    float r_rps = motor_get_rps(drivetrain->right_motor);
    pid_start(&pid_left, rps, tolerance);
    pid_start(&pid_right, rps, tolerance);
    while(!moveComplete && l_rps <= rps*2.0F && r_rps <= rps*2.0F){ 
        l_rps = motor_get_rps(drivetrain->left_motor);
        r_rps = motor_get_rps(drivetrain->right_motor);
        printf("%s (%f | %f)\n", INFO_MSG, l_rps, r_rps);
        if(l_rps > r_rps){
            drivetrain_spin(drivetrain, (int)pid_power(&pid_left, l_rps)-turn_bias, (int)pid_power(&pid_right, r_rps));
        }else if(r_rps > l_rps){
            drivetrain_spin(drivetrain, (int)pid_power(&pid_left, l_rps), (int)pid_power(&pid_right, r_rps)-turn_bias);
        }else{
            drivetrain_spin(drivetrain, (int)pid_power(&pid_left, l_rps), (int)pid_power(&pid_right, r_rps));
        }
        
        if(fabs(pid_right.error)<=pid_right.error_tolerance && fabs(pid_left.error)<=pid_left.error_tolerance){ timesGood++; }
        if(timesGood >= 1000){ moveComplete = true; }
        gpioSleep(PI_TIME_RELATIVE, 0, 1000);
    }
    return drivetrain_spin(drivetrain, 0, 0);
}

int drivetrain_stop(Drivetrain_t *drivetrain){
    return motor_stop(drivetrain->left_motor) || motor_stop(drivetrain->right_motor);
}
/*---DRIVETRAIN_C---*/