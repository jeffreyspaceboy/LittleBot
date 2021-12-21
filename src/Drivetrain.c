/*---DRIVETRAIN_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       Drivetrain.c                                              */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-11                                                */
/*----------------------------------------------------------------------------*/

/* LOCAL INCLUDES */
#include "Drivetrain.h"

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
    motor_create_rpm_control_thread(new_drivetrain.left_motor);
    motor_create_rpm_control_thread(new_drivetrain.right_motor);
    return new_drivetrain;
}

int drivetrain_del(Drivetrain_t *drivetrain){
    motor_del(drivetrain->left_motor);
    motor_del(drivetrain->right_motor);
    return SUCCESS;
}

int drivetrain_spin(Drivetrain_t *drivetrain, int left_power, int right_power){
    return motor_spin(drivetrain->left_motor, left_power) || motor_spin(drivetrain->right_motor, right_power);
}

int drivetrain_set_rpm(Drivetrain_t *drivetrain, float left_rpm_target, float right_rpm_target){
    motor_set_rpm(drivetrain->left_motor, left_rpm_target);
    motor_set_rpm(drivetrain->right_motor, right_rpm_target);
    return SUCCESS;
}

// int drivetrain_pid_distance_spin(Drivetrain_t *drivetrain, PID_Controller_t *pid_distance, float distance_target, float tolerance){
//     if(!pid_distance->enabled){ pid_start(pid_distance, distance_target, tolerance); }
//     float current_rotations = (motor_get_rotations(drivetrain->left_motor) + motor_get_rotations(drivetrain->right_motor))/2.0F;
//     float rpm_target = pid_power(pid_distance, current_rotations);
//     if(fabs(pid_distance->error) > pid_distance->error_tolerance){
//         motor_set_rpm(drivetrain->left_motor, rpm_target);
//         motor_set_rpm(drivetrain->right_motor, rpm_target);
//     }
//     return SUCCESS;
// }

// int drivetrain_pid_velocity_spin(Drivetrain_t *drivetrain, float rpm){
//     PID_Controller_t pid_left = pid_init(400.0, 0.000000001, 550.0);
//     PID_Controller_t pid_right = pid_init(400.0, 0.000000001, 550.0);
//     float tolerance = 0.01;
//     int timesGood = 0, turn_bias = 0;
//     bool moveComplete = false;
//     float l_rpm = motor_get_rpm(drivetrain->left_motor);
//     float r_rpm = motor_get_rpm(drivetrain->right_motor);
//     pid_start(&pid_left, rpm, tolerance);
//     pid_start(&pid_right, rpm, tolerance);
//     while(!moveComplete && l_rpm <= rpm*2.0F && r_rpm <= rpm*2.0F){ 
//         l_rpm = motor_get_rpm(drivetrain->left_motor);
//         r_rpm = motor_get_rpm(drivetrain->right_motor);
//         printf("%s (%f | %f)\n", INFO_MSG, l_rpm, r_rpm);
//         if(l_rpm > r_rpm){
//             drivetrain_spin(drivetrain, (int)pid_power(&pid_left, l_rpm)-turn_bias, (int)pid_power(&pid_right, r_rpm));
//         }else if(r_rpm > l_rpm){
//             drivetrain_spin(drivetrain, (int)pid_power(&pid_left, l_rpm), (int)pid_power(&pid_right, r_rpm)-turn_bias);
//         }else{
//             drivetrain_spin(drivetrain, (int)pid_power(&pid_left, l_rpm), (int)pid_power(&pid_right, r_rpm));
//         }
        
//         if(fabs(pid_right.error)<=pid_right.error_tolerance && fabs(pid_left.error)<=pid_left.error_tolerance){ timesGood++; }
//         if(timesGood >= 1000){ moveComplete = true; }
//         gpioSleep(PI_TIME_RELATIVE, 0, 1000);
//     }
//     return drivetrain_spin(drivetrain, 0, 0);
// }

int drivetrain_stop(Drivetrain_t *drivetrain){
    return motor_stop(drivetrain->left_motor) || motor_stop(drivetrain->right_motor);
}
/*---DRIVETRAIN_C---*/