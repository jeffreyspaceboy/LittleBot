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
        .position = {0,0,0},
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

int drivetrain_stop(Drivetrain_t *drivetrain){
    return motor_stop(drivetrain->left_motor) || motor_stop(drivetrain->right_motor);
}

void drivetrain_track_position_thread(){
    // While enabled
    // heading = prev_heading + ()
    // delta_position = (rpm * wheel_c)*delta_time;
    // position += delta_position
}
/*---DRIVETRAIN_C---*/