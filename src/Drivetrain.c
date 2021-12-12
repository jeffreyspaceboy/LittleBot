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

int drivetrain_stop(Drivetrain_t *drivetrain){
    return motor_stop(drivetrain->left_motor) || motor_stop(drivetrain->right_motor);
}
/*---DRIVETRAIN_C---*/