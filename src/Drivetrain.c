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

Drivetrain drivetrain_init(char drivetrain_name[NAME_MAX_SIZE], Motor *left_motor, Motor *right_motor){
    Drivetrain new_drivetrain = {
        .name = "",
        .left_motor = left_motor,
        .right_motor = right_motor,
    };
    strncpy(new_drivetrain.name, drivetrain_name, sizeof(new_drivetrain.name));
    return new_drivetrain;
}

int drivetrain_del(Drivetrain *drivetrain){
    return motor_del(drivetrain->left_motor) || motor_del(drivetrain->right_motor);  
}

int drivetrain_spin(Drivetrain *drivetrain, int left_power, int right_power){
    return motor_spin(drivetrain->left_motor, left_power) || motor_spin(drivetrain->right_motor, right_power);
}

int drivetrain_stop(Drivetrain *drivetrain){
    return motor_stop(drivetrain->left_motor) || motor_stop(drivetrain->right_motor);
}
/*---DRIVETRAIN_C---*/