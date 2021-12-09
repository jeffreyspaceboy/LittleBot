/*---DRIVETRAIN_H---*/
#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H
/*----------------------------------------------------------------------------*/
/*    Module:       Drivetrain.h                                              */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-08                                                */
/*----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#include "Definitions.h"
#include "Motor.h"
#include "Encoder.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct{
    char name[NAME_MAX_SIZE];
    Motor *left_motor, *right_motor;
} Drivetrain;

Drivetrain drivetrain_init(char drivetrain_name[NAME_MAX_SIZE], Motor *left_motor, Motor *right_motor);
int drivetrain_del(Drivetrain *drivetrain);

int drivetrain_spin(Drivetrain *drivetrain, int left_power, int right_power);
int drivetrain_stop(Drivetrain *drivetrain);

#ifdef __cplusplus
}
#endif
#endif
/*---DRIVETRAIN_H---*/