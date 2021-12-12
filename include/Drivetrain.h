/*---DRIVETRAIN_H---*/
#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H
/*----------------------------------------------------------------------------*/
/*    Module:       Drivetrain.h                                              */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-11                                                */
/*----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

/* LOCAL INCLUDES */
#include "Definitions.h"
#include "Motor.h"
#include "Encoder.h"

/* STANDARD INCLUDES */
#include <stdint.h>


/** @brief DRIVETRAIN TYPE - used to define a dual motor drivetrain.
 * @param name Name used for DEBUG printf
 * @param left_motor Left Motor
 * @param right_motor Right Motor
 */
typedef struct Drivetrain{
    char name[NAME_MAX_SIZE];
    Motor *left_motor, *right_motor;
} Drivetrain;


/** @brief Drivetrain initialization.
 * @param drivetrain_name Name for DEBUG
 * @param left_motor Motor for left side
 * @param right_motor Motor for right side
 * @return Drivetrain */
Drivetrain drivetrain_init(char drivetrain_name[NAME_MAX_SIZE], Motor *left_motor, Motor *right_motor);

/** @brief Drivetrain destruction.
 * @param drivetrain Drivetrain to be deleted 
 * @return int: SUCCESS or FAILURE */
int drivetrain_del(Drivetrain *drivetrain);


/** @brief Spin the drivetrain motors based on the powers given.
 * @param drivetrain Drivetrain you want to control
 * @param left_power Left motor power
 * @param right_power Right motor power
 * @return int: SUCCESS or FAILURE */
int drivetrain_spin(Drivetrain *drivetrain, int left_power, int right_power);

/** @brief Stop the drivetrain motors.
 * @param drivetrain Drivetrain to be stopped
 * @return int: SUCCESS or FAILURE */
int drivetrain_stop(Drivetrain *drivetrain);


#ifdef __cplusplus
}
#endif
#endif
/*---DRIVETRAIN_H---*/