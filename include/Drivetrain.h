/*---DRIVETRAIN_H---*/
#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP
/*----------------------------------------------------------------------------*/
/*    Module:       Drivetrain.h                                              */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-07                                                */
/*----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#include "Motor.h"
#include "Encoder.h"

typedef struct{
    Motor left_motor, right_motor; 
    Encoder left_encoder, right_encoder;
} Drivetrain;

// Drivetrain();
Drivetrain drivetrain_init(Motor *left_motor, Motor *right_motor, Encoder *left_encoder, Encoder *right_encoder);
void drivetrain_spin(Drivetrain *drivetrain, int velocity);
void drivetrain_stop(Drivetrain *drivetrain);
 
#ifdef __cplusplus
}
#endif
#endif
/*---DRIVETRAIN_H---*/