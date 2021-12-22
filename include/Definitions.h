/*---DEFINITIONS_H---*/
#ifndef DEFINITIONS_H
#define DEFINITIONS_H
/*----------------------------------------------------------------------------*/
/*    Module:       Definitions.h                                             */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-11                                                */
/*----------------------------------------------------------------------------*/

/* GPIO DEFINITIONS */
#define LDR_EN 21

#define L_MTR_EN 6
#define L_MTR_A 22
#define L_MTR_B 27
#define L_ENC_A 17
#define L_ENC_B 4

#define R_MTR_EN 5
#define R_MTR_A 12
#define R_MTR_B 13
#define R_ENC_A 19
#define R_ENC_B 26

/* RETURN DEFINITIONS */
#define SUCCESS 0
#define FAILURE 1

/* PRINT DEFINITIONS */
#define STATUS_MSG "STATUS:"
#define INFO_MSG "INFO:"
#define WARNING_MSG "WARNING:"
#define ERROR_MSG "ERROR:"

/* MOTOR DEFINITIONS */
#define MOTOR_DEFAULT_MAX_POWER 255
#define MOTOR_REFRESH_RATE 3000 //[usec]
//#define MOTOR_PWM_FREQUENCY 100000

/* ENCODER DEFINITIONS */
#define ENCODER_DEFAULT_TICK_RESET 0
#define ENCODER_RPM_BUFFER_SIZE 3
#define ENCODER_EVENT_TIMEOUT 100
#define ENCODER_REFRESH_RATE 1500 //[usec]

/* KEYBOARD DEFINITIONS */
#define KEY_MAX_COUNT 32
#define MAX_KEY_DELTA_US 500000ULL

/* GENERAL DEFINITIONS */
#define NAME_MAX_SIZE 64

#endif
/*---DEFINITIONS_H---*/