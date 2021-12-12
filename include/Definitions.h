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

/* ENCODER DEFINITIONS */
#define ENCODER_DEFAULT_TICK_RESET 0
#define ENCODER_RPS_REFRESH_RATE 11
#define ENCODER_RPS_BUFFER_SIZE 10

/* GENERAL DEFINITIONS */
#define NAME_MAX_SIZE 64

#endif
/*---DEFINITIONS_H---*/