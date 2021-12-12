/*---MAIN_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       main.c                                                    */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-11                                                */
/*----------------------------------------------------------------------------*/

/* LOCAL INCLUDES */
#include "include/Drivetrain.h"
#include "include/PID.h"

/* STANDARD INCLUDES */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>

/* NON-STANDARD INCLUDES */
#include <pigpio.h>
#include <curses.h>

/*
BUILD:
gcc -Wall -pthread -o little_bot main.c src/Encoder.c src/Motor.c src/PID.c src/Drivetrain.c -lpigpio -lrt -lncurses -ltinfo

RUN:
sudo ./little_bot
*/
  

#define errExit(msg) do { perror(msg); exit(EXIT_FAILURE); \
    } while (0)

/* Signal Handler for SIGINT */
void sigintHandler(int sig_num){
    gpioTerminate();
    write(STDERR_FILENO, "Caught SIGINT!\n", 15);
    
}

int main(int argc, char * argv[]){
    if (signal(SIGINT, sigintHandler) == SIG_ERR){ errExit("signal SIGINT"); }
    if (gpioInitialise() < 0) { return FAILURE; }
    Encoder_t left_encoder = encoder_init("LEFT_ENCODER", L_ENC_A, L_ENC_B,  1/(44.0*21.3), false);
    Motor_t left_motor = motor_init("LEFT_MOTOR", L_MTR_EN, L_MTR_A, L_MTR_B, &left_encoder, true);

    Encoder_t right_encoder = encoder_init("RIGHT_ENCODER", R_ENC_A, R_ENC_B,  1/(44.0*21.3), true);
    Motor_t right_motor = motor_init("RIGHT_MOTOR", R_MTR_EN, R_MTR_A, R_MTR_B, &right_encoder, false);

    Drivetrain_t drivetrain = drivetrain_init("DRIVETRAIN", &left_motor, &right_motor);
    PID_Controller_t pid_left = pid_init(400.0, 0.000000001, 550.0);
    PID_Controller_t pid_right = pid_init(400.0, 0.000000001, 550.0);
    int turn_bias = 100;

    int timesGood = 0;
    bool moveComplete = false;
    float left_rotations = motor_get_rotations(&left_motor);
    float right_rotations = motor_get_rotations(&right_motor);
    float distance = 2.0;
    float tolerance = 0.1;
    pid_start(&pid_left, distance, tolerance);
    pid_start(&pid_right, distance, tolerance);
    while(!moveComplete && right_rotations <= distance*2 && left_rotations <= distance*2){ 
        left_rotations = motor_get_rotations(&left_motor);
        right_rotations = motor_get_rotations(&right_motor);
        printf("%s (%f | %f)\n", INFO_MSG, left_rotations, right_rotations);
        if(left_rotations > right_rotations){
            drivetrain_spin(&drivetrain, (int)pid_power(&pid_left,left_rotations), (int)pid_power(&pid_right, right_rotations)+turn_bias);
        }else if(right_rotations > left_rotations){
            drivetrain_spin(&drivetrain, (int)pid_power(&pid_left, left_rotations)+turn_bias, (int)pid_power(&pid_right, right_rotations));
        }else{
            drivetrain_spin(&drivetrain, (int)pid_power(&pid_left, left_rotations), (int)pid_power(&pid_right, right_rotations));
        }
        
        if(fabs(pid_right.error)<=pid_right.error_tolerance && fabs(pid_left.error)<=pid_left.error_tolerance){ timesGood++; }
        if(timesGood >= 1000){ moveComplete = true; }
        gpioSleep(PI_TIME_RELATIVE, 0, 1000);
    }
    drivetrain_spin(&drivetrain, 0, 0);

    drivetrain_del(&drivetrain);
    gpioTerminate();
    exit(SUCCESS);
}
/*---MAIN_C---*/