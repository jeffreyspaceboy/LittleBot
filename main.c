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

void  sigintHandler(int sig){
     char  c;
     signal(sig, SIG_IGN);
     printf("%s Are you sure you want to quit? [y/n]\n",WARNING_MSG);
     c = getchar();
     if (c == 'y' || c == 'Y'){ 
        gpioWrite_Bits_0_31_Clear(0xFFFFFFFF);
        gpioWrite_Bits_32_53_Clear(0x0003FFFF);
        gpioTerminate(); 
        exit(SUCCESS); 
    }
     else { signal(SIGINT, sigintHandler); }
     getchar();
}

int main(int argc, char * argv[]){
    signal(SIGINT, sigintHandler);
    if (gpioInitialise() < 0) { return FAILURE; }
    Encoder_t left_encoder = encoder_init("LEFT_ENCODER", L_ENC_A, L_ENC_B,  1/(44.0*21.3), false);
    Motor_t left_motor = motor_init("LEFT_MOTOR", L_MTR_EN, L_MTR_A, L_MTR_B, &left_encoder, true);

    Encoder_t right_encoder = encoder_init("RIGHT_ENCODER", R_ENC_A, R_ENC_B,  1/(44.0*21.3), true);
    Motor_t right_motor = motor_init("RIGHT_MOTOR", R_MTR_EN, R_MTR_A, R_MTR_B, &right_encoder, false);

    Drivetrain_t drivetrain = drivetrain_init("DRIVETRAIN", &left_motor, &right_motor);
    
    //drivetrain_pid_distance_spin(&drivetrain, 2.0F);
    //drivetrain_pid_velocity_spin(&drivetrain, 2.0F);
    PID_Controller_t pid_right = pid_init(500.0, 0.00001, 0.0);
    while(true){
        motor_spin(&right_motor, 200);
        motor_pid_velocity(&right_motor, &pid_right, 2.0F);
        //printf("%s (%f)\n", INFO_MSG, motor_get_rps(&right_motor));
        gpioSleep(PI_TIME_RELATIVE, 0, 5000);
    }
    // initscr();
    // keypad(stdscr, TRUE);
    // int c, speed = 200;
    // while(TRUE) {
    //     c = getch();
    //     switch(c){
    //         case KEY_UP:
    //             motor_pid_velocity(&right_motor, &pid_right, 2.0F);
    //             printf("%s (%f)\n", INFO_MSG, motor_get_rps(&right_motor));
    //             gpioSleep(PI_TIME_RELATIVE, 0, 5000);
    //             break;
    //         case KEY_DOWN:
    //             drivetrain_spin(&drivetrain, -speed, -speed);
    //             break;
    //         case KEY_LEFT:
    //             drivetrain_spin(&drivetrain, -speed, speed);
    //             break;
    //         case KEY_RIGHT:
    //             drivetrain_spin(&drivetrain, speed, -speed);
    //             break;
    //         default:
    //             drivetrain_stop(&drivetrain);
    //             break;
    //     }
    //     if(c == 10){ break; }
    // }
    // clrtoeol();
    // endwin();
    drivetrain_del(&drivetrain);
    gpioWrite_Bits_0_31_Clear(0xFFFFFFFF);
    gpioWrite_Bits_32_53_Clear(0x0003FFFF);
    gpioTerminate();
    return SUCCESS;
}
/*---MAIN_C---*/