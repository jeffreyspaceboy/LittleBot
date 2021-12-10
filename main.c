#include "include/Definitions.h"
#include "include/Motor.h"
#include "include/Encoder.h"
#include "include/Drivetrain.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <pigpio.h>
#include <curses.h>

//Install:
//sudo apt-get install libncurses5-dev libncursesw5-dev

// Build with:
// gcc -Wall -pthread -o little_bot_program main.c src/Drivetrain.c src/Motor.c src/Encoder.c -lpigpio -lrt
// gcc -Wall -pthread -o little_bot_program main.c src/Drivetrain.c src/Motor.c src/Encoder.c -lpigpio -lrt -lncurses -ltinfo


// Compile with:
// sudo ./little_bot_program

#define WIDTH 30
#define HEIGHT 10 

int main(int argc, char * argv[]){
    if (gpioInitialise() < 0) { return FAILURE; }
    Encoder left_encoder = encoder_init("LEFT_ENCODER", L_ENC_A, L_ENC_B,  1/(44.0*21.3), false);
    Motor left_motor = motor_init("LEFT_MOTOR", L_MTR_EN, L_MTR_A, L_MTR_B, &left_encoder, true);

    Encoder right_encoder = encoder_init("RIGHT_ENCODER", R_ENC_A, R_ENC_B,  1/(44.0*21.3), true);
    Motor right_motor = motor_init("RIGHT_MOTOR", R_MTR_EN, R_MTR_A, R_MTR_B, &right_encoder, false);

    Drivetrain drivetrain = drivetrain_init("DRIVETRAIN", &left_motor, &right_motor);

    // drivetrain_spin(&drivetrain, 255, 255);
    // gpioSleep(PI_TIME_RELATIVE, 2, 500000);
    // drivetrain_stop(&drivetrain);

    //drivetrain_spin(&drivetrain, 255, 255);
    //drivetrain_stop(&drivetrain);

    initscr();
    noecho();
    keypad(stdscr, TRUE);
    int speed = 200;
    int c;
    while(true) {
        c = getch();
        printf("%d\r\n",c);
        switch(c){
            case KEY_UP:
                drivetrain_spin(&drivetrain, speed, speed);
                break;
            case KEY_DOWN:
                drivetrain_spin(&drivetrain, -speed, -speed);
                break;
            case KEY_LEFT:
                drivetrain_spin(&drivetrain, -speed, speed);
                break;
            case KEY_RIGHT:
                drivetrain_spin(&drivetrain, speed, -speed);
                break;
            default:
                drivetrain_stop(&drivetrain);
                break;
        }
        if(c == 10){ break; } //Break if Enter is pressed
    }
    clrtoeol();
    endwin();

    // while(1){
    //     //printf("(%f | %f)\n",360*drivetrain.left_motor->encoder->ticks/(44.0*21.3),360*drivetrain.right_motor->encoder->ticks/(44.0*21.3));
    //     printf("(%f | %f)\n",drivetrain.left_motor->encoder->rpm,drivetrain.right_motor->encoder->rpm);
    //     gpioSleep(PI_TIME_RELATIVE, 0, 100000);
    // }
    
    drivetrain_del(&drivetrain);
    gpioTerminate();
    return SUCCESS;
}