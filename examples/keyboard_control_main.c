#include "include/Drivetrain.h"

#include <stdio.h>
#include <stdlib.h>

#include <pigpio.h>
#include <curses.h>

/*
INSTALL:
pigpio.h
    wget https://github.com/joan2937/pigpio/archive/master.zip
    unzip master.zip
    cd pigpio-master
    make
    sudo make install
curses.h
    sudo apt-get install libncurses5-dev libncursesw5-dev

BUILD:
gcc -Wall -pthread -o little_bot_program keyboard_control_main.c src/Drivetrain.c src/Motor.c src/Encoder.c -lpigpio -lrt -lncurses -ltinfo

RUN:
sudo ./little_bot_program
*/

int main(int argc, char * argv[]){
    if (gpioInitialise() < 0) { return FAILURE; }
    Encoder left_encoder = encoder_init("LEFT_ENCODER", L_ENC_A, L_ENC_B,  1/(44.0*21.3), false);
    Motor left_motor = motor_init("LEFT_MOTOR", L_MTR_EN, L_MTR_A, L_MTR_B, &left_encoder, true);

    Encoder right_encoder = encoder_init("RIGHT_ENCODER", R_ENC_A, R_ENC_B,  1/(44.0*21.3), true);
    Motor right_motor = motor_init("RIGHT_MOTOR", R_MTR_EN, R_MTR_A, R_MTR_B, &right_encoder, false);

    Drivetrain drivetrain = drivetrain_init("DRIVETRAIN", &left_motor, &right_motor);

    initscr();
    keypad(stdscr, TRUE);
    int c, speed = 200;
    while(true) {
        c = getch();
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
    drivetrain_del(&drivetrain);
    gpioTerminate();
    return SUCCESS;
}