#include "include/Drivetrain.h"
#include "include/PID.h"

#include <stdint.h>
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
gcc -Wall -pthread -o little_bot_program main.c src/Drivetrain.c src/Motor.c src/Encoder.c -lpigpio -lrt -lncurses -ltinfo

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
    PID_Controller pid_left = pid_init(400.0, 0.0, 0.5, 0.005);
    PID_Controller pid_right = pid_init(400.0, 0.0, 0.5, 0.005);

    // drivetrain_spin(&drivetrain, 255, 255);
    // gpioSleep(PI_TIME_RELATIVE, 2, 500000);
    // drivetrain_stop(&drivetrain);

    //drivetrain_spin(&drivetrain, 255, 255);
    //drivetrain_stop(&drivetrain);

    while(true){
         float left_rotations = (float)(drivetrain.left_motor->encoder->ticks/(44.0*21.3));
         float right_rotations = (float)(drivetrain.right_motor->encoder->ticks/(44.0*21.3));
         uint32_t current_time = gpioTick();
         drivetrain_spin(&drivetrain, pid_power(&pid_left, 1.0, left_rotations, current_time), pid_power(&pid_right, 1.0, right_rotations, current_time));
         printf("(%f | %f)\n",left_rotations,right_rotations);
         //printf("(%f | %f)\n",drivetrain.left_motor->encoder->rpm,drivetrain.right_motor->encoder->rpm);
         //gpioSleep(PI_TIME_RELATIVE, 0, 100000);
    }
    
    drivetrain_del(&drivetrain);
    gpioTerminate();
    return SUCCESS;
}