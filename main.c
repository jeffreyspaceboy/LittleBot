#include "include/Drivetrain.h"
#include "include/PID.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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
gcc -Wall -pthread -o little_bot_program main.c src/Drivetrain.c src/PID.c src/Motor.c src/Encoder.c -lpigpio -lrt -lncurses -ltinfo

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
    PID_Controller pid_left = pid_init(400.0, 0.000000001, 550.0, 0.05);
    PID_Controller pid_right = pid_init(400.0, 0.000000001, 550.0, 0.05);
    int turn_bias = 50;

    int timesGood = 0;
    bool moveComplete = false;
    float left_rotations = motor_get_rotations(&left_motor);
    float right_rotations = motor_get_rotations(&right_motor);
    float distance = 2.0;
    while(!moveComplete && right_rotations <= distance*2 && left_rotations <= distance*2){ 
        uint32_t current_time = gpioTick();
        left_rotations = motor_get_rotations(&left_motor);
        right_rotations = motor_get_rotations(&right_motor);
        printf("(%f | %f)\n", left_rotations, right_rotations);
        if(left_rotations > right_rotations){
            drivetrain_spin(&drivetrain, (int)pid_power(&pid_left, distance, left_rotations, current_time), (int)pid_power(&pid_right, distance, right_rotations, current_time)+turn_bias);
        }else if(right_rotations > left_rotations){
            drivetrain_spin(&drivetrain, (int)pid_power(&pid_left, distance, left_rotations, current_time)+turn_bias, (int)pid_power(&pid_right, distance, right_rotations, current_time));
        }else{
            drivetrain_spin(&drivetrain, (int)pid_power(&pid_left, distance, left_rotations, current_time), (int)pid_power(&pid_right, distance, right_rotations, current_time));
        }
        
        if(fabs(pid_right.error)<=pid_right.error_tolerance && fabs(pid_left.error)<=pid_left.error_tolerance){ timesGood++; }
        if(timesGood >= 1000){ moveComplete = true; }
    }
    drivetrain_spin(&drivetrain, 0, 0);

    drivetrain_del(&drivetrain);
    gpioTerminate();
    return SUCCESS;
}

