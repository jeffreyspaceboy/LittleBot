#include "../include/Definitions.h"
#include "../include/Motor.h"
#include "../include/Encoder.h"

#include <pigpio.h>
#include <stdio.h>

// Build with:
// g++ -Wall -pthread -o basic_motor_test basic_motor_test.cpp -lpigpio -lrt

// Compile with:
// sudo ./basic_motor_test

int main(int argc, char * argv[]){
    if (gpioInitialise() < 0) { return FAILURE; }
    Encoder left_encoder = encoder_init("LEFT_ENCODER", L_ENC_A, L_ENC_B,  1/(22.0*21.3), false);
    Motor left_motor = motor_init("LEFT_MOTOR", L_MTR_EN, L_MTR_A, L_MTR_B, false);
    motor_link_encoder(&left_motor, &left_encoder);

    Encoder right_encoder = encoder_init("RIGHT_ENCODER", R_ENC_A, R_ENC_B,  1/(22.0*21.3), false);
    Motor right_motor = motor_init("RIGHT_MOTOR", R_MTR_EN, R_MTR_A, R_MTR_B, false);
    motor_link_encoder(&right_motor, &right_encoder);

    gpioTerminate();
    return SUCCESS;
}