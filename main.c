#include "include/Definitions.h"
#include "include/Motor.h"
#include "include/Encoder.h"

#include <pigpio.h>
#include <stdio.h>

// Build with:
// gcc -Wall -pthread -o little_bot_program main.c src/Motor.c src/Encoder.c -lpigpio -lrt

// Compile with:
// sudo ./little_bot_program

int main(int argc, char * argv[]){
    if (gpioInitialise() < 0) { return FAILURE; }
    Encoder left_encoder = encoder_init("LEFT_ENCODER", L_ENC_A, L_ENC_B,  1/(44.0*21.3), false);
    Motor left_motor = motor_init("LEFT_MOTOR", L_MTR_EN, L_MTR_A, L_MTR_B, true);
    motor_link_encoder(&left_motor, &left_encoder);
    encoder_start(&left_encoder);

    Encoder right_encoder = encoder_init("RIGHT_ENCODER", R_ENC_A, R_ENC_B,  1/(44.0*21.3), true);
    Motor right_motor = motor_init("RIGHT_MOTOR", R_MTR_EN, R_MTR_A, R_MTR_B, false);
    motor_link_encoder(&right_motor, &right_encoder);
    encoder_start(&right_encoder);

    while(1){
        printf("(%f | %f)\n",left_encoder.ticks/(44.0*21.3),right_encoder.ticks/(44.0*21.3));
        gpioSleep(PI_TIME_RELATIVE, 0, 100000);
    }
    // motor_spin(&left_motor, 200);
    // motor_spin(&right_motor, 200);
    // gpioSleep(PI_TIME_RELATIVE, 2, 500000);
    // motor_stop(&left_motor);
    // motor_stop(&right_motor);

    gpioTerminate();
    return SUCCESS;
}