#include "include/Definitions.h"
#include "include/Motor.h"
#include "include/Encoder.h"
#include "include/Drivetrain.h"

#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>

// Build with:
// gcc -Wall -pthread -o little_bot_program main.c src/Drivetrain.c src/Motor.c src/Encoder.c -lpigpio -lrt

// Compile with:
// sudo ./little_bot_program

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

    int c;
    system ("/bin/stty raw"); /* use system call to make terminal send all keystrokes directly to stdin */
    while((c = getchar())!= '.') {
        if(c == 'w'){
            drivetrain_spin(&drivetrain, 255, 255);
        }else if(c == 's'){
            drivetrain_spin(&drivetrain, -255, -255);
        }else if(c == 'a'){
            drivetrain_spin(&drivetrain, -255, 255);
        }else if(c == 'd'){
            drivetrain_spin(&drivetrain, 255, -255);
        }else{
            drivetrain_stop(&drivetrain);
        }
        
        putchar(c); /* type a period to break out of the loop, since CTRL-D won't work raw */
    }
    system ("/bin/stty cooked"); /* use system call to set terminal behaviour to more normal behaviour */
    

    
    // while(1){
    //     //printf("(%f | %f)\n",360*drivetrain.left_motor->encoder->ticks/(44.0*21.3),360*drivetrain.right_motor->encoder->ticks/(44.0*21.3));
    //     printf("(%f | %f)\n",drivetrain.left_motor->encoder->rpm,drivetrain.right_motor->encoder->rpm);
    //     gpioSleep(PI_TIME_RELATIVE, 0, 100000);
    // }
    
    drivetrain_del(&drivetrain);
    gpioTerminate();
    return SUCCESS;
}