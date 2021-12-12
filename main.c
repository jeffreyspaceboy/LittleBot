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

// BUILD:
// gcc -Wall -pthread -o little_bot main.c src/Encoder.c src/Motor.c src/PID.c src/Drivetrain.c -lpigpio -lrt -lncurses -ltinfo
// RUN:
// sudo ./little_bot

bool LOOP_ENABLED = true;
void signal_handler(int signal);

int main(int argc, char * argv[]){
    if (gpioInitialise() < 0) { return FAILURE; }
    printf("\n\n===STARTING=ROBOT===\n\n");
    int STATUS = 0;

    /*---ROBOT-CODE-HERE---*/
    Encoder_t left_encoder = encoder_init("LEFT_ENCODER", L_ENC_A, L_ENC_B,  1/(44.0*21.3), false);
    PID_Controller_t pid_velocity_left = pid_init(4.0F, 0.001F, 0.03F);
    Motor_t left_motor = motor_init("LEFT_MOTOR", L_MTR_EN, L_MTR_A, L_MTR_B, true, &left_encoder, &pid_velocity_left);

    Encoder_t right_encoder = encoder_init("RIGHT_ENCODER", R_ENC_A, R_ENC_B,  1/(44.0*21.3), true);
    PID_Controller_t pid_velocity_right = pid_init(4.0F, 0.001F, 0.03F);
    Motor_t right_motor = motor_init("RIGHT_MOTOR", R_MTR_EN, R_MTR_A, R_MTR_B, false, &right_encoder, &pid_velocity_right);
    
    Drivetrain_t drivetrain = drivetrain_init("DRIVETRAIN", &left_motor, &right_motor);
    //PID_Controller_t pid_distance = pid_init(0.005F, 0.0000000F, 0.00F);
    /*---ROBOT-CODE-HERE---*/
    
    //drivetrain_pid_distance_spin(&drivetrain, 2.0F);
    //drivetrain_pid_velocity_spin(&drivetrain, 2.0F);

    
    while(LOOP_ENABLED){
        signal(SIGINT, signal_handler);

        /*---ROBOT-CODE-HERE---*/
        motor_pid_velocity(&left_motor, 100.0F);
        motor_pid_velocity(&right_motor, 100.0F);
        printf("%f, %f\n", motor_get_rpm(&left_motor),  motor_get_rpm(&right_motor));
        gpioSleep(PI_TIME_RELATIVE, 0, 5000);
        /*---ROBOT-CODE-HERE---*/
        
        if(STATUS != SUCCESS){ break; }
    }

    drivetrain_del(&drivetrain);
    gpioWrite_Bits_0_31_Clear(0xFFFFFFFF);
    gpioWrite_Bits_32_53_Clear(0x0003FFFF);
    gpioTerminate();
    return SUCCESS;
}

void  signal_handler(int signal){
    printf("\n\n===STOPPING=ROBOT===\n\n");
    LOOP_ENABLED = false; 
}
/*---MAIN_C---*/