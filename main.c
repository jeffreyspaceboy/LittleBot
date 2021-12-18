/*---MAIN_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       main.c                                                    */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-11                                                */
/*----------------------------------------------------------------------------*/

/* LOCAL INCLUDES */
#include "Drivetrain.h"
#include "PID.h"

/* STANDARD INCLUDES */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>

/* NON-STANDARD INCLUDES */
#include <pigpio.h>
#include <curses.h>

// BUILD:
// make
// --or--
// gcc -Wall -pthread -o little_bot main.c src/Encoder.c src/Motor.c src/PID.c src/Drivetrain.c -lpigpio -lrt -lncurses -ltinfo
// RUN:
// sudo ./little_bot

bool RUNNING = true;
void  signal_handler(int signal){ RUNNING = false; }

int main(int argc, char * argv[]){
    if (gpioInitialise() < 0) { return FAILURE; }
    signal(SIGINT, signal_handler);
    printf("\n===STARTING=ROBOT===\n");
    int STATUS = 0;

    /*---ROBOT-CODE-HERE---\/---*/
    float ratio = 1.0F/(44.0F * 21.3F);
    PID_Controller_t l_vel_pid = pid_init(3.5F, 0.00001F, 0.0001F);
    PID_Controller_t r_vel_pid = pid_init(3.5F, 0.00001F, 0.0001F);
    Encoder_t l_encoder = encoder_init("LEFT_ENCODER", L_ENC_A, L_ENC_B,  ratio, false);
    Encoder_t r_encoder = encoder_init("RIGHT_ENCODER", R_ENC_A, R_ENC_B,  ratio, true);
    Motor_t l_motor = motor_init("LEFT_MOTOR", L_MTR_EN, L_MTR_A, L_MTR_B, true, &l_encoder, &l_vel_pid);
    Motor_t r_motor = motor_init("RIGHT_MOTOR", R_MTR_EN, R_MTR_A, R_MTR_B, false, &r_encoder, &r_vel_pid);
    Drivetrain_t drivetrain = drivetrain_init("DRIVETRAIN", &l_motor, &r_motor);
    /*---ROBOT-CODE-HERE---/\---*/
    
    float rpm_target = 60.0F;
    while(RUNNING){
        /*---ROBOT-CODE-HERE---\/---*/
        STATUS |= drivetrain__set_rpm(&drivetrain, rpm_target, rpm_target);
        printf("L:(%f|%d) R:(%f|%d)\n", motor_get_rpm(&l_motor), l_motor.power, motor_get_rpm(&r_motor), r_motor.power);
        gpioSleep(PI_TIME_RELATIVE, 0, 50000);
        /*---ROBOT-CODE-HERE---/\---*/
        if(STATUS != SUCCESS){ break; }
    }

    printf("\n===STOPPING=ROBOT===\n");
    drivetrain_del(&drivetrain);

    gpioWrite_Bits_0_31_Clear(0xFFFFFFFF);
    gpioWrite_Bits_32_53_Clear(0x0003FFFF);
    gpioTerminate();
    printf("\n===ROBOT=DONE===\n");
    pthread_exit(NULL);
    return SUCCESS;
}
/*---MAIN_C---*/