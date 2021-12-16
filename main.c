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
// gcc -Wall -pthread -o little_bot main.c src/Encoder.c src/Motor.c src/PID.c src/Drivetrain.c -lpigpio -lrt -lncurses -ltinfo
// RUN:
// sudo ./little_bot

bool RUNNING = true;
void  signal_handler(int signal){ RUNNING = false; }

int main(int argc, char * argv[]){
    if (gpioInitialise() < 0) { return FAILURE; }
    printf("\n\n===STARTING=ROBOT===\n\n");
    int STATUS = 0;

    /*---ROBOT-CODE-HERE---\/---*/
    Encoder_t left_encoder = encoder_init("LEFT_ENCODER", L_ENC_A, L_ENC_B,  1/(44.0*21.3), false);
    //PID_Controller_t pid_velocity_left = pid_init(4.0F, 0.001F, 0.03F); //RPS
    //PID_Controller_t pid_velocity_left = pid_init(4.0F, 0.000005F, 0.5F); //RPM //First good Setup
    PID_Controller_t pid_velocity_left = pid_init(3.5F, 0.000005F, 0.1F); //RPM //Tuning First Setup
    Motor_t left_motor = motor_init("LEFT_MOTOR", L_MTR_EN, L_MTR_A, L_MTR_B, true, &left_encoder, &pid_velocity_left);

    Encoder_t right_encoder = encoder_init("RIGHT_ENCODER", R_ENC_A, R_ENC_B,  1/(44.0*21.3), true);
    //PID_Controller_t pid_velocity_right = pid_init(4.0F, 0.001F, 0.03F); //RPS
    PID_Controller_t pid_velocity_right = pid_init(3.5F, 0.000005F, 0.1F); //RPM //Tuning First Setup
    Motor_t right_motor = motor_init("RIGHT_MOTOR", R_MTR_EN, R_MTR_A, R_MTR_B, false, &right_encoder, &pid_velocity_right);
    
    Drivetrain_t drivetrain = drivetrain_init("DRIVETRAIN", &left_motor, &right_motor);
    
    pthread_t left_motor_thread;
    pthread_create(&left_motor_thread, NULL, motor_control_thread, (void *)&left_motor);
    /*---ROBOT-CODE-HERE---/\---*/
    
    float rpm_target = 130.0F;
    float left_power, right_power;
    while(RUNNING){
        signal(SIGINT, signal_handler);

        /*---ROBOT-CODE-HERE---\/---*/
        left_power = motor_set_rpm(&left_motor, rpm_target);
        //right_power = motor_set_rpm(&right_motor, rpm_target);
        printf("L:(%f|%f) R:(%f|%f)\n", motor_get_rpm(&left_motor), left_power, motor_get_rpm(&right_motor), right_power);
        gpioSleep(PI_TIME_RELATIVE, 0, 5000);
        /*---ROBOT-CODE-HERE---/\---*/
        
        if(STATUS != SUCCESS){ break; }
    }

    printf("\n\n===STOPPING=ROBOT===\n\n");
    pthread_cancel(left_motor_thread);

    drivetrain_del(&drivetrain);
    gpioWrite_Bits_0_31_Clear(0xFFFFFFFF);
    gpioWrite_Bits_32_53_Clear(0x0003FFFF);
    gpioTerminate();
    return SUCCESS;
}
/*---MAIN_C---*/