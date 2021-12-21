/*---MAIN_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       main.c                                                    */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-21                                                */
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
    
    bool go_fwd = true;
    float rpm_target = 60.0F;
    while(RUNNING){
        /*---ROBOT-CODE-HERE---\/---*/
        // if(l_motor.rpm_target >= 150.0F){
        //     go_fwd = false;
        // }else if(l_motor.rpm_target <= -150.0F){
        //     go_fwd = true;
        // }

        // if(go_fwd == true){
        //     rpm_target += 1;
        // }else if(go_fwd == false){
        //     rpm_target -= 1;
        // }
        drivetrain_set_rpm(&drivetrain, rpm_target, rpm_target);
        printf("L:(%f|%f|%d) R:(%f|%f|%d)\n", motor_sense_rpm(&l_motor), l_motor.rpm_target, l_motor.power, motor_sense_rpm(&r_motor), r_motor.rpm_target, r_motor.power);
        gpioSleep(PI_TIME_RELATIVE, 0, 60000);
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