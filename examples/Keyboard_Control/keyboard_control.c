/*---KEYBOARD_CONTROL_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       keyboard_control.c                                        */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-11                                                */
/*----------------------------------------------------------------------------*/

/* LOCAL INCLUDES */
#include "../../include/Drivetrain.h"
#include "../../include/PID.h"

/* STANDARD INCLUDES */
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdint.h>
#include <stdbool.h>
#include <signal.h>
#include <unistd.h>

/* NON-STANDARD INCLUDES */
#include <pigpio.h>
#include <ncurses.h>

/*
BUILD:
gcc -Wall -pthread -o keyboard_control keyboard_control.c ../../src/Encoder.c ../../src/PID.c ../../src/Motor.c ../../src/Drivetrain.c -lpigpio -lrt -lncurses -ltinfo

RUN:
sudo ./keyboard_control
*/

bool RUNNING = true;
void  signal_handler(int signal){ RUNNING = false; }

#define MAX_KEY_DELTA_US 500000ULL
uint64_t get_us() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

typedef enum keyeevent_t {
    eKEY_DOWN,
    eKEY_UP,
    eKEY_REPEAT,
    eKEY_NOKEY,
}keyeevent_t;

typedef struct keystate_t {
    uint64_t last_us;
    uint64_t delta_us;
    int last_ch;
}keystate_t;

keyeevent_t check_key(keystate_t *kstate, int ch) {
    kstate->delta_us = get_us() - kstate->last_us;
    if (ch == -1) {
        if ((kstate->delta_us > MAX_KEY_DELTA_US) && (kstate->last_ch != 0)) {
            kstate->last_ch = 0;
            return eKEY_UP;
        }
        return eKEY_NOKEY;
    }
    kstate->last_us = get_us();
    if ((ch >= 0) && (ch != kstate->last_ch)) {
        kstate->last_ch = ch;
        return eKEY_DOWN;
    }
    return eKEY_REPEAT;
}

int main() {
    if (gpioInitialise() < 0) { return FAILURE; }
    printf("\n\n===STARTING=ROBOT===\n\n");
    Encoder_t left_encoder = encoder_init("LEFT_ENCODER", L_ENC_A, L_ENC_B,  1/(44.0*21.3), false);
    PID_Controller_t pid_velocity_left = pid_init(3.5F, 0.000005F, 0.1F); //RPM
    Motor_t left_motor = motor_init("LEFT_MOTOR", L_MTR_EN, L_MTR_A, L_MTR_B, true, &left_encoder, &pid_velocity_left);

    Encoder_t right_encoder = encoder_init("RIGHT_ENCODER", R_ENC_A, R_ENC_B,  1/(44.0*21.3), true);
    PID_Controller_t pid_velocity_right = pid_init(3.5F, 0.000005F, 0.1F); //RPM
    Motor_t right_motor = motor_init("RIGHT_MOTOR", R_MTR_EN, R_MTR_A, R_MTR_B, false, &right_encoder, &pid_velocity_right);
    
    Drivetrain_t drivetrain = drivetrain_init("DRIVETRAIN", &left_motor, &right_motor);

    keystate_t keystate = {0,0,0};
    WINDOW *win = initscr();
    keypad(stdscr, TRUE);
    noecho();
    nodelay(win, TRUE);

    int target = 230;
    float left_power = 0.0F, right_power = 0.0F;
    refresh();
    while(RUNNING){
        signal(SIGINT, signal_handler);
        int ch = getch();
        keyeevent_t evt = check_key(&keystate,ch);
        switch(evt) {
            case eKEY_DOWN:
                //printf("Key[%lu] 0x%02x down\r\n",keystate.last_us,(uint8_t)ch);
                switch(ch){
                    case 'w':
                        left_power = motor_spin(&left_motor, target);
                        right_power = motor_spin(&right_motor, target);
                        break;
                    case 's':
                        left_power = motor_spin(&left_motor, -target);
                        right_power = motor_spin(&right_motor, -target);
                        break;
                    case 'd':
                        left_power = motor_spin(&left_motor, target);
                        right_power = motor_spin(&right_motor, -target);
                        break;
                    case 'a':
                        left_power = motor_spin(&left_motor, -target);
                        right_power = motor_spin(&right_motor, target);
                        break;
                    default:
                        left_power = motor_spin(&left_motor, 0);
                        right_power = motor_spin(&right_motor, 0);
                        break;
                }
                printf("L:(%f|%f) R:(%f|%f)\r\n", motor_get_rpm(&left_motor), left_power, motor_get_rpm(&right_motor), right_power);
                break;
            case eKEY_UP:
                printf("Key[%lu] 0x%02x up\r\n",keystate.delta_us,(uint8_t)ch);
                drivetrain_spin(&drivetrain, 0, 0);
                break;
            case eKEY_REPEAT:
                //printf("Key[%lu] 0x%02x repeat\r\n",keystate.last_us,(uint8_t)ch);
                printf("L:(%f|%f) R:(%f|%f)\r\n", motor_get_rpm(&left_motor), left_power, motor_get_rpm(&right_motor), right_power);
                break;
            case eKEY_NOKEY:
                break;
        }
        //gpioSleep(PI_TIME_RELATIVE, 0, 5000);
    }
    getch();
    endwin();
    printf("\n\n===STOPPING=ROBOT===\n\n");
    drivetrain_del(&drivetrain);
    gpioWrite_Bits_0_31_Clear(0xFFFFFFFF);
    gpioWrite_Bits_32_53_Clear(0x0003FFFF);
    gpioTerminate();
    return SUCCESS;
}
/*---KEYBOARD_CONTROL_C---*/