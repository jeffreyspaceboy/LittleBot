/*---KEYBOARD_CONTROL_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       keyboard_control.c                                        */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-11                                                */
/*----------------------------------------------------------------------------*/

/* LOCAL INCLUDES */
#include "../../include/Drivetrain.h"

/* STANDARD INCLUDES */
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdint.h>
#include <stdbool.h>

/* NON-STANDARD INCLUDES */
#include <pigpio.h>
#include <ncurses.h>

/*
BUILD:
gcc -Wall -pthread -o keyboard_control keyboard_control.c ../../src/Encoder.c ../../src/Motor.c ../../src/Drivetrain.c -lpigpio -lrt -lncurses -ltinfo

RUN:
sudo ./keyboard_control
*/

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
    Encoder left_encoder = encoder_init("LEFT_ENCODER", L_ENC_A, L_ENC_B,  1/(44.0*21.3), false);
    Motor left_motor = motor_init("LEFT_MOTOR", L_MTR_EN, L_MTR_A, L_MTR_B, &left_encoder, true);
    Encoder right_encoder = encoder_init("RIGHT_ENCODER", R_ENC_A, R_ENC_B,  1/(44.0*21.3), true);
    Motor right_motor = motor_init("RIGHT_MOTOR", R_MTR_EN, R_MTR_A, R_MTR_B, &right_encoder, false);
    Drivetrain drivetrain = drivetrain_init("DRIVETRAIN", &left_motor, &right_motor);

    keystate_t keystate = {0,0,0};
    WINDOW *win = initscr();
    keypad(stdscr, TRUE);
    noecho();
    nodelay(win, TRUE);

    int speed = 200;
    refresh();
    while (1) {
        int ch = getch();
        keyeevent_t evt = check_key(&keystate,ch);
        switch(evt) {
            case eKEY_DOWN:
                printw("Key[%llu] 0x%02x down\n",keystate.last_us,(uint8_t)ch);
                drivetrain_spin(&drivetrain, speed, speed);
                break;
            case eKEY_UP:
                printw("Key[%llu] 0x%02x up\n",keystate.delta_us,(uint8_t)ch);
                drivetrain_spin(&drivetrain, 0, 0);
                break;
            case eKEY_REPEAT:
                printw("Key[%llu] 0x%02x repeat\n",keystate.last_us,(uint8_t)ch);
                break;
            case eKEY_NOKEY:
                break;
        }
        usleep(10000);
    }
    getch();
    endwin();
    drivetrain_del(&drivetrain);
    gpioTerminate();
    return SUCCESS;
}
/*---KEYBOARD_CONTROL_C---*/