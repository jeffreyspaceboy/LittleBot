/*---ENCODER_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       Motor.c                                                   */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-06                                                */
/*----------------------------------------------------------------------------*/
#include "../include/Encoder.h"

#include <pigpio.h>
#include <stdio.h>
#include <string.h>

Encoder encoder_init(char encoder_name[NAME_MAX_SIZE], int gpio_phase_a_pin, int gpio_phase_b_pin,  float encoder_ratio, bool reverse){
    Encoder new_encoder = {
        .name = "",
        .gpio_phase_a = (reverse == false) ? gpio_phase_a_pin : gpio_phase_b_pin,
        .gpio_phase_b = (reverse == false) ? gpio_phase_b_pin : gpio_phase_a_pin,
        .prev_gpio = -1, // GPIO does not exist
        .level_phase_a = 2, // No level change
        .level_phase_b = 2, // No level change
        .ticks = ENCODER_DEFAULT_TICK_RESET,
        .prev_ticks = ENCODER_DEFAULT_TICK_RESET,
        .prev_tick_us = 0,
        .rpm = 0.0,
        .ratio = encoder_ratio,
    };
    strncpy(new_encoder.name, encoder_name, sizeof(new_encoder.name));

    gpioSetMode(new_encoder.gpio_phase_a, PI_INPUT);
    gpioSetMode(new_encoder.gpio_phase_b, PI_INPUT);

    gpioSetPullUpDown(new_encoder.gpio_phase_a, PI_PUD_UP);
    gpioSetPullUpDown(new_encoder.gpio_phase_b, PI_PUD_UP);

    return new_encoder;
}

int encoder_del(Encoder *encoder){
    gpioSetISRFuncEx(encoder->gpio_phase_a, EITHER_EDGE, 0, NULL, (void *)encoder);
    gpioSetISRFuncEx(encoder->gpio_phase_b, EITHER_EDGE, 0, NULL, (void *)encoder);

    gpioSetPullUpDown(encoder->gpio_phase_a, PI_PUD_OFF);
    gpioSetPullUpDown(encoder->gpio_phase_b, PI_PUD_OFF);
    return SUCCESS;
}

int encoder_start(Encoder *encoder){
    gpioSetISRFuncEx(encoder->gpio_phase_a, EITHER_EDGE, 5000, encoder_event_callback, (void *)encoder);
    gpioSetISRFuncEx(encoder->gpio_phase_b, EITHER_EDGE, 5000, encoder_event_callback, (void *)encoder);
    return SUCCESS;
}

int encoder_reset(Encoder *encoder){
    encoder->ticks = ENCODER_DEFAULT_TICK_RESET;
    return SUCCESS;
}

double encoder_refresh_rpm(Encoder *encoder, uint32_t current_tick_us){
    //TODO: Calculate RPM using averaging.
    double rpus = (double)(encoder->ticks - encoder->prev_ticks) / (double)(current_tick_us - encoder->prev_tick_us);
    encoder->prev_ticks = encoder->ticks;
    encoder->prev_tick_us = current_tick_us;
    encoder->rpm = rpus * 60000000.0 * encoder->ratio;
    return encoder->rpm;
}

void encoder_event_callback(int gpio, int level, uint32_t tick, void *data){
    Encoder *encoder = (Encoder *) data;

    if(gpio == encoder->gpio_phase_a){
        encoder->level_phase_a = level; 
        if(encoder->level_phase_b == 0){
            encoder->ticks += (encoder->level_phase_a == 0) ? -1 : 1;
        }else{
            encoder->ticks += (encoder->level_phase_a == 0) ? 1 : -1;
        }
    }else if(gpio == encoder->gpio_phase_b){
        encoder->level_phase_b = level;
        if(encoder->level_phase_a == 0){
            encoder->ticks += (encoder->level_phase_b == 0) ? 1 : -1;
        }else{
            encoder->ticks += (encoder->level_phase_b == 0) ? -1 : 1;
        }
    } 

    if(encoder->ticks % ENCODER_RPM_REFRESH_RATE == 0){
        encoder_refresh_rpm(encoder, tick);
    }  
    return;
}
/*---ENCODER_C---*/