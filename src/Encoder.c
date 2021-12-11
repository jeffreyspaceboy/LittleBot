/*---ENCODER_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       Motor.c                                                   */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-09                                                */
/*----------------------------------------------------------------------------*/
#include "../include/Encoder.h"

#include <pigpio.h>

#include <stdio.h>
#include <string.h>

Encoder encoder_init(char encoder_name[NAME_MAX_SIZE], int gpio_phase_a_pin, int gpio_phase_b_pin,  float encoder_ratio, int reverse){
    Encoder new_encoder = {
        .name = "",
        .gpio_phase_a = (reverse == 0) ? gpio_phase_a_pin : gpio_phase_b_pin,
        .gpio_phase_b = (reverse == 0) ? gpio_phase_b_pin : gpio_phase_a_pin,
        .prev_gpio = -1, // GPIO does not exist
        .level_phase_a = 2, // No level change
        .level_phase_b = 2, // No level change
        .count = ENCODER_DEFAULT_TICK_RESET,
        .prev_count = ENCODER_DEFAULT_TICK_RESET,
        .prev_us = 0,
        .rps = 0.0,
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
    gpioSetISRFuncEx(encoder->gpio_phase_a, EITHER_EDGE, 100, encoder_event_callback, (void *)encoder);
    gpioSetISRFuncEx(encoder->gpio_phase_b, EITHER_EDGE, 100, encoder_event_callback, (void *)encoder);
    return encoder_reset(encoder);
}

int encoder_reset(Encoder *encoder){
    encoder->count = ENCODER_DEFAULT_TICK_RESET;
    return SUCCESS;
}

float encoder_refresh_rps(Encoder *encoder, uint32_t current_us){
    encoder->rps = ((float)(encoder->count - encoder->prev_count) / (float)(current_us - encoder->prev_us)) * 1000000.0 * encoder->ratio;
    encoder->prev_count = encoder->count;
    encoder->prev_us = current_us;
    return encoder->rps;
}

float encoder_get_rotations(Encoder *encoder){
    return ((float)encoder->count) * encoder->ratio;
}

void encoder_event_callback(int gpio, int level, uint32_t current_us, void *data){
    Encoder *encoder = (Encoder *) data;
    if(gpio == encoder->gpio_phase_a){
        encoder->level_phase_a = level; 
        if(encoder->level_phase_b == 0){
            encoder->count += (encoder->level_phase_a == 0) ? -1 : 1;
        }else{
            encoder->count += (encoder->level_phase_a == 0) ? 1 : -1;
        }
    }else if(gpio == encoder->gpio_phase_b){
        encoder->level_phase_b = level;
        if(encoder->level_phase_a == 0){
            encoder->count += (encoder->level_phase_b == 0) ? 1 : -1;
        }else{
            encoder->count += (encoder->level_phase_b == 0) ? -1 : 1;
        }
    } 

    if(encoder->count % ENCODER_RPM_REFRESH_RATE == 0){ 
        encoder_refresh_rps(encoder, current_us); 
    }
    return;
}
/*---ENCODER_C---*/