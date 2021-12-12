/*---ENCODER_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       Encoder.c                                                 */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-11                                                */
/*----------------------------------------------------------------------------*/

/* LOCAL INCLUDES */
#include "../include/Encoder.h"

/* NON-STANDARD INCLUDES */
#include <pigpio.h>

/* STANDARD INCLUDES */
#include <stdio.h>
#include <string.h>
#include <math.h>


Encoder encoder_init(char encoder_name[NAME_MAX_SIZE], uint8_t gpio_phase_a_pin, uint8_t gpio_phase_b_pin,  float encoder_ratio, int reverse){
    Encoder new_encoder = {
        .gpio_phase_a = (reverse == 0) ? gpio_phase_a_pin : gpio_phase_b_pin,
        .gpio_phase_b = (reverse == 0) ? gpio_phase_b_pin : gpio_phase_a_pin,
        .prev_gpio = -1, // GPIO does not exist
        .level_phase_a = 2, // No level change
        .level_phase_b = 2, // No level change
        .count = ENCODER_DEFAULT_TICK_RESET,
        .prev_count = ENCODER_DEFAULT_TICK_RESET,
        .prev_us = 0U,
        .rps = 0.0F,
        .ratio = encoder_ratio,
    };
    strncpy(new_encoder.name, encoder_name, sizeof(new_encoder.name));

    //Clear rps buffer
    for(int i = 0; i < ENCODER_RPS_BUFFER_SIZE; i++){
        new_encoder.prev_rps[i] = 0.0F;
    }

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
    encoder->prev_count = ENCODER_DEFAULT_TICK_RESET;
    return SUCCESS;
}


float encoder_get_rotations(Encoder *encoder){
    return ((float)encoder->count) * encoder->ratio;
}

float encoder_get_angle_degrees(Encoder *encoder){
    return encoder_get_rotations(encoder) * 360.0F;
}

float encoder_get_angle_radians(Encoder *encoder){
    return encoder_get_rotations(encoder) * 2.0F*M_PI;
}


float encoder_refresh_rps(Encoder *encoder, uint32_t current_us){
    encoder->rps = ((float)(encoder->count - encoder->prev_count) / (float)(current_us - encoder->prev_us)) * 1000000.0F * encoder->ratio;
    encoder->prev_count = encoder->count;
    encoder->prev_us = current_us;
    float sum = 0;
    for(int i = ENCODER_RPS_BUFFER_SIZE-1; i >= 1; i--){
        encoder->prev_rps[i] = encoder->prev_rps[i-1];
        sum += encoder->prev_rps[i];
    }
    encoder->prev_rps[0] = encoder->rps;
    sum += encoder->prev_rps[0];
    encoder->avg_rps = sum/ENCODER_RPS_BUFFER_SIZE;
    return encoder->rps;
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

    // Refresh RPS every QUARTER rotation of the encoder. (Subject to change)
    if(encoder->count % ENCODER_RPS_REFRESH_RATE == 0){
        encoder_refresh_rps(encoder, current_us); 
    }
    return;
}
/*---ENCODER_C---*/