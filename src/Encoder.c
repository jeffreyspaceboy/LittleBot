/*---ENCODER_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       Encoder.c                                                 */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-11                                                */
/*----------------------------------------------------------------------------*/

/* LOCAL INCLUDES */
#include "Encoder.h"

/* NON-STANDARD INCLUDES */
#include <pigpio.h>

/* STANDARD INCLUDES */
#include <stdio.h>
#include <string.h>
#include <math.h>


Encoder_t encoder_init(char encoder_name[NAME_MAX_SIZE], uint8_t gpio_phase_a_pin, uint8_t gpio_phase_b_pin,  float encoder_ratio, int reverse){
    Encoder_t new_encoder = {
        .gpio_phase_a = (reverse == 0) ? gpio_phase_a_pin : gpio_phase_b_pin,
        .gpio_phase_b = (reverse == 0) ? gpio_phase_b_pin : gpio_phase_a_pin,
        .prev_gpio = -1, // GPIO does not exist
        .level_phase_a = 2, // No level change
        .level_phase_b = 2, // No level change
        .count = ENCODER_DEFAULT_TICK_RESET,
        .prev_count = ENCODER_DEFAULT_TICK_RESET,
        .prev_us = 0U,
        .rpm = 0.0F,
        .ratio = encoder_ratio,
    };
    strncpy(new_encoder.name, encoder_name, sizeof(new_encoder.name));
    if(pthread_mutex_init(new_encoder.mutex, NULL) != 0){
        printf("%s(%s) Encoder mutex init failed.\n",ERROR_MSG,new_encoder.name);
        return new_encoder;
    }

    //Clear rps buffer
    for(int i = 0; i < ENCODER_RPM_BUFFER_SIZE; i++){
        new_encoder.prev_rpm[i] = 0.0F;
    }

    gpioSetMode(new_encoder.gpio_phase_a, PI_INPUT);
    gpioSetMode(new_encoder.gpio_phase_b, PI_INPUT);
    gpioSetPullUpDown(new_encoder.gpio_phase_a, PI_PUD_UP);
    gpioSetPullUpDown(new_encoder.gpio_phase_b, PI_PUD_UP);
    return new_encoder;
}

int encoder_del(Encoder_t *encoder){
    gpioSetISRFuncEx(encoder->gpio_phase_a, EITHER_EDGE, 0, NULL, (void *)NULL);
    gpioSetISRFuncEx(encoder->gpio_phase_b, EITHER_EDGE, 0, NULL, (void *)NULL);
    gpioSetPullUpDown(encoder->gpio_phase_a, PI_PUD_OFF);
    gpioSetPullUpDown(encoder->gpio_phase_b, PI_PUD_OFF);
    pthread_mutex_destroy(encoder->lock);
    return SUCCESS;
}


int encoder_start(Encoder_t *encoder){
    gpioSetISRFuncEx(encoder->gpio_phase_a, EITHER_EDGE, ENCODER_EVENT_TIMEOUT, encoder_event_callback, (void *)encoder);
    gpioSetISRFuncEx(encoder->gpio_phase_b, EITHER_EDGE, ENCODER_EVENT_TIMEOUT, encoder_event_callback, (void *)encoder);
    return encoder_reset(encoder);
}

int encoder_reset(Encoder_t *encoder){
    pthread_mutex_lock(encoder->mutex);
    encoder->count = ENCODER_DEFAULT_TICK_RESET;
    encoder->prev_count = ENCODER_DEFAULT_TICK_RESET;
    pthread_mutex_unlock(encoder->mutex);
    return SUCCESS;
}


float encoder_get_rotations(Encoder_t *encoder){
    pthread_mutex_lock(encoder->mutex);
    float rotations = ((float)encoder->count) * encoder->ratio;
    pthread_mutex_unlock(encoder->mutex);
    return rotations;
}

float encoder_get_angle_degrees(Encoder_t *encoder){
    return encoder_get_rotations(encoder) * 360.0F;
}

float encoder_get_angle_radians(Encoder_t *encoder){
    return encoder_get_rotations(encoder) * 2.0F*M_PI;
}


float encoder_refresh_rpm(Encoder_t *encoder, uint32_t current_us){
    //TODO: Call this function at a set Freq, not based on encoder events.
    pthread_mutex_lock(encoder->mutex);
    encoder->rpm = ((float)(encoder->count - encoder->prev_count) / (float)(current_us - encoder->prev_us)) * 60000000.0F * encoder->ratio;
    encoder->prev_count = encoder->count;
    encoder->prev_us = current_us;
    float sum = 0;
    for(int i = ENCODER_RPM_BUFFER_SIZE-1; i >= 1; i--){
        encoder->prev_rpm[i] = encoder->prev_rpm[i-1];
        sum += encoder->prev_rpm[i];
    }
    encoder->prev_rpm[0] = encoder->rpm;
    sum += encoder->prev_rpm[0];
    encoder->avg_rpm = sum/ENCODER_RPM_BUFFER_SIZE;
    encoder->rpm = encoder->avg_rpm;
    float rpm = encoder->rpm;
    pthread_mutex_unlock(encoder->mutex);
    return rpm;
}

void encoder_event_callback(int gpio, int level, uint32_t current_us, void *data){
    Encoder_t *encoder = (Encoder_t *) data;
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

    // Refresh RPM every Encoder rotation of the encoder. (Subject to change)
    if(encoder->count % ENCODER_RPM_REFRESH_RATE == 0){
        encoder_refresh_rpm(encoder, current_us); 
    }
    return;
}
/*---ENCODER_C---*/