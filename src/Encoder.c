/*---ENCODER_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       Encoder.c                                                 */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-21                                                */
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
        .enabled = true,
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
    if(pthread_mutex_init(&new_encoder.mutex, NULL) != 0){
        printf("%s(%s) Encoder mutex init failed.\n",ERROR_MSG,encoder_name);
        return new_encoder;
    }
    pthread_mutex_lock(&new_encoder.mutex);
    strncpy(new_encoder.name, encoder_name, sizeof(new_encoder.name));
    for(int i = 0; i < ENCODER_RPM_BUFFER_SIZE; i++){ new_encoder.prev_rpm[i] = 0.0F; }
    gpioSetMode(new_encoder.gpio_phase_a, PI_INPUT);
    gpioSetMode(new_encoder.gpio_phase_b, PI_INPUT);
    gpioSetPullUpDown(new_encoder.gpio_phase_a, PI_PUD_UP);
    gpioSetPullUpDown(new_encoder.gpio_phase_b, PI_PUD_UP);
    pthread_mutex_unlock(&new_encoder.mutex);
    return new_encoder;
}

int encoder_del(Encoder_t *encoder){
    pthread_mutex_lock(&encoder->mutex);
    encoder->enabled = false;
    pthread_mutex_unlock(&encoder->mutex);
    pthread_join(encoder->thread, NULL);
    pthread_mutex_destroy(&encoder->mutex);
    return SUCCESS;
}

int encoder_start(Encoder_t *encoder){
    pthread_mutex_lock(&encoder->mutex);
    gpioSetISRFuncEx(encoder->gpio_phase_a, EITHER_EDGE, ENCODER_EVENT_TIMEOUT, encoder_tick_event_callback, (void *)encoder);
    gpioSetISRFuncEx(encoder->gpio_phase_b, EITHER_EDGE, ENCODER_EVENT_TIMEOUT, encoder_tick_event_callback, (void *)encoder);
    pthread_mutex_unlock(&encoder->mutex);
    return encoder_reset(encoder);
}

int encoder_create_thread(Encoder_t *encoder){
    return pthread_create(&encoder->thread, NULL, encoder_rpm_control_thread, (void *)encoder);
}

int encoder_reset(Encoder_t *encoder){
    pthread_mutex_lock(&encoder->mutex);
    encoder->count = ENCODER_DEFAULT_TICK_RESET;
    encoder->prev_count = ENCODER_DEFAULT_TICK_RESET;
    pthread_mutex_unlock(&encoder->mutex);
    return SUCCESS;
}

float encoder_sense_rotations(Encoder_t *encoder){
    pthread_mutex_lock(&encoder->mutex);
    float rotations = ((float)encoder->count) * encoder->ratio;
    pthread_mutex_unlock(&encoder->mutex);
    return rotations;
}

float encoder_sense_angle_degrees(Encoder_t *encoder){
    return encoder_sense_rotations(encoder) * 360.0F;
}

float encoder_sense_angle_radians(Encoder_t *encoder){
    return encoder_sense_rotations(encoder) * 2.0F*M_PI;
}

float encoder_sense_rpm(Encoder_t *encoder){
    pthread_mutex_lock(&encoder->mutex);
    float rpm = encoder->rpm;
    pthread_mutex_unlock(&encoder->mutex);
    return rpm;
}

float encoder_refresh_rpm(Encoder_t *encoder){
    uint32_t current_us = gpioTick();
    pthread_mutex_lock(&encoder->mutex);
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
    float rpm = encoder->rpm = sum/ENCODER_RPM_BUFFER_SIZE;
    pthread_mutex_unlock(&encoder->mutex);
    return rpm;
}

/* 
Encoder Tick Counting Logic:
   There are four possible states that can occur on any given level
   shift of the encoder Phases. 

   NOTE: Dependant on knowing the shift in level, not just H/L state.
   This allows for every level shift to count as a tick.

CLOCK WISE:
A&!B B&A !A&B !B&A A&!B B&A !A&B !B&!A
    ________          ________
   |    ____|___     |    ____|___
A__|   |    |___|____|   |    |___|__
B______|        |________|        |__

COUNTER CLOCK WISE:
B&!A A&B !B&A !A&!B B&!A A&B !B&A !A&!B
        ________          ________
    ___|____    |     ___|____    |
A__|___|    |   |____|___|    |   |__
B__|        |________|        |______
*/
void encoder_tick_event_callback(int gpio, int level, uint32_t current_us, void *data){
    Encoder_t *encoder = (Encoder_t *) data;
    pthread_mutex_lock(&encoder->mutex);
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
    pthread_mutex_unlock(&encoder->mutex);
}

void *encoder_rpm_control_thread(void *arg){
    Encoder_t *encoder = (Encoder_t *) arg;
    while(encoder->enabled){
        encoder_refresh_rpm(encoder);
        gpioSleep(PI_TIME_RELATIVE, 0, ENCODER_REFRESH_RATE);
    }
}
/*---ENCODER_C---*/