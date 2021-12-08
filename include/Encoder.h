/*---ENCODER_H---*/
#ifndef ENCODER_H
#define ENCODER_H
/*----------------------------------------------------------------------------*/
/*    Module:       Encoder.h                                                 */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-07                                                */
/*----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#include "Definitions.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct{
    char name[NAME_MAX_SIZE];
    uint8_t gpio_phase_a, gpio_phase_b;
    volatile int32_t prev_gpio, level_phase_a, level_phase_b;
    volatile int64_t ticks, prev_ticks;
    volatile uint32_t prev_tick_us;
    volatile double rpm;
    double ratio;
} Encoder;

typedef void (*gpioISRFuncEx_t)(int gpio, int level, uint32_t tick, void *data);

Encoder encoder_init(char encoder_name[NAME_MAX_SIZE], uint8_t gpio_phase_a_pin, uint8_t gpio_phase_b_pin, float encoder_ratio, bool reverse);
int32_t encoder_del(Encoder *encoder);
int32_t encoder_reset(Encoder *encoder);

double encoder_refresh_rpm(Encoder *encoder, uint32_t current_tick_us);

static void encoder_event_callback(int gpio, int level, uint32_t tick, void *data);

#ifdef __cplusplus
}
#endif
#endif
/*---ENCODER_H---*/