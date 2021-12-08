/*---MOTOR_H---*/
#ifndef MOTOR_H
#define MOTOR_H
/*----------------------------------------------------------------------------*/
/*    Module:       Motor.h                                                   */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-06                                                */
/*----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#include "Definitions.h"
#include "Encoder.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct{
    char name[NAME_MAX_SIZE];
    uint8_t gpio_enable, gpio_phase_a, gpio_phase_b;
    int32_t max_power;
    Encoder *encoder;
} Motor;

Motor motor_init(char motor_name[NAME_MAX_SIZE], uint8_t gpio_enable_pin, uint8_t gpio_phase_a_pin, uint8_t gpio_phase_b_pin, bool reverse);
int32_t motor_del(Motor *motor);

int32_t motor_link_encoder(Motor *motor, Encoder *new_encoder, double encoder_to_motor_ratio);

int32_t motor_spin(Motor *motor, int32_t power);
int32_t motor_stop(Motor *motor);
int32_t motor_set_max_power(Motor *motor, int32_t new_max_power);

#ifdef __cplusplus
}
#endif
#endif
/*---MOTOR_H---*/