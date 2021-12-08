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
    int max_power;
    Encoder *encoder;
} Motor;

Motor motor_init(char motor_name[NAME_MAX_SIZE], uint8_t gpio_enable_pin, uint8_t gpio_phase_a_pin, uint8_t gpio_phase_b_pin, bool reverse);
int motor_del(Motor *motor);

int motor_link_encoder(Motor *motor, Encoder *new_encoder);

int motor_spin(Motor *motor, int power);
int motor_stop(Motor *motor);
int motor_set_max_power(Motor *motor, int new_max_power);

#ifdef __cplusplus
}
#endif
#endif
/*---MOTOR_H---*/