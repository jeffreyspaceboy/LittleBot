/*---MOTOR_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       Motor.c                                                   */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-06                                                */
/*----------------------------------------------------------------------------*/
#include "../include/Motor.h"

#include <pigpio.h>
#include <stdio.h>
#include <string.h>

Motor motor_init(char motor_name[NAME_MAX_SIZE], uint8_t gpio_enable_pin, uint8_t gpio_phase_a_pin, uint8_t gpio_phase_b_pin, bool reverse){
    Motor new_motor = {
        .name = "",
        .gpio_enable = gpio_enable_pin,
        .gpio_phase_a = (reverse == false) ? gpio_phase_a_pin : gpio_phase_b_pin,
        .gpio_phase_b = (reverse == false) ? gpio_phase_b_pin : gpio_phase_a_pin,
        .max_power = MOTOR_DEFAULT_MAX_POWER,
        .encoder = NULL,
    };
    strncpy(new_motor.name, motor_name, sizeof(new_motor.name));
    gpioSetMode(new_motor.gpio_enable, PI_OUTPUT);
    gpioSetMode(new_motor.gpio_phase_a, PI_OUTPUT);
    gpioSetMode(new_motor.gpio_phase_b, PI_OUTPUT);
    motor_stop(&new_motor);
    return new_motor;
}

int motor_del(Motor *motor){ 
    return encoder_del(motor->encoder) || motor_stop(motor);  
}

int motor_link_encoder(Motor *motor, Encoder *new_encoder){
    if(motor->encoder != NULL){
        printf("%s Motor %s replacing existing encoder %s with new encoder %s.\n",WARNING_MSG, motor->name, motor->encoder->name, new_encoder->name);
    }
    motor->encoder = new_encoder;
    return SUCCESS;
}

int motor_spin(Motor *motor, int power){
    if(power == 0){
        return motor_stop(motor);
    }else if(power > 0){
        gpioWrite(motor->gpio_phase_a, 1);
        gpioWrite(motor->gpio_phase_b, 0);
    }else{
        gpioWrite(motor->gpio_phase_a, 0);
        gpioWrite(motor->gpio_phase_b, 1);
    }

    if(power > motor->max_power){
        gpioPWM(motor->gpio_enable, motor->max_power);
    }else if(power < -motor->max_power){
        gpioPWM(motor->gpio_enable, -motor->max_power);
    }else{
        gpioPWM(motor->gpio_enable, power);
    }
    return SUCCESS;
}

int motor_stop(Motor *motor){
    gpioWrite(motor->gpio_phase_a, 0);
    gpioWrite(motor->gpio_phase_b, 0);
    gpioWrite(motor->gpio_enable, 0);
    return SUCCESS;
}

int motor_set_max_power(Motor *motor, int new_max_power){
    motor->max_power = new_max_power;
    return new_max_power;
}
/*---MOTOR_C---*/