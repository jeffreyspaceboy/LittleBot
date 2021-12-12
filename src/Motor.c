/*---MOTOR_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       Motor.c                                                   */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-12                                                */
/*----------------------------------------------------------------------------*/

/* LOCAL INCLUDES */
#include "../include/Motor.h"

/* NON-STANDARD INCLUDES */
#include <pigpio.h>

/* STANDARD INCLUDES */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>


/* MOTOR SETUP FUNCTIONS */

Motor_t motor_init(char motor_name[NAME_MAX_SIZE], uint8_t gpio_enable_pin, uint8_t gpio_phase_a_pin, uint8_t gpio_phase_b_pin, int reverse, Encoder_t *new_encoder, PID_Controller_t *new_pid_velocity_controller){
    Motor_t new_motor = {
        .name = "",
        .gpio_enable = gpio_enable_pin,
        .gpio_phase_a = (reverse == 0) ? gpio_phase_a_pin : gpio_phase_b_pin,
        .gpio_phase_b = (reverse == 0) ? gpio_phase_b_pin : gpio_phase_a_pin,
        .max_power = MOTOR_DEFAULT_MAX_POWER,
        .encoder = new_encoder,
        .pid_velocity_controller = new_pid_velocity_controller,
    };
    strncpy(new_motor.name, motor_name, sizeof(new_motor.name));
    gpioSetMode(new_motor.gpio_enable, PI_OUTPUT);
    gpioSetMode(new_motor.gpio_phase_a, PI_OUTPUT);
    gpioSetMode(new_motor.gpio_phase_b, PI_OUTPUT);
    motor_stop(&new_motor);
    if(new_motor.encoder != NULL){ encoder_start(new_motor.encoder); }
    return new_motor;
}

int motor_del(Motor_t *motor){ 
    if(motor->encoder != NULL){ encoder_del(motor->encoder); }
    return motor_stop(motor);  
}


/* SET FUNCTIONS */

int motor_set_max_power(Motor_t *motor, int new_max_power){
    motor->max_power = new_max_power;
    return new_max_power;
}


/* GET FUNCTIONS */

float motor_get_rotations(Motor_t *motor){
    if(motor->encoder == NULL){ 
        printf("%s(%s) This motor has not been setup with an Encoder.\n", ERROR_MSG, motor->name); 
        return FAILURE; 
    }
    return encoder_get_rotations(motor->encoder);
}

float motor_get_angle_degrees(Motor_t *motor){
    return encoder_get_angle_degrees(motor->encoder);
}

float motor_get_angle_radians(Motor_t *motor){
    return encoder_get_angle_radians(motor->encoder);
}

float motor_get_rpm(Motor_t *motor){
    if(motor->encoder == NULL){ 
        printf("%s(%s) This motor has not been setup with an Encoder.\n", ERROR_MSG, motor->name); 
        return FAILURE; 
    }
    return motor->encoder->rpm;
}


/* MOTION FUNCTIONS */

int motor_spin(Motor_t *motor, int power){
    if(abs(power) > motor->max_power){
        gpioPWM(motor->gpio_enable, motor->max_power);
    }else{
        gpioPWM(motor->gpio_enable, abs(power));
    }

    if(power == 0){
        return motor_stop(motor);
    }else if(power > 0){
        gpioWrite(motor->gpio_phase_a, 1);
        gpioWrite(motor->gpio_phase_b, 0);
    }else{
        gpioWrite(motor->gpio_phase_a, 0);
        gpioWrite(motor->gpio_phase_b, 1);
    }
    return SUCCESS;
}

float motor_set_rpm(Motor_t *motor, float rpm_target){
    if(motor->pid_velocity_controller == NULL){ 
        printf("%s(%s) This motor has not been setup with a PID Velocity Controller.\n", ERROR_MSG, motor->name); 
        motor_spin(motor, 0);
        return 0; 
    }
    if(!motor->pid_velocity_controller->enabled){ pid_start(motor->pid_velocity_controller, rpm_target, 0.0); }
    float power = pid_power(motor->pid_velocity_controller, motor_get_rpm(motor));
    motor_spin(motor, (int)power);
    return power;
}

int motor_stop(Motor_t *motor){
    gpioWrite(motor->gpio_enable, 0);
    gpioWrite(motor->gpio_phase_a, 0);
    gpioWrite(motor->gpio_phase_b, 0);
    return SUCCESS;
}

/*---MOTOR_C---*/