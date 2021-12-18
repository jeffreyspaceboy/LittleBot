/*---MOTOR_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       Motor.c                                                   */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-15                                                */
/*----------------------------------------------------------------------------*/

/* LOCAL INCLUDES */
#include "Motor.h"

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
        .enabled = true,
        .name = "",
        .gpio_enable = gpio_enable_pin,
        .gpio_phase_a = (reverse == 0) ? gpio_phase_a_pin : gpio_phase_b_pin,
        .gpio_phase_b = (reverse == 0) ? gpio_phase_b_pin : gpio_phase_a_pin,
        .rpm_target = 0.0F,
        .power = 0,
        .max_power = MOTOR_DEFAULT_MAX_POWER,
        .encoder = new_encoder,
        .pid_velocity_controller = new_pid_velocity_controller,
    };
    if(pthread_mutex_init(&new_motor.mutex, NULL) != 0){
        printf("%s(%s) Motor mutex init failed.\n",ERROR_MSG,motor_name);
        return new_motor;
    }
    pthread_mutex_lock(&new_motor.mutex);
    strncpy(new_motor.name, motor_name, sizeof(new_motor.name));
    gpioSetMode(new_motor.gpio_enable, PI_OUTPUT);
    gpioSetMode(new_motor.gpio_phase_a, PI_OUTPUT);
    gpioSetMode(new_motor.gpio_phase_b, PI_OUTPUT);
    pthread_mutex_unlock(&new_motor.mutex);
    motor_stop(&new_motor);
    if(new_motor.encoder != NULL){ 
        encoder_create_thread(new_motor.encoder);
        encoder_start(new_motor.encoder); 
    }
    pthread_mutex_unlock(&new_motor.mutex);
    return new_motor;
}

int motor_create_thread(Motor_t *motor){
    pthread_create(&motor->thread, NULL, motor_control_thread, (void *)motor);
    return SUCCESS;
}

int motor_del(Motor_t *motor){ 
    pthread_mutex_lock(&motor->mutex);
    motor->enabled = false;
    pthread_mutex_unlock(&motor->mutex);
    if(motor->encoder != NULL){ 
        encoder_del(motor->encoder); 
    }
    motor_set_rpm(motor, 0);
    pthread_join(motor->thread, NULL);
    motor_stop(motor);
    pthread_mutex_destroy(&motor->mutex);
    return SUCCESS;
}


/* SET FUNCTIONS */

int motor_set_max_power(Motor_t *motor, int new_max_power){
    pthread_mutex_lock(&motor->mutex);
    motor->max_power = new_max_power;
    pthread_mutex_unlock(&motor->mutex);
    return new_max_power;
}

float motor_set_rpm(Motor_t *motor, float rpm_target){
    pthread_mutex_lock(&motor->mutex);
    motor->rpm_target = rpm_target;
    pthread_mutex_unlock(&motor->mutex);
    return rpm_target;
}


/* GET FUNCTIONS */

float motor_get_rotations(Motor_t *motor){
    if(motor->encoder == NULL){ 
        pthread_mutex_lock(&motor->mutex);
        printf("%s(%s) This motor has not been setup with an Encoder.\n", ERROR_MSG, motor->name); 
        pthread_mutex_unlock(&motor->mutex);
        return FAILURE; 
    }
    float rotations = encoder_get_rotations(motor->encoder);
    return rotations;
}

float motor_get_angle_degrees(Motor_t *motor){
    return encoder_get_angle_degrees(motor->encoder);
}

float motor_get_angle_radians(Motor_t *motor){
    return encoder_get_angle_radians(motor->encoder);
}

float motor_get_rpm(Motor_t *motor){
    if(motor->encoder == NULL){ 
        pthread_mutex_lock(&motor->mutex);
        printf("%s(%s) This motor has not been setup with an Encoder.\n", ERROR_MSG, motor->name); 
        pthread_mutex_unlock(&motor->mutex);
        return FAILURE;
    }
    return motor->encoder->rpm;
}

int motor_get_power(Motor_t *motor){
    pthread_mutex_lock(&motor->mutex);
    int power = motor->power;
    pthread_mutex_unlock(&motor->mutex);
    return power;
}


/* MOTION FUNCTIONS */

int motor_spin(Motor_t *motor, int power){
    pthread_mutex_lock(&motor->mutex);
    motor->power = power;
    if(abs(power) > motor->max_power){
        gpioPWM(motor->gpio_enable, motor->max_power);
    }else{
        gpioPWM(motor->gpio_enable, abs(power));
    }

    if(power < 0){
        gpioWrite(motor->gpio_phase_a, 0);
        gpioWrite(motor->gpio_phase_b, 1);
    }else if(power > 0){
        gpioWrite(motor->gpio_phase_a, 1);
        gpioWrite(motor->gpio_phase_b, 0);
    }else{
        gpioWrite(motor->gpio_phase_a, 0);
        gpioWrite(motor->gpio_phase_b, 0);
    }
    pthread_mutex_unlock(&motor->mutex);
    return SUCCESS;
}

int motor_stop(Motor_t *motor){
    pthread_mutex_lock(&motor->mutex);
    gpioWrite(motor->gpio_enable, 0);
    gpioWrite(motor->gpio_phase_a, 0);
    gpioWrite(motor->gpio_phase_b, 0);
    pthread_mutex_unlock(&motor->mutex);
    return SUCCESS;
}

void *motor_control_thread(void *arg){
    Motor_t *motor = (Motor_t *) arg;
    if(motor->pid_velocity_controller == NULL){ 
        pthread_mutex_lock(&motor->mutex);
        printf("%s(%s) This motor has not been setup with a PID Velocity Controller.\n", ERROR_MSG, motor->name); 
        motor_stop(motor);
        pthread_mutex_unlock(&motor->mutex);
        return NULL; 
    }
    while(motor->enabled){
        pthread_mutex_lock(&motor->mutex);
        if(!motor->pid_velocity_controller->enabled){ pid_start(motor->pid_velocity_controller, motor->rpm_target, 0.0); }
        int power = (int)pid_power(motor->pid_velocity_controller, motor_get_rpm(motor));
        pthread_mutex_unlock(&motor->mutex);
        motor_spin(motor, power);
        gpioSleep(PI_TIME_RELATIVE, 0, MOTOR_REFRESH_RATE);
    }
    motor_stop(motor);
}
/*---MOTOR_C---*/