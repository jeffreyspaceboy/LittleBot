/*---MOTOR_H---*/
#ifndef MOTOR_H
#define MOTOR_H
/*----------------------------------------------------------------------------*/
/*    Module:       Motor.h                                                   */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-11                                                */
/*----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

/* LOCAL INCLUDES */
#include "Definitions.h"
#include "Encoder.h"
#include "PID.h"

/* STANDARD INCLUDES */
#include <stdint.h>


/** @brief MOTOR TYPE - used to define a dual phase motor w/ encoder.
 * @param name Name used for DEBUG printf
 * @param gpio_enable GPIO pin for motor enable/speed
 * @param gpio_phase_a GPIO pin for phase A
 * @param gpio_phase_b GPIO pin for phase B
 * @param max_power Max power input to the motor
 * @param Encoder The encoder connected to the motor
 */
typedef struct Motor_t{
    char name[NAME_MAX_SIZE];
    uint8_t gpio_enable, gpio_phase_a, gpio_phase_b;
    int max_power;
    Encoder_t *encoder;
    //TODO: ADD PID CONTROLLER
} Motor_t;


/** @brief Motor initialization.
 * @param encoder_name Name for DEBUG
 * @param gpio_enable_pin GPIO Motor Enable Pin
 * @param gpio_phase_a_pin GPIO Phase A Pin
 * @param gpio_phase_b_pin GPIO Phase B Pin
 * @param new_encoder Encoder connected to the motor
 * @param reverse Bolean to reverse Phase A & B
 * @return Motor */
Motor_t motor_init(char motor_name[NAME_MAX_SIZE], uint8_t gpio_enable_pin, uint8_t gpio_phase_a_pin, uint8_t gpio_phase_b_pin, Encoder_t *new_encoder, int reverse);

/** @brief Motor & Encoder destruction.
 * @param motor Motor to be deleted
 * @return int: SUCCESS or FAILURE */
int motor_del(Motor_t *motor);


/** @brief Spin the Motor at a given power.
 * @param motor Motor to spin
 * @param power Power to spin the motor at
 * @return int: SUCCESS or FAILURE */
int motor_spin(Motor_t *motor, int power);

int motor_pid_velocity(Motor_t *motor, PID_Controller_t* pid, float rps_target);

/** @brief Stop the Motor.
 * @param motor Motor to be stopped
 * @return int: SUCCESS or FAILURE */
int motor_stop(Motor_t *motor);

/** @brief Set the max power of the motor.
 * @param motor Motor to set max power to
 * @param new_max_power Max power to be set
 * @return int: SUCCESS or FAILURE */
int motor_set_max_power(Motor_t *motor, int new_max_power);


/** @brief Gets rotations from the Encoder.
 * @param motor Motor to get rotations from 
 * @return float: Rotations */
float motor_get_rotations(Motor_t *motor);

/** @brief Gets angle in degrees from the Encoder.
 * @param motor Motor to get angel from 
 * @return float: Angle in degrees */
float motor_get_angle_degrees(Motor_t *motor);

/** @brief Gets angle in radians from the Encoder.
 * @param motor Motor to get angel from 
 * @return float: Angle in radians */
float motor_get_angle_radians(Motor_t *motor);

/** @brief Gets rotations per second from the Encoder.
 * @param motor Motor to get RPS from
 * @return float: Motor RPS*/
float motor_get_rps(Motor_t *motor);


#ifdef __cplusplus
}
#endif
#endif
/*---MOTOR_H---*/