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
#include <pthread.h>


/** @brief MOTOR TYPE - used to define a dual phase motor w/ encoder.
 * @param enabled Controls the looping of the motor control thread
 * @param name Name used for DEBUG printf
 * @param gpio_enable GPIO pin for motor enable/speed
 * @param gpio_phase_a GPIO pin for phase A
 * @param gpio_phase_b GPIO pin for phase B
 * @param rpm_target Target RPM to be maintained by the motor control thread
 * @param prev_target_rpm This is used to check if the target has changed to reset the PID controller
 * @param power the most recent power value set to the motor
 * @param max_power Max power input to the motor
 * @param encoder The encoder connected to the motor
 * @param pid_velocity_controller A PID controller for velocity control of the motor
 * @param thread Pthread for the motor control thread
 * @param mutex Mutex used to control locking of data
 */
typedef struct Motor_t{
    bool rpm_control_enabled;
    char name[NAME_MAX_SIZE];
    uint8_t gpio_enable, gpio_phase_a, gpio_phase_b;
    float rpm_target, prev_target_rpm;
    int power, max_power;
    Encoder_t *encoder;
    PID_Controller_t *pid_velocity_controller;
    pthread_t thread;
    pthread_mutex_t mutex;
} Motor_t;


/* MOTOR SETUP FUNCTIONS */

/** @brief Motor initialization.
 * @param encoder_name Name for DEBUG
 * @param gpio_enable_pin GPIO Motor Enable Pin
 * @param gpio_phase_a_pin GPIO Phase A Pin
 * @param gpio_phase_b_pin GPIO Phase B Pin
 * @param reverse Bolean to reverse Phase A & B
 * @param new_encoder Encoder connected to the motor
 * @param new_pid_velocity_controller Velocity PID controller to control the motor
 * @return Motor */
Motor_t motor_init(char motor_name[NAME_MAX_SIZE], uint8_t gpio_enable_pin, uint8_t gpio_phase_a_pin, uint8_t gpio_phase_b_pin, int reverse, Encoder_t *new_encoder, PID_Controller_t *new_pid_velocity_controller);


int motor_rpm_control_enable(Motor_t *motor);
int motor_rpm_control_disable(Motor_t *motor);

int motor_create_rpm_control_thread(Motor_t *motor);

/** @brief Motor & Encoder destruction.
 * @param motor Motor to be deleted
 * @return int: SUCCESS or FAILURE */
int motor_del(Motor_t *motor);


/* SET FUNCTIONS */

/** @brief Set the max power of the motor.
 * @param motor Motor to set max power to
 * @param new_max_power Max power to be set
 * @return int: SUCCESS or FAILURE */
int motor_set_max_power(Motor_t *motor, int new_max_power);

/** @brief Sets velocity to be used by the motor control thread.
 * @param motor Motor to control
 * @param rps_target RPM target
 * @return float: RPM that was set by the PID controller */
float motor_set_rpm(Motor_t *motor, float rpm_target);


/* GET FUNCTIONS */

/** @brief Gets rotations from the Encoder.
 * @param motor Motor to get rotations from 
 * @return float: Rotations */
float motor_sense_rotations(Motor_t *motor);

/** @brief Gets angle in degrees from the Encoder.
 * @param motor Motor to get angel from 
 * @return float: Angle in degrees */
float motor_sense_angle_degrees(Motor_t *motor);

/** @brief Gets angle in radians from the Encoder.
 * @param motor Motor to get angel from 
 * @return float: Angle in radians */
float motor_sense_angle_radians(Motor_t *motor);

/** @brief Gets rotations per second from the Encoder.
 * @param motor Motor to get RPS from
 * @return float: Motor RPS*/
float motor_sense_rpm(Motor_t *motor);

/** @brief Gets the most recent power set to the Motor.
 * @param motor Motor to get power from 
 * @return int: Most recent power set to the motot
 */
int motor_get_power(Motor_t *motor);


/* MOTION FUNCTIONS */

/** @brief Spin the Motor at a given power.
 * @param motor Motor to spin
 * @param power Power to spin the motor at
 * @return int: SUCCESS or FAILURE */
int motor_spin(Motor_t *motor, int power);

/** @brief Stop the Motor.
 * @param motor Motor to be stopped
 * @return int: SUCCESS or FAILURE */
int motor_stop(Motor_t *motor);

/** @brief Motor Control Thread. Uses PID to control the motor constantly.
 * @param arg To pass Motor pointer as arg
 * @return void*: NULL */
void *motor_rpm_control_thread(void *arg);

#ifdef __cplusplus
}
#endif
#endif
/*---MOTOR_H---*/