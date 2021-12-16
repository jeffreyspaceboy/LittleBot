/*---ENCODER_H---*/
#ifndef ENCODER_H
#define ENCODER_H
/*----------------------------------------------------------------------------*/
/*    Module:       Encoder.h                                                 */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-15                                                */
/*----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

/* LOCAL INCLUDES */
#include "Definitions.h"

/* STANDARD INCLUDES */
#include <stdint.h>
#include <pthread.h>


/** @brief ENCODER TYPE - used to define a dual phase encoder.
 * @param name Name used for DEBUG printf
 * @param gpio_phase_a GPIO pin for phase A
 * @param gpio_phase_b GPIO pin for phase B
 * @param prev_gpio The most recent phase to change state
 * @param level_phase_a Most recent detected state of encoder phase A
 * @param level_phase_b Most recent detected state of encoder phase B
 * @param count The sum of all phase state changes of the encoder. aka Encoder Ticks
 * @param prev_count The most recent count value. Used for rps calculation
 * @param prev_us The time in usec from boot to the most recent rps measurement
 * @param rps Rotations Per Second
 * @param avg_rps RPS average over the prev_rps values
 * @param prev_rps RPS recorded from the previous ENCODER_RPS_BUFFER_SIZE rps readings
 * @param ratio The ratio from encoder ticks to the desired axis of rotation. aka Wheel Rotation Per Encoder Ticks
*/
typedef struct Encoder_t{
    char name[NAME_MAX_SIZE];
    uint8_t gpio_phase_a, gpio_phase_b, prev_gpio; 
    int level_phase_a, level_phase_b, count, prev_count;
    uint32_t prev_us;
    float rpm, avg_rpm, ratio;
    float prev_rpm[ENCODER_RPM_BUFFER_SIZE];
    pthread_mutex_t mutex;
} Encoder_t;


/** @brief Encoder initialization.
 * @param encoder_name Name for DEBUG
 * @param gpio_phase_a_pin GPIO Phase A Pin
 * @param gpio_phase_b_pin GPIO Phase B Pin
 * @param encoder_ratio Ratio of: Output (Wheel) Rotations per Encoder Ticks
 * @param reverse Boolean to reverse Phase A & B
 * @return Encoder */
Encoder_t encoder_init(char encoder_name[NAME_MAX_SIZE], uint8_t gpio_phase_a_pin, uint8_t gpio_phase_b_pin, float encoder_ratio, int reverse);

/** @brief Encoder destruction.
 * @param encoder Encoder you want to delete 
 * @return int: SUCCESS or FAILURE */
int encoder_del(Encoder_t *encoder);


/** @brief Enables GPIO interupts for Phase A & B, and reset encoder count.
 * @param encoder Encoder to be started
 * @return int: SUCCESS or FAILURE */
int encoder_start(Encoder_t *encoder);

/** @brief Sets Encoder.count & Encoder.prev_count to 0.
 * @param encoder Encoder to be reset
 * @return int: SUCCESS or FAILURE */
int encoder_reset(Encoder_t *encoder);


/** @brief Calculates rotations using count * ratio.
 * @param encoder Encoder to get rotations from 
 * @return float: Rotations */
float encoder_get_rotations(Encoder_t *encoder);

/** @brief Calculates angle in degrees using count * ratio * 360.
 * @param encoder Encoder to get angel from 
 * @return float: Angle in degrees */
float encoder_get_angle_degrees(Encoder_t *encoder);

/** @brief Calculates angle in radians using count * ratio * 2 * PI.
 * @param encoder Encoder to get angel from 
 * @return float: Angle in radians */
float encoder_get_angle_radians(Encoder_t *encoder);


/** @brief Updates encoder RPS.
 * @param encoder Encoder to be refreshed
 * @return float: New RPS */
float encoder_refresh_rpm(Encoder_t *encoder);

/** @brief Phase Interupt Event Callback. (Called by gpioSetISRFuncEx when Encoder Phases change)
 * @param gpio GPIO that caused event
 * @param level Phase Level (HIGH or LOW)
 * @param tick Time (usec) of the encoder phase change
 * @param data Encoder Pointer*/
void encoder_event_callback(int gpio, int level, uint32_t tick, void *data);

void *encoder_control_thread(void *arg);


#ifdef __cplusplus
}
#endif
#endif
/*---ENCODER_H---*/