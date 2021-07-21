/*---MOTOR_HPP---*/
#ifndef MOTOR_HPP
#define MOTOR_HPP
/*----------------------------------------------------------------------------*/
/*    Module:       Motor.hpp                                                 */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2020-07-21                                                */
/*----------------------------------------------------------------------------*/
#include <stdint.h>

class Motor{
    private:
        uint8_t gpio_speed, gpio_dir_1, gpio_dir_2, gpio_enc_a, gpio_enc_b;
        int max_speed = 255;
    public: 
        Motor();
        Motor(uint8_t gpio_speed, uint8_t gpio_dir_1, uint8_t gpio_dir_2, uint8_t gpio_enc_a, uint8_t gpio_enc_b);
        ~Motor();
        void spin(int velocity);
        void stop();
        void set_max_speed(int new_max_speed);
};
#endif
/*---MOTOR_HPP---*/