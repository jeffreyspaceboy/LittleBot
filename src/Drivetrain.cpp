/*---DRIVETRAIN_CPP---*/
/*----------------------------------------------------------------------------*/
/*    Module:       Drivetrain.cpp                                            */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2020-07-21                                                */
/*----------------------------------------------------------------------------*/
#include "../include/Drivetrain.hpp"

#define LEFT_MOTOR_SPEED 2
#define LEFT_MOTOR_DIR_1 4
#define LEFT_MOTOR_DIR_2 3
#define LEFT_ENCODER_A 27
#define LEFT_ENCODER_B 17

#define RIGHT_MOTOR_SPEED 13
#define RIGHT_MOTOR_DIR_1 6
#define RIGHT_MOTOR_DIR_2 5
#define RIGHT_ENCODER_A 26
#define RIGHT_ENCODER_B 19

Drivetrain::Drivetrain(){
    left_motor.gpio_setup(LEFT_MOTOR_SPEED, LEFT_MOTOR_DIR_1, LEFT_MOTOR_DIR_2, LEFT_ENCODER_A, LEFT_ENCODER_B);
    right_motor.gpio_setup(RIGHT_MOTOR_SPEED, RIGHT_MOTOR_DIR_1, RIGHT_MOTOR_DIR_2, RIGHT_ENCODER_A, RIGHT_ENCODER_B);
}

Drivetrain::Drivetrain(Motor left_motor, Motor right_motor){
    this->left_motor = left_motor;
    this->right_motor = right_motor;
}

void Drivetrain::drive(int velocity){
    this->left_motor.spin(velocity);
    this->right_motor.spin(velocity);
}

void Drivetrain::turn(int velocity){
    this->left_motor.spin(velocity);
    this->right_motor.spin(-velocity);
}

void Drivetrain::stop(){
    this->left_motor.stop();
    this->right_motor.stop();
}
/*---DRIVETRAIN_CPP---*/