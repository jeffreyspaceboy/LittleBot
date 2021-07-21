/*---DRIVETRAIN_HPP---*/
#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP
/*----------------------------------------------------------------------------*/
/*    Module:       Drivetrain.hpp                                            */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2020-07-21                                                */
/*----------------------------------------------------------------------------*/
#include "Motor.hpp"

class Drivetrain{
    private:
        Motor left_motor;
        Motor right_motor;
    public:
        Drivetrain();
        Drivetrain(Motor left_motor, Motor right_motor);
        void drive(int velocity);
        void turn(int velocity);
        void stop();
};
#endif
/*---DRIVETRAIN_HPP---*/