// /*---ROBOT_CPP---*/
// /*----------------------------------------------------------------------------*/
// /*    Module:       Robot.cpp                                                 */
// /*    Author:       Jeffrey Fisher II                                         */
// /*    Created:      23 Dec 2020                                               */
// /*----------------------------------------------------------------------------*/
// #include "Motor.hpp"

// #define LEFT_MOTOR_SPEED 2
// #define LEFT_MOTOR_DIR_1 4
// #define LEFT_MOTOR_DIR_2 3
// #define LEFT_ENCODER_A 27
// #define LEFT_ENCODER_B 17

// #define RIGHT_MOTOR_SPEED 13
// #define RIGHT_MOTOR_DIR_1 6
// #define RIGHT_MOTOR_DIR_2 5
// #define RIGHT_ENCODER_A 26
// #define RIGHT_ENCODER_B 19

// class Drivetrain{
//   private:
//     Motor left_motor(LEFT_MOTOR_SPEED, LEFT_MOTOR_DIR_1, LEFT_MOTOR_DIR_2, LEFT_ENCODER_A, LEFT_ENCODER_B);
//     Motor right_motor(RIGHT_MOTOR_SPEED, RIGHT_MOTOR_DIR_1, RIGHT_MOTOR_DIR_2, RIGHT_ENCODER_A, RIGHT_ENCODER_B);
//   public:
//     Drivetrain(){}

//     void drive(double velocity){
//       this->left_motor.spin(velocity);
//       this->right_motor.spin(velocity);
//     }

//     void turn(double velocity){
//       this->left_motor.spin(velocity);
//       this->right_motor.spin(-velocity);
//     }

//     void stop(){
//       this->left_motor.stop();
//       this->right_motor.stop();
//     }
// };