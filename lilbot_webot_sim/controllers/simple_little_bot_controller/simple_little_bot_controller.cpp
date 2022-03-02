// File:          simple_little_bot_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Lidar.hpp>
#include <webots/PositionSensor.hpp>

#define MAX_SPEED 10.0

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  
  Motor *left_wheel_motor = robot->getMotor("left_wheel_motor");
  Motor *right_wheel_motor = robot->getMotor("right_wheel_motor");
  
  Lidar *lidar = robot->getLidar("lidar");
  lidar->enable(timeStep);
  lidar->enablePointCloud();
  
  PositionSensor *left_wheel_encoder = robot->getPositionSensor("left_wheel_encoder");
  PositionSensor *right_wheel_encoder = robot->getPositionSensor("right_wheel_encoder");
  left_wheel_encoder->enable(timeStep);
  right_wheel_encoder->enable(timeStep);
  
  
  left_wheel_motor->setPosition(INFINITY);
  right_wheel_motor->setPosition(INFINITY);
  

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    
    left_wheel_motor->setVelocity(10.0);
    right_wheel_motor->setVelocity(10.0);
    //std::cout << lidar->getNumberOfPoints() << std::endl;
    
    std::cout << "("<< left_wheel_encoder->getValue() << ", " << right_wheel_encoder->getValue() << ")" << std::endl;
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
