#include <memory>
#include <string>

#include "../include/Robot.hpp"

using std::placeholders::_1;

void Robot::command_callback(const std_msgs::msg::String::SharedPtr msg){
    std::string command = msg->data.c_str();
    if(command == "drive"){
        printf("Driving\n");
        this->base.drive(255);
    }else if(command == "stop"){
        printf("Stopping\n");
        this->base.stop();
    }
}

void Robot::cmd_vel_callback(const geometry_msgs::Twist::SharedPtr msg){
    printf("Velocity Commanded\n");
    this->base.drive(msg->linear.x);
    //this->base.drive(msg->angular.z);
}

Robot::Robot(){
    this->command_subscription = this->create_subscription<std_msgs::msg::String>("command", 10, std::bind(&Robot::command_callback, this, _1));
    this->cmd_vel_subscription = this->create_subscription<geometry_msgs::Twist>("cmd_vel", 2, std::bind(&Robot::cmd_vel_callback, this, _1));
}