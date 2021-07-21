/*---ROBOT_HPP---*/
#ifndef ROBOT_HPP
#define ROBOT_HPP
/*----------------------------------------------------------------------------*/
/*    Module:       Robot.hpp                                                 */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2020-07-21                                                */
/*----------------------------------------------------------------------------*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/Twist.h>"

#include "Drivetrain.hpp"

class Robot : public rclcpp::Node{
  private:
    Drivetrain base;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription;
    rclcpp::Subscription<geometry_msgs::Twist>::SharedPtr cmd_vel_subscription;
    void command_callback(const std_msgs::msg::String::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::Twist::SharedPtr msg);
  public:
    Robot() : Node("little_bot");
};
#endif
/*---ROBOT_HPP---*/