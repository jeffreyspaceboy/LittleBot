/*---KEYBOARD_CONTROL_HPP---*/
#ifndef KEYBOARD_CONTROL_HPP
#define KEYBOARD_CONTROL_HPP
/*----------------------------------------------------------------------------*/
/*    Module:       keyboard_control.hpp                                      */
/*    Author:       Jeffrey Fisher II                                         */
/*    Modified:     2022-02-22                                                */
/*----------------------------------------------------------------------------*/


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
//#include <rclcpp_action/rclcpp_action.hpp>
//#include <turtlesim/action/rotate_absolute.hpp>

class KeyboardController{
	public:
  		KeyboardController();
  		int keyLoop();
	private:
		void spin();
		//void sendGoal(float theta);
		//void goalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr> future);
		//void cancelGoal();
  
		rclcpp::Node::SharedPtr nh_;
		double linear_, angular_, l_scale_, a_scale_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
		//rclcpp_action::Client<turtlesim::action::RotateAbsolute>::SharedPtr rotate_absolute_client_;
		//rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr goal_handle_;
};
#endif
/*---KEYBOARD_CONTROL_HPP---*/