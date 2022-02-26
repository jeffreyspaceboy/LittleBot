/*---KEYBOARD_REMOTE_CONTROLLER_HPP---*/
#ifndef KEYBOARD_REMOTE_CONTROLLER_HPP
#define KEYBOARD_REMOTE_CONTROLLER_HPP 
/*----------------------------------------------------------------------------*/
/*    Module:       keyboard_remote_controller.hpp                            */
/*    Author:       Jeffrey Fisher II                                         */
/*    Modified:     2022-02-23                                                */
/*----------------------------------------------------------------------------*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
//#include <rclcpp_action/rclcpp_action.hpp>
//#include <turtlesim/action/rotate_absolute.hpp>

class KeyboardRemoteController{
	public:
  		KeyboardRemoteController();
  		int keyLoop();
	private:
		void spin();
		//void sendGoal(float theta);
		//void goalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr> future);
		//void cancelGoal();
  
		rclcpp::Node::SharedPtr node_handler_;
		double linear_, angular_, l_scale_, a_scale_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
		//rclcpp_action::Client<turtlesim::action::RotateAbsolute>::SharedPtr rotate_absolute_client_;
		//rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr goal_handle_;
};
#endif
/*---KEYBOARD_REMOTE_CONTROLLER_HPP---*/