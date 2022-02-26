/*---KEYBOARD_REMOTE_CONTROLLER_CPP---*/
/*----------------------------------------------------------------------------*/
/*    Module:       keyboard_remote_controller.c                              */
/*    Author:       Jeffrey Fisher II                                         */
/*    Modified:     2022-02-23                                                */
/*----------------------------------------------------------------------------*/

/* HEADER INCLUDE */
#include "keyboard_remote_controller.hpp"

#include "keyboard_reader.hpp"

KeyboardRemoteController::KeyboardRemoteController(): 
	linear_(0.0),
	angular_(0.0),
	l_scale_(2.0),
	a_scale_(2.0)
{
  	node_handler_ = rclcpp::Node::make_shared("teleop_turtle");
  	node_handler_->declare_parameter("scale_angular", rclcpp::ParameterValue(2.0));
  	node_handler_->declare_parameter("scale_linear", rclcpp::ParameterValue(2.0));
  	node_handler_->get_parameter("scale_angular", a_scale_);
  	node_handler_->get_parameter("scale_linear", l_scale_);

	twist_pub_ = node_handler_->create_publisher<geometry_msgs::msg::Twist>("little_robot/cmd_vel", 1);
  	//rotate_absolute_client_ = rclcpp_action::create_client<turtlesim::action::RotateAbsolute>(node_handler_, "little_robot/rotate_absolute");
}

// void KeyboardRemoteController::sendGoal(float theta){
//   	auto goal = turtlesim::action::RotateAbsolute::Goal();
//   	goal.theta = theta;
//   	auto send_goal_options = rclcpp_action::Client<turtlesim::action::RotateAbsolute>::SendGoalOptions();
//   	send_goal_options.goal_response_callback = [this](std::shared_future<rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr> future){
// 		RCLCPP_DEBUG(node_handler_->get_logger(), "Goal response received");
// 		this->goal_handle_ = future.get();
// 	};
//   	rotate_absolute_client_->async_send_goal(goal, send_goal_options);
// }

// void KeyboardRemoteController::goalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr> future){
//   	RCLCPP_DEBUG(node_handler_->get_logger(), "Goal response received");
//   	this->goal_handle_ = future.get();
// }

// void KeyboardRemoteController::cancelGoal(){
//  	if (goal_handle_){
//    		RCLCPP_DEBUG(node_handler_->get_logger(), "Sending cancel request");
//    		try{
//     		rotate_absolute_client_->async_cancel_goal(goal_handle_);
//    		}
//    		catch (...){
//      		// This can happen if the goal has already terminated and expired
// 		}
//  	}
// }

KeyboardReader input;

void quit(int sig){
  	(void)sig;
  	input.shutdown();
  	rclcpp::shutdown();
  	exit(0);
}


int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	KeyboardRemoteController teleop;
	signal(SIGINT,quit);
	int rc = teleop.keyLoop();
	input.shutdown();
	rclcpp::shutdown();
	return rc;
}

void KeyboardRemoteController::spin(){
  	while (rclcpp::ok()){ 
		rclcpp::spin_some(node_handler_); 
	}
}

int KeyboardRemoteController::keyLoop(){
	char c;
	bool dirty = false;
	std::thread{ std::bind(&KeyboardRemoteController::spin, this) }.detach();

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use arrow keys to move the turtle.");
	puts("'Q' to quit.");

	for(;;){
		// Get the next event from the keyboard  
		try{
			input.readOne(&c);
		}catch (const std::runtime_error &){
			perror("read():");
			return -1;
		}

		linear_ = angular_ = 0.0;
		RCLCPP_DEBUG(node_handler_->get_logger(), "value: 0x%02X\n", c);
	
		switch(c){
			case KEYCODE_LEFT:
				RCLCPP_DEBUG(node_handler_->get_logger(), "LEFT");
				angular_ = 1.0;
				dirty = true;
				break;
			case KEYCODE_RIGHT:
				RCLCPP_DEBUG(node_handler_->get_logger(), "RIGHT");
				angular_ = -1.0;
				dirty = true;
				break;
			case KEYCODE_UP:
				RCLCPP_DEBUG(node_handler_->get_logger(), "UP");
				linear_ = 1.0;
				dirty = true;
				break;
			case KEYCODE_DOWN:
				RCLCPP_DEBUG(node_handler_->get_logger(), "DOWN");
				linear_ = -1.0;
				dirty = true;
				break;
			case KEYCODE_Q:
				RCLCPP_DEBUG(node_handler_->get_logger(), "quit");
				return 0;
		}
	
		geometry_msgs::msg::Twist twist;
		twist.angular.z = a_scale_ * angular_;
		twist.linear.x = l_scale_ * linear_;
		if(dirty ==true)
		{
			twist_pub_->publish(twist);    
			dirty = false;
		}
	}
	return 0;
}

/*---KEYBOARD_REMOTE_CONTROLLER_CPP---*/