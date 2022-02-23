/*---KEYBOARD_CONTROL_CPP---*/
/*----------------------------------------------------------------------------*/
/*    Module:       keyboard_control.cpp                                      */
/*    Author:       Jeffrey Fisher II                                         */
/*    Modified:     2022-02-22                                                */
/*----------------------------------------------------------------------------*/

/* HEADER INCLUDE */
#include "little_bot/remote_control/keyboard_control.hpp"

#include <signal.h>
#include <stdio.h>
#ifndef _WIN32
	#include <termios.h>
	#include <unistd.h>
#else
	#include <windows.h>
#endif

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76

/*----- CLASS Keyboard_Reader -----*/
class KeyboardReader
{
	public:
  		KeyboardReader()
		#ifndef _WIN32
			: kfd(0)
		#endif
  		{
			#ifndef _WIN32
				// get the console in raw mode
				tcgetattr(kfd, &cooked);
				struct termios raw;
				memcpy(&raw, &cooked, sizeof(struct termios));
				raw.c_lflag &=~ (ICANON | ECHO);
				// Setting a new line, then end of file
				raw.c_cc[VEOL] = 1;
				raw.c_cc[VEOF] = 2;
				tcsetattr(kfd, TCSANOW, &raw);
			#endif
  		}
  		void readOne(char * c)
  		{
			#ifndef _WIN32
				int rc = read(kfd, c, 1);
				if (rc < 0) { throw std::runtime_error("read failed"); }
			#else
				for(;;){
					HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
					INPUT_RECORD buffer;
					DWORD events;
					PeekConsoleInput(handle, &buffer, 1, &events);
					if(events > 0){
						ReadConsoleInput(handle, &buffer, 1, &events);
						if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT){
							*c = KEYCODE_LEFT;
							return;
						}else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP){
							*c = KEYCODE_UP;
							return;
						}else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT){
							*c = KEYCODE_RIGHT;
							return;
						}else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN){
							*c = KEYCODE_DOWN;
							return;
						}else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x42){
							*c = KEYCODE_B;
							return;
						}else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x43){
							*c = KEYCODE_C;
							return;
						}else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x44){
							*c = KEYCODE_D;
							return;
						}else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x45){
							*c = KEYCODE_E;
							return;
						}else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x46){
							*c = KEYCODE_F;
							return;
						}else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x47){
							*c = KEYCODE_G;
							return;
						}else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x51){
							*c = KEYCODE_Q;
							return;
						}else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x52){
							*c = KEYCODE_R;
							return;
						}else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x54){
							*c = KEYCODE_T;
							return;
						}else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x56){
							*c = KEYCODE_V;
							return;
						}
					}
				}
			#endif
  		}
  		void shutdown()
  		{
			#ifndef _WIN32
    			tcsetattr(kfd, TCSANOW, &cooked);
			#endif
  		}
	private:
		#ifndef _WIN32
			int kfd;
			struct termios cooked;
		#endif
};




KeyboardController::KeyboardController(): 
	linear_(0.0),
	angular_(0.0),
	l_scale_(2.0),
	a_scale_(2.0)
{
  	nh_ = rclcpp::Node::make_shared("teleop_turtle");
  	nh_->declare_parameter("scale_angular", rclcpp::ParameterValue(2.0));
  	nh_->declare_parameter("scale_linear", rclcpp::ParameterValue(2.0));
  	nh_->get_parameter("scale_angular", a_scale_);
  	nh_->get_parameter("scale_linear", l_scale_);

	twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("little_robot/cmd_vel", 1);
  	//rotate_absolute_client_ = rclcpp_action::create_client<turtlesim::action::RotateAbsolute>(nh_, "little_robot/rotate_absolute");
}

// void KeyboardController::sendGoal(float theta){
//   	auto goal = turtlesim::action::RotateAbsolute::Goal();
//   	goal.theta = theta;
//   	auto send_goal_options = rclcpp_action::Client<turtlesim::action::RotateAbsolute>::SendGoalOptions();
//   	send_goal_options.goal_response_callback = [this](std::shared_future<rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr> future){
// 		RCLCPP_DEBUG(nh_->get_logger(), "Goal response received");
// 		this->goal_handle_ = future.get();
// 	};
//   	rotate_absolute_client_->async_send_goal(goal, send_goal_options);
// }

// void KeyboardController::goalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr> future){
//   	RCLCPP_DEBUG(nh_->get_logger(), "Goal response received");
//   	this->goal_handle_ = future.get();
// }

// void KeyboardController::cancelGoal(){
//  	if (goal_handle_){
//    		RCLCPP_DEBUG(nh_->get_logger(), "Sending cancel request");
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
	KeyboardController teleop;
	signal(SIGINT,quit);
	int rc = teleop.keyLoop();
	input.shutdown();
	rclcpp::shutdown();
	return rc;
}

void KeyboardController::spin(){
  	while (rclcpp::ok()){ 
		rclcpp::spin_some(nh_); 
	}
}

int KeyboardController::keyLoop(){
	char c;
	bool dirty = false;
	std::thread{ std::bind(&KeyboardController::spin, this) }.detach();

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
		RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);
	
		switch(c){
			case KEYCODE_LEFT:
				RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
				angular_ = 1.0;
				dirty = true;
				break;
			case KEYCODE_RIGHT:
				RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
				angular_ = -1.0;
				dirty = true;
				break;
			case KEYCODE_UP:
				RCLCPP_DEBUG(nh_->get_logger(), "UP");
				linear_ = 1.0;
				dirty = true;
				break;
			case KEYCODE_DOWN:
				RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
				linear_ = -1.0;
				dirty = true;
				break;
			case KEYCODE_Q:
				RCLCPP_DEBUG(nh_->get_logger(), "quit");
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

/*---KEYBOARD_CONTROL_CPP---*/