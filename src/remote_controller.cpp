/*---REMOTE_CONTROLLER_CPP---*/
/*----------------------------------------------------------------------------*/
/*    Module:       remote_controller.cpp                                     */
/*    Author:       Jeffrey Fisher II                                         */
/*    Modified:     2022-01-30                                                */
/*----------------------------------------------------------------------------*/

/* HEADER INCLUDE */
#include "little_bot/remote_controller.hpp"

/*----- CLASS Keyboard_Reader -----*/
/* PUBLIC */
Keyboard_Reader::Keyboard_Reader(): kfd(0){
	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	struct termios raw;
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);
}

void Keyboard_Reader::readOne(char * c){
	int rc = read(kfd, c, 1);
	if(rc < 0){ throw std::runtime_error("read failed"); }
}
		
void Keyboard_Reader::shutdown(){
	tcsetattr(kfd, TCSANOW, &cooked);
}


/*----- CLASS Remote_Controller -----*/
/* PUBLIC */
Remote_Controller::Remote_Controller(): linear_(0), angular_(0), l_scale_(2.0), a_scale_(2.0){
	node_ = rclcpp::Node::make_shared("remote_controller");
	node_->declare_parameter("scale_angular", rclcpp::ParameterValue(2.0));
	node_->declare_parameter("scale_linear", rclcpp::ParameterValue(2.0));
	node_->get_parameter("scale_angular", a_scale_);
	node_->get_parameter("scale_linear", l_scale_);

	twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("command_velocity", 1);
	rotate_absolute_client_ = rclcpp_action::create_client<little_bot::action::RotateAbsolute>(node_, "rotate_absolute");
}

int Remote_Controller::keyLoop(){
	char c;
	bool dirty=false;
	std::thread{std::bind(&Remote_Controller::spin, this)}.detach();
	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use arrow keys to move the little_bot.");
	puts("Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.");
	puts("'Q' to quit.");

	for(;;){
		// get the next event from the keyboard  
		try{
			input.readOne(&c);
		}catch(const std::runtime_error &){
			perror("read():");
			return -1;
		}

		linear_=angular_=0;
		RCLCPP_DEBUG(node_->get_logger(), "value: 0x%02X\n", c);
  
		switch(c){
			case KEYCODE_LEFT:
				RCLCPP_DEBUG(node_->get_logger(), "LEFT");
				angular_ = 1.0;
				dirty = true;
				break;
			case KEYCODE_RIGHT:
				RCLCPP_DEBUG(node_->get_logger(), "RIGHT");
				angular_ = -1.0;
				dirty = true;
				break;
			case KEYCODE_UP:
				RCLCPP_DEBUG(node_->get_logger(), "UP");
				linear_ = 1.0;
				dirty = true;
				break;
			case KEYCODE_DOWN:
				RCLCPP_DEBUG(node_->get_logger(), "DOWN");
				linear_ = -1.0;
				dirty = true;
				break;
			case KEYCODE_G:
				RCLCPP_DEBUG(node_->get_logger(), "G");
				sendGoal(0.0f);
				break;
			case KEYCODE_T:
				RCLCPP_DEBUG(node_->get_logger(), "T");
				sendGoal(0.7854f);
				break;
			case KEYCODE_R:
				RCLCPP_DEBUG(node_->get_logger(), "R");
				sendGoal(1.5708f);
				break;
			case KEYCODE_E:
				RCLCPP_DEBUG(node_->get_logger(), "E");
				sendGoal(2.3562f);
				break;
			case KEYCODE_D:
				RCLCPP_DEBUG(node_->get_logger(), "D");
				sendGoal(3.1416f);
				break;
			case KEYCODE_C:
				RCLCPP_DEBUG(node_->get_logger(), "C");
				sendGoal(-2.3562f);
				break;
			case KEYCODE_V:
				RCLCPP_DEBUG(node_->get_logger(), "V");
				sendGoal(-1.5708f);
				break;
			case KEYCODE_B:
				RCLCPP_DEBUG(node_->get_logger(), "B");
				sendGoal(-0.7854f);
				break;
			case KEYCODE_F:
				RCLCPP_DEBUG(node_->get_logger(), "F");
				cancelGoal();
				break;
			case KEYCODE_Q:
				RCLCPP_DEBUG(node_->get_logger(), "quit");
				return 0;
		}

    	geometry_msgs::msg::Twist twist;
    	twist.angular.z = a_scale_*angular_;
    	twist.linear.x = l_scale_*linear_;
    	if(dirty ==true){
      		twist_pub_->publish(twist);
      		dirty=false;
    	}
  	}
	return 0;
}

/* PRIVATE */
void Remote_Controller::spin(){
	while (rclcpp::ok()){
		rclcpp::spin_some(node_);
	}
}

void Remote_Controller::sendGoal(float theta){
	auto goal = little_bot::action::RotateAbsolute::Goal();
	goal.theta = theta;
	auto send_goal_options = rclcpp_action::Client<little_bot::action::RotateAbsolute>::SendGoalOptions();
	send_goal_options.goal_response_callback = [this](std::shared_future<rclcpp_action::ClientGoalHandle<little_bot::action::RotateAbsolute>::SharedPtr> future){
		RCLCPP_DEBUG(node_->get_logger(), "Goal response received");
		this->goal_handle_ = future.get();
	};
	rotate_absolute_client_->async_send_goal(goal, send_goal_options);
}

void Remote_Controller::goalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<little_bot::action::RotateAbsolute>::SharedPtr> future){
	RCLCPP_DEBUG(node_->get_logger(), "Goal response received");
	this->goal_handle_ = future.get();
}

void Remote_Controller::cancelGoal(){
	if(goal_handle_){
		RCLCPP_DEBUG(node_->get_logger(), "Sending cancel request");
		try{
			rotate_absolute_client_->async_cancel_goal(goal_handle_);
		}catch(...){
			// This can happen if the goal has already terminated and expired
		}
	}
}


/*----- MAIN -----*/
void quit(int sig){
	(void)sig;
	input.shutdown();
	rclcpp::shutdown();
	exit(0);
}

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	Remote_Controller remote_controller;
	signal(SIGINT,quit);
	int rc = remote_controller.keyLoop();
	input.shutdown();
	rclcpp::shutdown();
	return rc;
}
/*---REMOTE_CONTROLLER_CPP---*/