
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "lilbot_msgs/srv/pid.hpp"

#include "lilbot_driver/lilbot_pid_controller.hpp"
#include "lilbot_driver/lilbot_encoder.hpp"
#include "lilbot_driver/lilbot_motor.hpp"

#ifdef __aarch64__
#include <pigpio.h> //https://roboticsbackend.com/use-and-compile-wiringpi-with-ros-on-raspberry-pi/
#endif

#define WHEEL_BASE 0.185
#define HALF_WHEEL_BASE 0.0925
#define WHEEL_RADIUS 0.038

//#define ENCODER_RPM_BUFFER_SIZE 10
#define ENC_RATIO 1.0F/(44.0F * 21.3F)

namespace Lilbot{

	class Drivetrain : public rclcpp::Node{
		public:
			Drivetrain(const std::string &node_name) : 
				Node(node_name),
				_pidVelL("PID_VEL_LEFT", 1.0F, 1.0F, 1.0F),
				_pidVelR("PID_VEL_RGHT", 3.5F, 0.00001F, 0.0001F),
				_encL("ENC_LEFT", L_ENC_A, L_ENC_B, ENC_RATIO, true),
				_encR("ENC_RGHT", R_ENC_A, R_ENC_B, ENC_RATIO, false),
				_motorL("MOTOR_LEFT", L_MTR_EN, L_MTR_A, L_MTR_B, true, 255, &_encL, &_pidVelL),
				_motorR("MOTOR_RGHT", R_MTR_EN, R_MTR_A, R_MTR_B, false, 255, &_encR, &_pidVelR)
			{
				this->_odom_refresh_timer = this->create_wall_timer(
					this->_refresh_delay_usec, 
					std::bind(&Drivetrain::odom_timer_callback, this)
				);

				this->_cmd_vel_subscription = this->create_subscription<geometry_msgs::msg::Twist>(
					"cmd_vel",
					10,
					std::bind(&Drivetrain::command_velocity_callback, this, std::placeholders::_1)
				);
			}
		private:
			PID_Controller _pidVelL;
			PID_Controller _pidVelR;

			Encoder _encL;
			Encoder _encR;

			Motor _motorL;
			Motor _motorR;

			rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_publisher;

			rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_subscription;

			rclcpp::TimerBase::SharedPtr _odom_refresh_timer;
			std::chrono::microseconds _refresh_delay_usec = std::chrono::microseconds(200000); // [usec]

			void odom_timer_callback()
			{
				#ifndef __aarch64__
					RCLCPP_WARN_ONCE(this->get_logger(), "GPIO's are disabled on this platform. Try this node on the Lilbot instead.");
				#endif
				RCLCPP_INFO(this->get_logger(), "MTR_L: %f | MTR_R: %f",_motorL.get_rpm(), _motorR.get_rpm());
			}

			void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
			{
				//RCLCPP_INFO(this->get_logger(), "X: %f",msg->linear.x);
				double forward_speed = msg->linear.x;
				double angular_speed = msg->angular.z;
				float command_wheel_motor_left = (float)((forward_speed - angular_speed * HALF_WHEEL_BASE) / WHEEL_RADIUS);
				float command_wheel_motor_right = (float)((forward_speed + angular_speed * HALF_WHEEL_BASE) / WHEEL_RADIUS);
				//_motorL.spin(command_wheel_motor_left);
				//_motorR.spin(command_wheel_motor_right);
				_motorL.set_rpm(command_wheel_motor_left);
				_motorR.set_rpm(command_wheel_motor_right);
				RCLCPP_INFO(this->get_logger(), "MTR_CMD_L: %f | MTR_R: %f",command_wheel_motor_left, command_wheel_motor_right);
			}
	};
}

int main(int argc, char * argv[]){
	#ifdef __aarch64__
	if (gpioInitialise() < 0) return 1;
	#endif
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Lilbot::Drivetrain>("Lilbot"));
	rclcpp::shutdown();
	#ifdef __aarch64__
	gpioTerminate();
	#endif
	return 0;
}




// std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
//     while (1) {
//       std::this_thread::sleep_for(std::chrono::milliseconds(500));
//       std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

//       auto diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
//       if (diff_ms.count() > 5000) {
//         break;
//       }
//     }