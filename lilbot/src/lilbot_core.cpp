
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#ifdef __arm__
#include <pigpio.h> //https://roboticsbackend.com/use-and-compile-wiringpi-with-ros-on-raspberry-pi/
#endif

namespace Lilbot{
	class Motor : public rclcpp::Node{
		public:
			Motor(const std::string &node_name) : Node(node_name){
				RCLCPP_INFO(this->get_logger(), "MOTOR INIT: %s", node_name);
			}
		private:
			uint8_t gpio_pin_enable, gpio_pin_phase_a, gpio_pin_phase_b;
	};

	class Drivetrain : public rclcpp::Node{
		public:
			Drivetrain(const std::string &node_name) : Node(node_name){
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
			rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_publisher;

			rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_subscription;

			rclcpp::TimerBase::SharedPtr _odom_refresh_timer;
			std::chrono::microseconds _refresh_delay_usec = std::chrono::microseconds(1500); // [usec]

			void odom_timer_callback()
			{
				#ifndef __arm__
					RCLCPP_WARN_ONCE(this->get_logger(), "GPIO's are disabled on this platform. Try using this node on a Raspberry Pi.");
				#elif
					// Encoder stuff
				#endif
			}

			void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
			{
				geometry_msgs::msg::Twist *cmd_vel_msg = msg.get();
				RCLCPP_INFO(this->get_logger(), "X: %f",cmd_vel_msg->linear.x);
			}
	};
}

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Lilbot::Drivetrain>("Lilbot"));
	rclcpp::shutdown();
	return 0;
}