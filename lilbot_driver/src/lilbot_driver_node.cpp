
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "lilbot_msgs/srv/pid.hpp"

#include "lilbot_driver/lilbot_pid_controller.hpp"
#include "lilbot_driver/lilbot_encoder.hpp"

#ifdef __arm__
#include <pigpio.h> //https://roboticsbackend.com/use-and-compile-wiringpi-with-ros-on-raspberry-pi/
#endif

#define WHEEL_BASE 0.185
#define WHEEL_RADIUS 0.038

//#define ENCODER_RPM_BUFFER_SIZE 10

namespace Lilbot{

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
					RCLCPP_WARN_ONCE(this->get_logger(), "GPIO's are disabled on this platform. Try this node on the Lilbot instead.");
				#elif
					// Encoder stuff
				#endif
			}

			void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
			{
				RCLCPP_INFO(this->get_logger(), "X: %f",msg->linear.x);
			}
	};
}

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Lilbot::Drivetrain>("Lilbot"));
	rclcpp::shutdown();
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