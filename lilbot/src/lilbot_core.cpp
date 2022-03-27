
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#ifdef __arm__
#include <pigpio.h> //https://roboticsbackend.com/use-and-compile-wiringpi-with-ros-on-raspberry-pi/
#endif

#define WHEEL_BASE 0.185
#define WHEEL_RADIUS 0.038

namespace Lilbot{
	class PID_Controller{
		bool _enabled;
		float _target;
		float _kp, _ki, _kd;
		float _error, _prev_error, _error_tolerance, _error_integral, _dedt, _dt;
		long _prev_time;

		PID_Controller(float kP = 0.0, float kI = 0.0, float kD = 0.0) : 
			_enabled(false),
			_target(0.0),
			_kp(kP), _ki(kI), _kd(kD),
			_error(0.0), _prev_error(0.0), _error_tolerance(0.0), _error_integral(0.0), _dedt(0.0), _dt(0.0),
			_prev_time(0)
		{
			// Insert other setup
		}

		void start(float target, float error_tolerance = 0.0){
			_error_tolerance = error_tolerance;
			_target = target;
			_prev_error = target;
			_error_integral = 0.0;
			#ifdef __arm__
			_prev_time = gpioTick();
			#endif
			_enabled = true;
		}

		float control(float current){
			if(!_enabled){ 
				//RCLCPP_WARN_ONCE(this->get_logger(), "You must run pid_start before using the PID controller.");
			}

			long current_time = 0.0;
			#ifdef __arm__
			current_time = gpioTick();
			#endif
			_dt = (float)(current_time - _prev_time);	// Time Delta
			_error = _target - current;					// Error

			// If error is within the user defined tolerance range, act as if there is no error.
			if(_error < _error_tolerance  || _error > -_error_tolerance) { _error = 0.0; } 

			_error_integral += _error * _dt;			// Integral
			_dedt = (_error - _prev_error) / _dt;		// Derivative  

			_prev_error = _error;						// Update Previous Error
			_prev_time = current_time;					// Update Previous Time

			return (_kp * _error) + (_ki * _error_integral) + (_kd * _dedt); // Return the Control Signal
		}
		
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
					RCLCPP_WARN_ONCE(this->get_logger(), "GPIO's are disabled on this platform. Try this node on the Lilbot instead.");
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