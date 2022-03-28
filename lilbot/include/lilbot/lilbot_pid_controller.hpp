#ifndef LILBOT_PID_CONTROLLER_H
#define LILBOT_PID_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "lilbot_msgs/srv/pid.hpp"

namespace Lilbot{
	/** @brief PID service node for controlling motors and other robotic systems. (ROS2 - Galactic)*/
	class PID_Controller : public rclcpp::Node {
		public:  
			/** @brief Construct a new pid controller node. 
			 * (NOTE: Each system must use its own PID controller node. Do not use the same PID controller for multiple systems.)
			 * 
			 * @param node_name Name associated with the PID controller node.
			 * @param service_name Name associated with the PID service.
			 * @param kP Proportional gain constant.
			 * @param kI Integral gain constant.
			 * @param kD Derivative gain constant.
			 */
			PID_Controller(const std::string &node_name, const std::string &service_name, float kP = 0.0, float kI = 0.0, float kD = 0.0);

			/** @brief Updates the PID controller, and outputs an adjusted control signal. 
			 * 
			 * @param current Current sensor/calculated value.
			 * @param target Target sensor/calculated value.
			 * @param tolerance Error tolerance for the sensor/calculated value.
			 * @param current_time The current time.
			 * @return float: Result control signal.
			 */
			float control(float current, float target, float tolerance, float current_time);
			
		private:  
			/** @brief Target sensor/calculated value.*/
			float _target;

			/** @brief Proportional gain constant.*/
			float _kp;
			/** @brief Integral gain constant.*/
			float _ki; 
			/** @brief Derivative gain constant.*/
			float _kd;

			/** @brief Error between the current sensor/calculated value and the target.*/
			float _error;
			float _prev_error;
			float _error_tolerance;
			float _error_integral;
			float _dedt;

			float _prev_time; // [sec]
			rclcpp::Service<lilbot_msgs::srv::Pid>::SharedPtr _service;
	};
}
#endif