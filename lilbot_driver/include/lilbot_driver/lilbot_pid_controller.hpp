#ifndef LILBOT_PID_CONTROLLER_HPP
#define LILBOT_PID_CONTROLLER_HPP

#include "lilbot_driver/lilbot_definitions.h"

namespace Lilbot
{
	/** @brief PID controller for motors and other robotic systems. (Any ROS Version)*/
	class PID_Controller
	{
		public:  
			/** @brief Construct a new pid controller.
			 * (NOTE: Each system should use an unique PID controller. Do not use the same PID controller for multiple systems.)
			 * 
			 * @param name Name of the PID Controller to help with debug.
			 * @param kP Proportional gain constant.
			 * @param kI Integral gain constant.
			 * @param kD Derivative gain constant.
			 */
			PID_Controller(char name[NAME_MAX_SIZE] = "PID_Controller", float kP = 0.0, float kI = 0.0, float kD = 0.0);

			/** @brief Updates the PID controller, and outputs an adjusted control signal. 
			 * 
			 * @param current Current sensor/calculated value.
			 * @param target Target sensor/calculated value.
			 * @param tolerance Error tolerance for the sensor/calculated value.
			 * @param current_time The current time.
			 * @return float: Result control signal.
			 */
			float control(float current, float target, float tolerance, float current_time_sec, float timeout_sec = 1.0);
			
		private:  
			char _name[NAME_MAX_SIZE];
			float _target;
			float _kp, _ki, _kd;
			float _error, _error_prev, _error_tolerance;
			float _error_integral, _dedt;
			float _time_prev; // [sec]
	};
}
#endif