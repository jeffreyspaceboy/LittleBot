#ifndef LILBOT_MOTOR_HPP
#define LILBOT_MOTOR_HPP

/*---C---*/
#include <stdint.h>

/*---CPP---*/
#include <string>
#include <mutex>
#include <thread>

/*---LILBOT---*/
#include "lilbot_driver/lilbot_encoder.hpp"
#include "lilbot_driver/lilbot_pid_controller.hpp"

namespace Lilbot{
	/** @brief For individual motor control. (Any ROS Distribution)*/
	class Motor{
		public:
			Motor(const std::string &motor_name, uint8_t gpio_enable_pin, uint8_t gpio_pin_phase_a, uint8_t gpio_pin_phase_b, int reverse, int max_power, Encoder *new_encoder, PID_Controller *new_pid_velocity_controller, unsigned int rpm_control_refresh_us = MOTOR_DEFAULT_REFRESH_RATE);
			~Motor();

			/* SET FUNCTIONS */
			void set_rpm(float rpm_target);

			/* GET FUNCTIONS */
			float get_rotations();
			float get_angle_degrees();
			float get_angle_radians();
			float get_rpm();
			int get_power();

			/* MOTION FUNCTIONS */
			void spin(int new_power);
			void stop();

		private:
			bool _rpm_control_enabled;	
			std::string _name;
			uint8_t _gpio_pin_enable, _gpio_pin_phase_a, _gpio_pin_phase_b; 
			float _rpm_target, _prev_target_rpm;
			int _power, _power_max;
			unsigned int _rpm_control_refresh_us;

			Encoder *_encoder;
			PID_Controller *_pid_velocity_controller;

			std::mutex _mutex;
			std::thread _thread;	
			
			void _rpm_control_thread();
	};
}
#endif