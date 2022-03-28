#ifndef LILBOT_ENCODER_HPP
#define LILBOT_ENCODER_HPP

#define ENCODER_RPM_BUFFER_SIZE 10
#define ENCODER_REFRESH_USEC 100

#ifndef TWO_PI
#define TWO_PI 6.28318530718
#endif

#include <stdint.h>
#include "rclcpp/rclcpp.hpp"

/*
	Encoder Tick Counting Logic:
	There are four possible states that can occur on any given level
	shift of the encoder Phases. 
	NOTE: Dependant on knowing the shift in level, not just H/L state.
	This allows for every level shift to count as a tick.
	CLOCK WISE:
	A&!B B&A !A&B !B&A A&!B B&A !A&B !B&!A
		_________          ________
		|    ____|___     |    ____|___
	A___|   |    |___|____|   |    |___|__
	B_______|        |________|        |__
	COUNTER CLOCK WISE:
	B&!A A&B !B&A !A&!B B&!A A&B !B&A !A&!B
			________          ________
		___|____    |     ___|____    |
	A__|___|    |   |____|___|    |   |__
	B__|        |________|        |______
*/

namespace Lilbot{
	class Encoder : public rclcpp::Node {
		public:
			Encoder(const std::string &node_name, uint8_t gpio_pin_phase_a, uint8_t gpio_pin_phase_b, float encoder_ratio, int reverse = 0);

			void reset_count();

			float get_rotations();
			float get_angle_degrees();
			float get_angle_radians();
			float get_rpm();

		private:
			uint8_t _gpio_pin_phase_a, _gpio_pin_phase_b, _gpio_pin_prev; 
			int _level_phase_a, _level_phase_b;
			long _count, _count_prev;
			float _rpm, _ratio;
			float _rpm_prev[ENCODER_RPM_BUFFER_SIZE];
			float _time_prev;

			rclcpp::TimerBase::SharedPtr _timer;
			
			void update_rpm();
			void encoder_tick_event_callback(int gpio, int level, uint32_t current_us, void *data);
	};
}
#endif