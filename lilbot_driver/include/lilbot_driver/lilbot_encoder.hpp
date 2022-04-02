#ifndef LILBOT_ENCODER_HPP
#define LILBOT_ENCODER_HPP


/*---C---*/
#include <stdint.h>

/*---CPP---*/
#include <mutex>
#include <thread>
#include <string>

/*---LILBOT---*/
#include <lilbot_driver/lilbot_definitions.h>

namespace Lilbot
{
	/** @brief Encoder node for controlling sensing the rotation of motors. (Any ROS Distribution)*/
	class Encoder
	{
		public:
			/** @brief Construct a new Encoder object.
			 * 
			 * @param encoder_name Name of the Encoder to help with debug.
			 * @param gpio_pin_phase_a GPIO pin of phase a of the encoder.
			 * @param gpio_pin_phase_b GPIO pin of phase be of the encoder.
			 * @param encoder_ratio Ratio between the output of what you're trying to track (wheels), and the raw encoder. This should combine all gear ratios into one single value.
			 * @param reverse Enable to swap phase a and b. Use this to ensure forward on the encoder is the same as it is on your motor.
			 * @param rpm_refresh_us Amount of microseconds between each rpm refresh. RPM is calculated ever rpm_refresh_us amount of time.
			 */
			Encoder(const std::string &encoder_name, uint8_t gpio_pin_phase_a, uint8_t gpio_pin_phase_b, float encoder_ratio, bool reverse = false, unsigned int rpm_refresh_us = ENCODER_DEFAULT_REFRESH_RATE);
			
			/** @brief Destroy the Encoder object. */
			~Encoder();

			/** @brief Resets count and prev_count to 0. */
			void reset_count();

			float get_rotations();
			float get_angle_degrees();
			float get_angle_radians();
			float get_rpm();

		private:
			bool _enabled;
			std::string _name;
			uint8_t _gpio_pin_phase_a, _gpio_pin_phase_b, _gpio_pin_prev; 
			int _level_phase_a, _level_phase_b;
			long _count, _count_prev;
			float _rpm, _ratio;
			float _rpm_prev[ENCODER_RPM_BUFFER_SIZE];
			unsigned int _rpm_refresh_us; // [usec]
			float _time_prev; // [sec]

			std::mutex _mutex;
			std::thread _thread;		

			void _rpm_thread();
			void _tick_compute(int gpio, int level, uint32_t current_us);
			static void _tick_event_execute(int gpio, int level, uint32_t current_us, void *data);
	};
}
#endif

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