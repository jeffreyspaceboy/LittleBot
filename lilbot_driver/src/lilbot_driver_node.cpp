
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "lilbot_msgs/srv/pid.hpp"

#include "lilbot_driver/lilbot_pid_controller.hpp"

#ifdef __arm__
#include <pigpio.h> //https://roboticsbackend.com/use-and-compile-wiringpi-with-ros-on-raspberry-pi/
#endif

#define WHEEL_BASE 0.185
#define WHEEL_RADIUS 0.038

#define ENCODER_RPM_BUFFER_SIZE 10

namespace Lilbot{
	
	class Encoder{
		private:
			bool _enabled;
			uint8_t _gpio_phase_a, _gpio_phase_b, _prev_gpio; 
			int _level_phase_a, _level_phase_b;
			long _count, _prev_count;
			float _rpm, _ratio;
			float _prev_rpm[ENCODER_RPM_BUFFER_SIZE];
			uint32_t _prev_us;
		public:
		Encoder(uint8_t gpio_phase_a_pin, uint8_t gpio_phase_b_pin,  float encoder_ratio, int reverse = 0) :
			_enabled(true),
			_gpio_phase_a((reverse == 0) ? gpio_phase_a_pin : gpio_phase_b_pin),
			_gpio_phase_b((reverse == 0) ? gpio_phase_b_pin : gpio_phase_a_pin),
			_prev_gpio(-1), _level_phase_a(2), _level_phase_b(2),
			_count(0.0), _prev_count(0.0),
			_rpm(0.0), _ratio(encoder_ratio),
			_prev_us(0)
		{
			for(int i = 0; i < ENCODER_RPM_BUFFER_SIZE; i++){ _prev_rpm[i] = 0.0; }

			#ifdef __arm__
			gpioSetMode(new_encoder.gpio_phase_a, PI_INPUT);
			gpioSetMode(new_encoder.gpio_phase_b, PI_INPUT);
			gpioSetPullUpDown(new_encoder.gpio_phase_a, PI_PUD_UP);
			gpioSetPullUpDown(new_encoder.gpio_phase_b, PI_PUD_UP);
			#endif
		}

		void start(){
			#ifdef __arm__
			gpioSetISRFuncEx(_gpio_phase_a, EITHER_EDGE, ENCODER_EVENT_TIMEOUT, encoder_tick_event_callback, (void *)this);
			gpioSetISRFuncEx(_gpio_phase_b, EITHER_EDGE, ENCODER_EVENT_TIMEOUT, encoder_tick_event_callback, (void *)this);
			#endif
		}

		void reset(){
			_count = 0;
			_prev_count = 0;
		}

		float sense_rotations(){
			return ((float)_count) * _ratio;
		}

		float sense_angle_degrees(){
			return sense_rotations() * 360.0;
		}

		float sense_angle_radians(){
			return sense_rotations() * 2.0 * M_PI;
		}

		float sense_rpm(){
			return _rpm;
		}
	
	private:
		float refresh_rpm(){
			uint32_t current_us = 0;
			#ifdef __arm__
			current_us = gpioTick();
			#endif
			_rpm = ((float)(_count - _prev_count) / (float)(current_us - _prev_us)) * 60000000.0F * _ratio;
			_prev_count = _count;
			_prev_us = current_us;
			float sum = 0;
			for(int i = ENCODER_RPM_BUFFER_SIZE-1; i >= 1; i--){
				_prev_rpm[i] = _prev_rpm[i-1];
				sum += _prev_rpm[i];
			}
			_prev_rpm[0] = _rpm;
			sum += _prev_rpm[0];
			float rpm = _rpm = sum/ENCODER_RPM_BUFFER_SIZE;
			return rpm;
		}

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
		void tick_event_callback(int gpio, int level, uint32_t current_us, void *data){
			Encoder *encoder = (Encoder *) data;
			if(gpio == encoder->_gpio_phase_a){
				encoder->_level_phase_a = level;
				if(encoder->_level_phase_b == 0){
					encoder->_count += (encoder->_level_phase_a == 0) ? -1 : 1; 
				}else{
					encoder->_count += (encoder->_level_phase_a == 0) ? 1 : -1;
				}
			}else if(gpio == encoder->_gpio_phase_b){
				encoder->_level_phase_b = level;
				if(encoder->_level_phase_a == 0){
					encoder->_count += (encoder->_level_phase_b == 0) ? 1 : -1;
				}else{
					encoder->_count += (encoder->_level_phase_b == 0) ? -1 : 1;
				}
			} 
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