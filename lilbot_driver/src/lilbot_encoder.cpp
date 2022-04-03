/*---LILBOT_ENCODER_CPP---*/


/*---C---*/
#include <unistd.h>
#include <climits>
#ifdef __aarch64__
#include <pigpio.h>
#endif

/*---CPP---*/
#include <chrono>
#include <ctime>

/*---LILBOT---*/
#include "lilbot_driver/lilbot_encoder.hpp"

Lilbot::Encoder::Encoder(const std::string &encoder_name, uint8_t gpio_pin_phase_a, uint8_t gpio_pin_phase_b, float encoder_ratio, bool reverse, unsigned int rpm_refresh_us) :
	_enabled(true),
	_name(encoder_name),
	_gpio_pin_phase_a((reverse) ? gpio_pin_phase_a : gpio_pin_phase_b),
	_gpio_pin_phase_b((reverse) ? gpio_pin_phase_b : gpio_pin_phase_a),
	_gpio_pin_prev(-1), _level_phase_a(0), _level_phase_b(0),
	_count(0L), _count_prev(0L),
	_rpm(0.0), _ratio(encoder_ratio),
	_rpm_refresh_us(rpm_refresh_us)
{
	for(int i = 0; i < ENCODER_RPM_BUFFER_SIZE; i++){ _rpm_prev[i] = 0.0; }

	#ifdef __aarch64__
	gpioSetMode(_gpio_pin_phase_a, PI_INPUT);
	gpioSetMode(_gpio_pin_phase_b, PI_INPUT);
	gpioSetPullUpDown(_gpio_pin_phase_a, PI_PUD_UP);
	gpioSetPullUpDown(_gpio_pin_phase_b, PI_PUD_UP);

	gpioSetISRFuncEx(_gpio_pin_phase_a, EITHER_EDGE, ENCODER_EVENT_TIMEOUT, _tick_event_execute, this);
	gpioSetISRFuncEx(_gpio_pin_phase_b, EITHER_EDGE, ENCODER_EVENT_TIMEOUT, _tick_event_execute, this);
	#endif
	
	_thread = std::thread(&Lilbot::Encoder::_rpm_thread, this);
}

Lilbot::Encoder::~Encoder()
{
	_mutex.lock();
	_enabled = false;
	#ifdef __aarch64__
	gpioSetISRFuncEx(_gpio_pin_phase_a, EITHER_EDGE, ENCODER_EVENT_TIMEOUT, 0, this);
	gpioSetISRFuncEx(_gpio_pin_phase_b, EITHER_EDGE, ENCODER_EVENT_TIMEOUT, 0, this);
	#endif
	_thread.join();
	_mutex.unlock();
}

void Lilbot::Encoder::reset_count(){
	_mutex.lock();
	_count = 0L;
	_count_prev = 0L;
	_mutex.unlock();
}

float Lilbot::Encoder::get_rotations(){ 
	_mutex.lock();
	float rotations = (float)_count * _ratio;
	_mutex.unlock();
	return rotations; // [rotations]
} 

float Lilbot::Encoder::get_angle_degrees(){ return get_rotations() * 360.0; } // [degrees]

float Lilbot::Encoder::get_angle_radians(){ return get_rotations() * TWO_PI; } // [radians]

float Lilbot::Encoder::get_rpm(){ 
	_mutex.lock();
	float rpm = _rpm;
	_mutex.unlock();
	return rpm; // [rotations/minute]
}

void Lilbot::Encoder::_rpm_thread(){
	float time_current = 0.0;
	unsigned int microseconds;
	while(_enabled){
		#ifdef __aarch64__
		time_current = ((float)gpioTick()) / 1E6; // [seconds]
		#endif
		_mutex.lock();
		if(_count >= LONG_MAX - 1000L || _count <= LONG_MIN + 1000L)
		{
			// Handle overflow case
			_count -= _count_prev;
			_count_prev = 0L;
		}
		_rpm = ((float)(_count - _count_prev) / (time_current - _time_prev)) * 60.0F * _ratio;
		_count_prev = _count;
		_time_prev = time_current;
		float sum = _rpm;
		for(int i = ENCODER_RPM_BUFFER_SIZE-1; i >= 1; i--){
			_rpm_prev[i] = _rpm_prev[i-1];
			sum += _rpm_prev[i];
		}
		_rpm_prev[0] = _rpm;
		_rpm = sum/ENCODER_RPM_BUFFER_SIZE;
		microseconds = _rpm_refresh_us;
		_mutex.unlock();
		usleep(microseconds);
	}
}

void Lilbot::Encoder::_tick_compute(int gpio, int level){
	_mutex.lock();
	if(gpio == this->_gpio_pin_phase_a){
		this->_level_phase_a = level;
		if(this->_level_phase_b == 0){
			this->_count += (this->_level_phase_a == 0) ? -1 : 1; 
		}else{
			this->_count += (this->_level_phase_a == 0) ? 1 : -1;
		}
	}else if(gpio == this->_gpio_pin_phase_b){
		this->_level_phase_b = level;
		if(this->_level_phase_a == 0){
			this->_count += (this->_level_phase_b == 0) ? 1 : -1;
		}else{
			this->_count += (this->_level_phase_b == 0) ? -1 : 1;
		}
	} 
	_mutex.unlock();
}

void Lilbot::Encoder::_tick_event_execute(int gpio, int level, uint32_t current_us, void *data){
	Lilbot::Encoder *encoder = (Lilbot::Encoder *) data;
	encoder->_tick_compute(gpio, level);
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