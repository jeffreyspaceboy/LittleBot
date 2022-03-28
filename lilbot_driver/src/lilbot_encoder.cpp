#include "lilbot_driver/lilbot_encoder.hpp"

Lilbot::Encoder::Encoder(const std::string &node_name, uint8_t gpio_pin_phase_a, uint8_t gpio_pin_phase_b, float encoder_ratio, int reverse) :
	Node(node_name),
	_gpio_pin_phase_a((reverse == 0) ? gpio_pin_phase_a : gpio_pin_phase_b),
	_gpio_pin_phase_b((reverse == 0) ? gpio_pin_phase_b : gpio_pin_phase_a),
	_gpio_pin_prev(-1), _level_phase_a(2), _level_phase_b(2),
	_count(0L), _count_prev(0L),
	_rpm(0.0), _ratio(encoder_ratio),
	_time_prev((float)rclcpp::Time(0).seconds())
{
	for(int i = 0; i < ENCODER_RPM_BUFFER_SIZE; i++){ _rpm_prev[i] = 0.0; }

	#ifdef __arm__
	gpioSetMode(_gpio_pin_phase_a, PI_INPUT);
	gpioSetMode(_gpio_pin_phase_b, PI_INPUT);
	gpioSetPullUpDown(_gpio_pin_phase_a, PI_PUD_UP);
	gpioSetPullUpDown(_gpio_pin_phase_b, PI_PUD_UP);

	gpioSetISRFuncEx(_gpio_pin_phase_a, EITHER_EDGE, ENCODER_EVENT_TIMEOUT, this->tick_event_callback, (void *)this);
	gpioSetISRFuncEx(_gpio_pin_phase_b, EITHER_EDGE, ENCODER_EVENT_TIMEOUT, this->tick_event_callback, (void *)this);
	#endif

	_timer = this->create_wall_timer(std::chrono::microseconds(ENCODER_REFRESH_USEC), std::bind(&Encoder::update_rpm, this)); // Setup a timer to refresh the RPM
}

void Lilbot::Encoder::reset_count(){
	_count = 0L;
	_count_prev = 0L;
}

float Lilbot::Encoder::get_rotations(){
	return (float)_count * _ratio; // [rotations]
}

float Lilbot::Encoder::get_angle_degrees(){
	return get_rotations() * 360.0; // [degrees]
}

float Lilbot::Encoder::get_angle_radians(){
	return get_rotations() * TWO_PI; // [radians]
}

float Lilbot::Encoder::get_rpm(){
	return _rpm; // [rotations/minute]
}

void Lilbot::Encoder::update_rpm(){
	float time_current = (float)rclcpp::Time(0).seconds();
	_rpm = ((float)(_count - _count_prev) / (float)(time_current - _time_prev)) * 60.0F * _ratio;
	_count_prev = _count;
	_time_prev = time_current;
	float sum = _rpm;
	for(int i = ENCODER_RPM_BUFFER_SIZE-1; i >= 1; i--){
		_rpm_prev[i] = _rpm_prev[i-1];
		sum += _rpm_prev[i];
	}
	_rpm_prev[0] = _rpm;
	_rpm = sum/ENCODER_RPM_BUFFER_SIZE;
}

void Lilbot::Encoder::encoder_tick_event_callback(int gpio, int level, uint32_t current_us, void *data){
	//Encoder *encoder = (Encoder *) data;
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