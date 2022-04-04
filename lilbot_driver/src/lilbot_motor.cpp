/*---LILBOT_MOTOR_CPP---*/


/*---C---*/
#include <unistd.h>
#ifdef __aarch64__
#include <pigpio.h>
#endif

/*---LILBOT---*/
#include "lilbot_driver/lilbot_motor.hpp"

Lilbot::Motor::Motor(const std::string &motor_name, uint8_t gpio_enable_pin, uint8_t gpio_pin_phase_a, uint8_t gpio_pin_phase_b, int reverse, int max_power, Encoder *new_encoder, PID_Controller *new_pid_velocity_controller, unsigned int rpm_control_refresh_us) : 
	_rpm_control_enabled(true),
	_name(motor_name),
	_gpio_pin_enable(gpio_enable_pin),
	_gpio_pin_phase_a((reverse) ? gpio_pin_phase_a : gpio_pin_phase_b),
	_gpio_pin_phase_b((reverse) ? gpio_pin_phase_b : gpio_pin_phase_a),
	_rpm_target(0.0F), _prev_target_rpm(0.0F),
	_power(0), _power_max(max_power),
	_rpm_control_refresh_us(rpm_control_refresh_us),
	_encoder(new_encoder),
	_pid_velocity_controller(new_pid_velocity_controller)
{

	#ifdef __aarch64__
	gpioSetMode(_gpio_pin_enable, PI_OUTPUT);
	gpioSetMode(_gpio_pin_phase_a, PI_OUTPUT);
	gpioSetMode(_gpio_pin_phase_b, PI_OUTPUT);
	#ifdef MOTOR_PWM_FREQUENCY
	gpioSetPWMfrequency(_gpio_pin_enable, MOTOR_PWM_FREQUENCY);
	#endif
	#endif

	_thread = std::thread(&Lilbot::Motor::_rpm_control_thread, this);
}

Lilbot::Motor::~Motor(){
	_mutex.lock();
	_rpm_control_enabled = false;
	_thread.join();
	gpioWrite(_gpio_pin_enable, 0);
	gpioWrite(_gpio_pin_phase_a, 0);
	gpioWrite(_gpio_pin_phase_b, 0);
	_mutex.unlock();
}

/* SET FUNCTIONS */
void Lilbot::Motor::set_rpm(float rpm_target){
	_mutex.lock();
	_rpm_target = rpm_target;
	_mutex.unlock();
}

/* GET FUNCTIONS */
float Lilbot::Motor::get_rotations(){ return _encoder->get_rotations(); }
float Lilbot::Motor::get_angle_degrees(){ return _encoder->get_angle_degrees(); }
float Lilbot::Motor::get_angle_radians(){ return _encoder->get_angle_radians(); }
float Lilbot::Motor::get_rpm(){ return _encoder->get_rpm(); }
int Lilbot::Motor::get_power(){
	_mutex.lock();
	int power = _power;
	_mutex.unlock();
	return power;
}

/* MOTION FUNCTIONS */
void Lilbot::Motor::spin(int new_power){
	#ifdef __aarch64__
	_mutex.lock();
	_power = new_power;
	if(_power > _power_max || _power < -_power_max){
		gpioPWM(_gpio_pin_enable, _power_max);
	}else{
		gpioPWM(_gpio_pin_enable, (_power >= 0) ? _power : -_power);
	}
	gpioWrite(_gpio_pin_phase_a, (_power > 0) ? 1 : 0);
	gpioWrite(_gpio_pin_phase_b, (_power < 0) ? 1 : 0);
	_mutex.unlock();
	#else
	_mutex.lock();
	_power = new_power;
	_mutex.unlock();
	#endif
}
void Lilbot::Motor::stop(){
	#ifdef __aarch64__
	_mutex.lock();
	gpioWrite(_gpio_pin_enable, 0);
	gpioWrite(_gpio_pin_phase_a, 0);
	gpioWrite(_gpio_pin_phase_b, 0);
	_mutex.unlock();
	#endif
}

void Lilbot::Motor::_rpm_control_thread(){
	float time_current = 0.0;
	int new_power = 0;
	unsigned int microseconds;
	while(_rpm_control_enabled){
		_mutex.lock();
		#ifdef __aarch64__
		time_current = ((float)gpioTick()) / 1E6; // [seconds]
		#endif
		if(_prev_target_rpm != _rpm_target){ 
			_pid_velocity_controller->control(get_rpm(), _rpm_target, 0.0, time_current);
		}
		_prev_target_rpm = _rpm_target;
		new_power = (int)_pid_velocity_controller->control(get_rpm(), _rpm_target, 0.0, time_current, 100.0F);
		microseconds = _rpm_control_refresh_us;
		_mutex.unlock();
		spin(new_power);
		usleep(microseconds);
	}
}