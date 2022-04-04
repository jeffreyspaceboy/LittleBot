/*---LILBOT_PID_CONTROLLER_CPP---*/


/*---LILBOT---*/
#include "lilbot_driver/lilbot_pid_controller.hpp"

Lilbot::PID_Controller::PID_Controller(const std::string &pid_controller_name, float kP, float kI, float kD) : 
	_name(pid_controller_name),
	_target(0.0),
	_kp(kP), _ki(kI), _kd(kD),
	_error(0.0), _error_prev(0.0), _error_tolerance(0.0), _error_integral(0.0), _dedt(0.0),
	_time_prev(0.0) 
{
	// Initialize PID Controller
}

float Lilbot::PID_Controller::control(float current, float target, float tolerance, float current_time_sec, float timeout_sec)
{
	float dt = current_time_sec - _time_prev;		// Calculate Time Delta [sec]

	_target = target;  								// Update Target
	_error_tolerance = tolerance;					// Update Error Tolerance

	_error = _target - current;						// Proportional
	if(_error < _error_tolerance  || _error > -_error_tolerance || dt > timeout_sec) 
	{ 
		// If error is within the user defined tolerance range, act as if there is no error.
		// Also, if there is too big of a time gap since the last time the controller was used, otherwise there may be an extreme PID response.
		_error = 0.0; 
	}
	_error_integral += _error * dt;					// Integral
	_dedt = (_error - _error_prev) / dt;			// Derivative 
	float control_output = (_kp * _error) + (_ki * _error_integral) + (_kd * _dedt); // Return the Control Signal

	_error_prev = _error;							// Update Previous Error
	_time_prev = current_time_sec;					// Update Previous Time
	return control_output;
}