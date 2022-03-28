#include "lilbot_driver/lilbot_pid_controller.hpp"

namespace Lilbot{
	PID_Controller::PID_Controller(const std::string &node_name, const std::string &service_name, float kP, float kI, float kD) : 
		Node(node_name),
		_target(0.0),
		_kp(kP), _ki(kI), _kd(kD),
		_error(0.0), _prev_error(0.0), _error_tolerance(0.0), _error_integral(0.0), _dedt(0.0),
		_prev_time(0.0) 
	{    
		// _kp = kP;
		// _ki = kI;
		// _kd = kD;
		auto handle_service = [this](            
			const std::shared_ptr<rmw_request_id_t> request_header,
			const std::shared_ptr<lilbot_msgs::srv::Pid::Request> request,
			std::shared_ptr<lilbot_msgs::srv::Pid::Response> response) -> void {      
				(void)request_header;
				float current_time_sec = (float)request->stamp.sec + ((float)request->stamp.nanosec / 1E9F);
				response->control_signal = control(request->current, request->target, request->tolerance, current_time_sec);
				RCLCPP_INFO(this->get_logger(), "Target: %0.3f, Current: %0.3f, Error: %0.3f, Signal: %0.3f", _target, request->current, _error, response->control_signal);   
			};     
		_service = this->create_service<lilbot_msgs::srv::Pid>(service_name, handle_service);  
	} 

	float PID_Controller::control(float current, float target, float tolerance, float current_time){
		float dt = current_time - _prev_time;			// Calculate Time Delta [sec]

		_target = target;  								// Update Target
		_error_tolerance = tolerance;					// Update Error Tolerance

		_error = _target - current;						// Proportional
		if(_error < _error_tolerance  || _error > -_error_tolerance) { _error = 0.0; } // If error is within the user defined tolerance range, act as if there is no error.
		_error_integral += _error * dt;					// Integral
		_dedt = (_error - _prev_error) / dt;			// Derivative 

		_prev_error = _error;							// Update Previous Error
		_prev_time = current_time;						// Update Previous Time
		return (_kp * _error) + (_ki * _error_integral) + (_kd * _dedt); // Return the Control Signal
	}
}

// int main(int argc, char *argv[]) {  
// 	rclcpp::init(argc, argv);  
// 	auto node = std::make_shared<Lilbot::PID_Controller>("pid_node","pid_service", 20.0, 1.55, 0.05);  
// 	rclcpp::spin(node);  
// 	return 0;
// }