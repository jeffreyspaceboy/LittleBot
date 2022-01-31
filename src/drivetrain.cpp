// /*---DRIVETRAIN_CPP---*/
// /*----------------------------------------------------------------------------*/
// /*    Module:       drivetrain.cpp                                            */
// /*    Author:       Jeffrey Fisher II                                         */
// /*    Edited:       2022-01-30                                                */
// /*----------------------------------------------------------------------------*/

// /* SYSTEM LIBRARIES */
// #include <chrono>

// /* ROS LIBRARIES */
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"

// /* PACKAGE LIBRARIES */
// #include "little_bot/msg/num.hpp"

// class Drivetrain : public rclcpp::Node{
// 	public:
// 		Drivetrain(): Node("drivetrain"), count_(0){
// 			publisher_ = this->create_publisher<little_bot::msg::Num>("topic1", 10);

// 			using namespace std::chrono_literals; // Allows for "500ms"
// 			timer_ = this->create_wall_timer(500ms, std::bind(&Drivetrain::timer_callback, this));
// 			this->declare_parameter<std::string>("my_parameter", "world");
// 		}

// 	private:
// 		rclcpp::Publisher<little_bot::msg::Num>::SharedPtr publisher_; 
// 		rclcpp::TimerBase::SharedPtr timer_;
// 		size_t count_;

// 		void timer_callback(){
// 			little_bot::msg::Num message = little_bot::msg::Num();
// 			message.num = count_++;
// 			RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.num);
// 			publisher_->publish(message);
// 		}
// };

// int main(int argc, char * argv[]){
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<Drivetrain>());
//   rclcpp::shutdown();
//   return 0;
// }

// /*---DRIVETRAIN_CPP---*/