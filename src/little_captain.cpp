//---SYS-LIBS---//
#include <chrono>

//---ROS-LIBS---//
#include "rclcpp/rclcpp.hpp"

//---PKG-LIBS---//
#include "geometry_msgs/msg/twist.hpp"

class Little_Captain : public rclcpp::Node{
	public:
		Little_Captain(): Node("little_captain"){
			twist_pubr_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
			using namespace std::chrono_literals; // Allows for "2000ms"
			timer_ = this->create_wall_timer(2000ms, std::bind(&Little_Captain::timer_callback, this));
		}

	private:
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pubr_;
		rclcpp::TimerBase::SharedPtr timer_;
		int count_ = 0;

		void timer_callback(){
			geometry_msgs::msg::Twist twist;
			twist.linear.x = count_;
			twist.linear.y = count_*2;
			RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f'", twist.linear.x, twist.linear.y);
			twist_pubr_->publish(twist);    
			count_++;
		}
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Little_Captain>());
  rclcpp::shutdown();
  return 0;
}