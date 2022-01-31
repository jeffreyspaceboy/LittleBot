//---SYS-LIBS---//

//---ROS-LIBS---//
#include "rclcpp/rclcpp.hpp"

//---PKG-LIBS---//
#include "geometry_msgs/msg/twist.hpp"

class Little_Robot : public rclcpp::Node{
    public:
        Little_Robot() : Node("little_robot"){
            velocity_subr_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Little_Robot::velocityCallback, this, std::placeholders::_1));
        }
    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subr_;
        rclcpp::Time last_command_time_;

        void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr vel){
            last_command_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Velocity Request: ('%f', '%f') ", vel->linear.x, vel->linear.y);
        }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Little_Robot>());
  rclcpp::shutdown();
  return 0;
}