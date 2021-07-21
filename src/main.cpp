#include "../include/Robot.hpp"

int main(int argc, char * argv[]){
    gpioInitialise();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot>());
    rclcpp::shutdown();
    gpioTerminate();
    return 0;
}