
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <geometry_msgs/msg/pose.hpp>

// //---PKG-LIBS---//
// #include "little_bot/action/rotate_absolute.hpp"

// #include <math.h>

// class Little_Robot: public rclcpp::Node{
//     public:
//         using RotateAbsoluteGoalHandle = rclcpp_action::ServerGoalHandle<little_bot::action::RotateAbsolute>;

//         Little_Robot(): Node("little_bot"){
//             this->declare_parameter("start_position", std::vector<double>{0.0, 0.0, 0.0});
//             this->declare_parameter("start_orientation", std::vector<double>{0.0, 0.0, 0.0, 1.0});
//             velocity_subr_ = this->create_subscription<geometry_msgs::msg::Twist>("command_velocity", 10, std::bind(&Little_Robot::velocityCallback, this, std::placeholders::_1));
//             std::vector<double> start_position, start_orientation;
//             this->get_parameter("start_position", start_position);
//             this->get_parameter("start_orientation", start_orientation);
//             pose_.position.set__x(start_position[0]);
//             pose_.position.set__y(start_position[1]);
//             pose_.position.set__z(start_position[2]);
//             pose_.orientation.set__x(start_orientation[0]);
//             pose_.orientation.set__y(start_orientation[1]);
//             pose_.orientation.set__z(start_orientation[2]);
//             pose_.orientation.set__w(start_orientation[3]);
//             RCLCPP_INFO(this->get_logger(), "POSE: Position: (x: %0.3f, y: %0.3f, z: %0.3f) ", start_position[0], start_position[1], start_position[2]);
//             RCLCPP_INFO(this->get_logger(), "POSE: Orientation: (x: %0.3f, y: %0.3f, z: %0.3f, w: %0.3f) ", start_orientation[0], start_orientation[1], start_orientation[2], start_orientation[3]);
//         }
//     private:
//         geometry_msgs::msg::Pose pose_;
//         geometry_msgs::msg::Twist twist_;
//         rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subr_;
//         rclcpp_action::Server<little_bot::action::RotateAbsolute>::SharedPtr rotate_absolute_action_server_;
//         std::shared_ptr<RotateAbsoluteGoalHandle> rotate_absolute_goal_handle_;
//         std::shared_ptr<little_bot::action::RotateAbsolute::Feedback> rotate_absolute_feedback_;
//         std::shared_ptr<little_bot::action::RotateAbsolute::Result> rotate_absolute_result_;

//         rclcpp::Time last_command_time_;

//         void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr velocity){
//             last_command_time_ = this->now();
//             twist_.set__linear(velocity->linear);
//             twist_.set__angular(velocity->angular);
//             // lin_vel_x_ = velocity->linear.x;
//             // lin_vel_y_ = velocity->linear.y;
//             // ang_vel_ = velocity->angular.z;

//             if (rotate_absolute_goal_handle_){
//                 RCLCPP_WARN(this->get_logger(), "Velocity command received during rotation goal. Aborting goal");
//                 rotate_absolute_goal_handle_->abort(rotate_absolute_result_);
//                 rotate_absolute_goal_handle_ = nullptr;
//             }else{
//                 RCLCPP_INFO(this->get_logger(), "TWIST: Linear: (x: %0.3f, y: %0.3f, z: %0.3f) ", twist_.linear.x, twist_.linear.y, twist_.linear.z);
//                 RCLCPP_INFO(this->get_logger(), "TWIST: Angular: (x: %0.3f, y: %0.3f, z: %0.3f) ", twist_.angular.x , twist_.angular.y, twist_.angular.z);
//             }   
//         }


//         // void Turtle::rotateAbsoluteAcceptCallback(const std::shared_ptr<RotateAbsoluteGoalHandle> goal_handle){
//         //     // Abort any existing goal
//         //     if (rotate_absolute_goal_handle_){
//         //         RCLCPP_WARN(nh_->get_logger(), "Rotation goal received before a previous goal finished. Aborting previous goal");
//         //         rotate_absolute_goal_handle_->abort(rotate_absolute_result_);
//         //     }
//         //     rotate_absolute_goal_handle_ = goal_handle;
//         //     rotate_absolute_feedback_.reset(new turtlesim::action::RotateAbsolute::Feedback);
//         //     rotate_absolute_result_.reset(new turtlesim::action::RotateAbsolute::Result);
//         //     rotate_absolute_start_orient_ = orient_;
//         // }
// };

// int main(int argc, char * argv[]){
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<Little_Robot>());
//   rclcpp::shutdown();
//   return 0;
// }