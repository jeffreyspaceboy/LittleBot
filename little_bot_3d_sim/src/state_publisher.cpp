#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <functional>
#include <memory>
using namespace std::chrono_literals;

using std::placeholders::_1;

class StatePublisher : public rclcpp::Node{
	public:
		StatePublisher() : Node("state_publisher"), angle_(0.0){
			// Initialize the transform broadcaster
			tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
			timer_ = this->create_wall_timer(33ms, std::bind(&StatePublisher::timer_callback, this));
			
		}

	private:
		void handle_pose()
		{
			rclcpp::Time now = this->get_clock()->now();
			geometry_msgs::msg::TransformStamped t;

			// Read message content and assign it to
			// corresponding tf variables
			t.header.stamp = now;
			t.header.frame_id = "world";
			t.child_frame_id = "body";

			// Turtle only exists in 2D, thus we get x and y translation
			// coordinates from the message and set the z coordinate to 0
			t.transform.translation.x = cos(angle_);
			t.transform.translation.y = sin(angle_);
			t.transform.translation.z = 0.0;

			// For the same reason, turtle can only rotate around one axis
			// and this why we set rotation in x and y to 0 and obtain
			// rotation in z axis from the message
			tf2::Quaternion q;
			q.setRPY(0.0, 0.0, angle_ + 3.13159265/2);
			t.transform.rotation.x = q.x();
			t.transform.rotation.y = q.y();
			t.transform.rotation.z = q.z();
			t.transform.rotation.w = q.w();

			double degree = 3.14159265 / 180.0;

			// Send the transformation
			tf_broadcaster_->sendTransform(t);
			angle_ += degree/4;
		}
		void timer_callback()
		{
			auto message = std_msgs::msg::String();
			message.data = std::to_string(angle_);
			RCLCPP_INFO(this->get_logger(), "Angle: '%s'", message.data.c_str());
			this->handle_pose();
		}
		
		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
		rclcpp::TimerBase::SharedPtr timer_;
		double angle_;
};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StatePublisher>());
	rclcpp::shutdown();
	return 0;
}