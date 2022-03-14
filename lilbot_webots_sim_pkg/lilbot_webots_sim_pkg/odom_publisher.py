import rclpy
from rclpy.node import Node
# from webots_ros2_core.webots_node import WebotsNode
from datetime import datetime
import math
from std_msgs.msg import Float64
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from rclpy.clock import ROSClock


class Odometry_pub(Node):
    def __init__(self):
        super().__init__('odom_pub')
        
        # Create Subscriber
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.cmdVel_callback, 1)

        # Create Lidar subscriber
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.time_step=0.02
        self.odom_pub = self.create_publisher(Odometry,"odom",1)
        self.odom_timer = self.create_timer(self.time_step, self.odom_callback)

    def odom_callback(self):
        self.publish_odom()


    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return [qx, qy, qz, qw]

    def publish_odom(self):
        self.odom_broadcaster = TransformBroadcaster(self)
        # self.current_time = datetime.now().microsecond
        # compute odometry in a typical way given the velocities of the robot
        dt = self.time_step
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        # odom_quat=[0.0,0.0,0.0,1.0]
        odom_quat=self.euler_to_quaternion(self.th,0,0)

        # first, we'll publish the transform over tf
        odom_transform = TransformStamped()
        odom_transform.header.stamp = self.get_clock().now().to_msg()

        # odom_transform.header.stamp = Time(seconds=self.getTime().now()).to_msg()
        odom_transform.header.frame_id = 'odom'
        odom_transform.child_frame_id = 'base_link'
        odom_transform.transform.rotation.x = odom_quat[0]
        odom_transform.transform.rotation.y = odom_quat[1]
        odom_transform.transform.rotation.z = odom_quat[2]
        odom_transform.transform.rotation.w = odom_quat[3]
        odom_transform.transform.translation.x = self.x
        odom_transform.transform.translation.y = self.y
        odom_transform.transform.translation.z = 0.0
        # self.get_logger().info('base_link to odom being published : %d' % self.get_clock().now())

        self.odom_broadcaster.sendTransform(odom_transform)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        # odom.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        # set the position
        odom.pose.pose.position.x= self.x
        odom.pose.pose.position.y= self.y
        odom.pose.pose.orientation.x=odom_quat[0]
        odom.pose.pose.orientation.y=odom_quat[1]
        odom.pose.pose.orientation.z=odom_quat[2]
        odom.pose.pose.orientation.w= odom_quat[3]

        # set the velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z=self.vth

        # publish the message
        self.odom_pub.publish(odom)

        # self.last_time = self.current_time

    def cmdVel_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z

def main(args=None):
    rclpy.init(args=args)
    client_vel = Odometry_pub()
    rclpy.spin(client_vel)

    client_vel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()