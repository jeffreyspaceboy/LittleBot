import imp
import rclpy
from rclpy.time import Time
from webots_ros2_core.webots_node import WebotsNode

# Interfaces
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from tf2_ros import StaticTransformBroadcaster,TransformBroadcaster 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import tf_transformations

from math import cos, sin , tan , pi

HALF_DISTANCE_BETWEEN_WHEELS = 0.094
WHEEL_RADIUS = 0.038



class ServiceNodeVelocity(WebotsNode):
    def init(self, args):
        super().__init__("slave_node", args)

        self.timestep = 32
        
        # SENSORS
        self.sensor_timer = self.create_timer(0.001 * self.timestep, self.sensor_callback)
        self.lidar = self.robot.getLidar("lidar")
        self.lidar.enable(self.timestep)
        self.laser_publisher = self.create_publisher(LaserScan, "/scan", 1)

        self.left_wheel_sensor = self.robot.getPositionSensor("left_wheel_encoder")
        self.left_wheel_sensor.enable(self.timestep)

        self.right_wheel_sensor = self.robot.getPositionSensor("right_wheel_encoder")
        self.right_wheel_sensor.enable(self.timestep)

        # MOTORS
        self.left_wheel_motor = self.robot.getDevice("left_wheel_motor")
        self.left_wheel_motor.setPosition(float("inf"))
        self.left_wheel_motor.setVelocity(0.0)

        self.right_wheel_motor = self.robot.getDevice("right_wheel_motor")
        self.right_wheel_motor.setPosition(float("inf"))
        self.right_wheel_motor.setVelocity(0.0)

        self.motor_max_speed = self.left_wheel_motor.getMaxVelocity()

        # PUBLISHERS

        # SUBSCRIBERS
        self.target_twist = Twist()
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)

        ##########################
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.time_step=0.032
        self.left_omega = 0.0
        self.right_omega = 0.0
        self.odom_pub = self.create_publisher(Odometry,"odom",1)
        self.odom_timer = self.create_timer(self.time_step, self.odom_callback)
        #########################
        self.get_logger().info("Sensor enabled")
        self.prev_angle = 0.0
        self.prev_left_wheel_ticks = 0.0
        self.prev_right_wheel_ticks = 0.0
        self.last_time = 0.0
        self.wheel_gap = HALF_DISTANCE_BETWEEN_WHEELS * 2.0  # in meter
        self.wheel_radius = WHEEL_RADIUS  # in meter
        self.front_back = 0.1 # in meter

    def odom_callback(self):
        self.publish_odom()

    def publish_odom(self):
        stamp = Time(seconds=self.robot.getTime()).to_msg()

        self.odom_broadcaster = TransformBroadcaster(self)
        time_diff_s = self.robot.getTime() - self.last_time
        # time_diff_s = self.time_step

        left_wheel_ticks = self.left_wheel_sensor.getValue()
        right_wheel_ticks = self.right_wheel_sensor.getValue()

        if time_diff_s == 0.0:
            return

        # Calculate velocities
        v_left_rad = (left_wheel_ticks - self.prev_left_wheel_ticks) / time_diff_s
        v_right_rad = (right_wheel_ticks - self.prev_right_wheel_ticks) / time_diff_s
        v_left = v_left_rad * self.wheel_radius
        v_right = v_right_rad * self.wheel_radius
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / 2 * 2 * self.wheel_gap # (Vright - Vleft) / 2* wheel_gap


        # ################################################################
        # angle_v = self.th+omega
        # vx=v*cos(omega)
        # vy=v*sin(omega)
        # # self.get_logger().info("th = %f , v = %f , omega = %f" % (self.th ,v , omega) )
        # dx = (cos(angle_v)*vx - sin(angle_v)*vy)*time_diff_s
        # dy = (sin(angle_v)*vx + cos(angle_v)*vy)*time_diff_s
        # dth = tan(omega)*vx*time_diff_s / self.front_back

        # self.x += dx
        # self.y += dy
        # self.th += omega


        # # Calculate position & angle
        # # Fourth order Runge - Kutta
        # # Reference: https://www.cs.cmu.edu/~16311/s07/labs/NXTLabs/Lab%203.html
        # k00 = v * cos(self.prev_angle)
        # k01 = v * sin(self.prev_angle)
        # k02 = omega
        # k10 = v * cos(self.prev_angle + time_diff_s * k02 / 2)
        # k11 = v * sin(self.prev_angle + time_diff_s * k02 / 2)
        # k12 = omega
        # k20 = v * cos(self.prev_angle + time_diff_s * k12 / 2)
        # k21 = v * sin(self.prev_angle + time_diff_s * k12 / 2)
        # k22 = omega
        # k30 = v * cos(self.prev_angle + time_diff_s * k22 / 2)
        # k31 = v * sin(self.prev_angle + time_diff_s * k22 / 2)
        # k32 = omega


        self.x += v * cos(self.prev_angle)*time_diff_s
        self.y += v * sin(self.prev_angle)*time_diff_s
        self.th += omega



        ################################################################

        # Reset section
        self.prev_angle = self.th
        self.prev_left_wheel_ticks = left_wheel_ticks
        self.prev_right_wheel_ticks = right_wheel_ticks
        self.last_time = self.robot.getTime()

        # since all odometry is 6DOF we"ll need a quaternion created from yaw
        odom_quat=tf_transformations.quaternion_from_euler(0.0, 0.0, self.th)

        # first, we"ll publish the transform over tf
        odom_transform = TransformStamped()
        odom_transform.header.stamp = stamp
        odom_transform.header.frame_id = "odom"
        odom_transform.child_frame_id = "base_link"
        odom_transform.transform.rotation = odom_quat
        odom_transform.transform.translation.x = self.x
        odom_transform.transform.translation.y = self.y
        odom_transform.transform.translation.z = 0.0

        self.odom_broadcaster.sendTransform(odom_transform)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        # set the position
        odom.pose.pose.position.x= self.x
        odom.pose.pose.position.y= self.y
        odom.pose.pose.orientation = odom_quat
        # set the velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z=self.vth

        # publish the message
        self.odom_pub.publish(odom)
    

    # def cmd_vel_callback(self, twist):
    #     self._target_twist = twist

    #     forward_speed = self.target_twist.linear.x
    #     angular_speed = self.target_twist.angular.z

    #     command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
    #     command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

    #     self.left_wheel_motor.setVelocity(command_motor_left)
    #     self.right_wheel_motor.setVelocity(command_motor_right)

    def cmdVel_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z
        left_speed = ((2.0 * msg.linear.x - msg.angular.z * self.wheel_gap) / (2.0 * self.wheel_radius))
        right_speed = ((2.0 * msg.linear.x + msg.angular.z * self.wheel_gap) / (2.0 * self.wheel_radius))
        left_speed = min(self.motor_max_speed, max(-self.motor_max_speed, left_speed))
        right_speed = min(self.motor_max_speed, max(-self.motor_max_speed, right_speed))

        self.left_omega = left_speed / (self.wheel_radius)
        self.right_omega = right_speed / (self.wheel_radius)
        self.left_wheel_motor.setVelocity(left_speed)
        self.right_wheel_motor.setVelocity(right_speed)

    def sensor_callback(self):
        self.laser_pub()

    def laser_pub(self):
        msg_lidar = LaserScan()
        msg_lidar.header.frame_id = "base_link"
        stamp = Time(seconds=self._robot.getTime()).to_msg()
        msg_lidar.header.stamp = stamp
        msg_lidar.angle_min = 0.0
        msg_lidar.angle_max = 2 * 22 / 7
        msg_lidar.angle_increment = ( 0.25 * 22 ) / (180 * 7 )
        msg_lidar.range_min = 0.12
        msg_lidar.range_max = 2.0
        msg_lidar.scan_time = 0.032
        msg_lidar.ranges = self._lidar.getRangeImage()
        self._laser_publisher.publish(msg_lidar)

def main(args=None):
    rclpy.init(args=args)
    client_vel = ServiceNodeVelocity("slave_node", args=args)
    rclpy.spin(client_vel)
    client_vel.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()