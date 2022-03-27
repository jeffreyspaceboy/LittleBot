
import math
import rclpy
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster 
import tf_transformations

HALF_DISTANCE_BETWEEN_WHEELS = 0.0925
WHEEL_RADIUS = 0.038

# Based on: https://docs.ros.org/en/foxy/Tutorials/Simulators/Webots/Setting-up-a-Robot-Simulation-Webots.html

class LilbotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = 0.032

        # ===== WHEEL MOTORS ===== #
        self.__motor_wheel_left = self.__robot.getMotor("motor_wheel_left")
        self.__motor_wheel_right = self.__robot.getMotor("motor_wheel_right")
        self.__motor_wheel_left.setPosition(float("inf"))
        self.__motor_wheel_right.setPosition(float("inf"))
        self.__motor_wheel_left.setVelocity(0.0)
        self.__motor_wheel_right.setVelocity(0.0)

        # ===== SENSORS ===== #
        self.__encoder_wheel_left = self.__robot.getPositionSensor("encoder_wheel_left")
        self.__encoder_wheel_right = self.__robot.getPositionSensor("encoder_wheel_right")
        self.__encoder_wheel_left.enable(1)
        self.__encoder_wheel_right.enable(1)

        self.__lidar_sensor = self.__robot.getLidar("lidar")
        self.__lidar_sensor.enable(1)
        self.__lidar_sensor.enablePointCloud()
        ###rotation 0 0 1 3.14159265359

        # ===== ODOMETRY ===== #
        self.__odom = Odometry()

        self.__x = 0.0
        self.__y = 0.0
        self.__h = 0.0

        self.__odom_time_prev = 0.0
        self.__wheel_ticks_left_prev = 0.0
        self.__wheel_ticks_right_prev = 0.0
        self.__heading_prev = 0.0

        # ===== TWIST ===== #
        self.__target_twist = Twist()


        # ===== NODE ===== #
        rclpy.init(args=None)
        self.__node = rclpy.create_node("lilbot_webots_driver")

        # ===== SUBSCRIBERS ===== #
        self.__node.create_subscription(Twist,"cmd_vel", self.cmd_vel_callback,1)

        
        


        # ===== PUBLISHERS ===== #
        self.__odom_publisher = self.__node.create_publisher(Odometry,"odom",1)
        self.__laser_publisher = self.__node.create_publisher(LaserScan, "scan", 1)
        self.__node.create_timer(self.__timestep, self.publish_odom)
        self.__node.create_timer(self.__timestep, self.laser_pub)
        
    def cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def refresh_vel(self):
        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z
        command_wheel_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_wheel_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        self.__motor_wheel_left.setVelocity(command_wheel_motor_left)
        self.__motor_wheel_right.setVelocity(command_wheel_motor_right)

    def publish_odom(self):
        odom_time = self.__robot.getTime()
        timestamp = rclpy.time.Time(seconds = self.__robot.getTime()).to_msg() 
        time_delta = odom_time - self.__odom_time_prev # [sec]

        ticks_left = self.__encoder_wheel_left.getValue()
        ticks_right = self.__encoder_wheel_right.getValue()

        if time_delta == 0.0:
            return

        # Calculate Velocities
        v_left_radians = (ticks_left - self.__wheel_ticks_left_prev) / time_delta
        v_right_radians = (ticks_right - self.__wheel_ticks_right_prev) / time_delta
        v_left = v_left_radians * WHEEL_RADIUS
        v_right = v_right_radians * WHEEL_RADIUS
        velocity = (v_left + v_right) / 2.0
        self.__h += ((v_right - v_left) / 2.0) * (2.0 * HALF_DISTANCE_BETWEEN_WHEELS) # (Vright - Vleft) / 2* wheel_gap
        self.__x += velocity * math.cos(self.__heading_prev)*time_delta
        self.__y += velocity * math.sin(self.__heading_prev)*time_delta

        ################################################################

        # Reset section
        self.__heading_prev = self.__h
        self.__wheel_ticks_left_prev = ticks_left
        self.__wheel_ticks_right_prev = ticks_right
        self.__odom_time_prev = odom_time

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.__h)
        odom_quaternion = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # First, we'll publish the transform over tf
        odom_transform = TransformStamped()
        odom_transform.header.stamp = timestamp
        odom_transform.header.frame_id = "base_link"
        odom_transform.child_frame_id = "odom"
        odom_transform.transform.rotation = odom_quaternion
        odom_transform.transform.translation.x = self.__x
        odom_transform.transform.translation.y = self.__y
        odom_transform.transform.translation.z = 0.0
        TransformBroadcaster(self.__node).sendTransform(odom_transform)

        self.__odom.header.stamp = timestamp
        self.__odom.header.frame_id = "base_link"
        self.__odom.child_frame_id = "odom"
        self.__odom.pose.pose.position.x = self.__x
        self.__odom.pose.pose.position.y = self.__y
        self.__odom.pose.pose.orientation = odom_quaternion
        self.__odom.twist.twist.linear.x = self.__target_twist.linear.x
        self.__odom.twist.twist.angular.z = self.__target_twist.angular.z

        self.__odom_publisher.publish(self.__odom)

    def laser_pub(self):
        msg_lidar = LaserScan()
        msg_lidar.header.frame_id = "odom"
        stamp = rclpy.time.Time(seconds=self.__robot.getTime()).to_msg()
        msg_lidar.header.stamp = stamp
        msg_lidar.angle_min = 0.0
        msg_lidar.angle_max = 2 * (22 / 7) # 22/7 = PI
        msg_lidar.angle_increment = (22 / 7) * (1 / 180) #( 0.25 * 22 ) / (180 * 7 ) 
        msg_lidar.range_min = 0.12
        msg_lidar.range_max = 8.0
        msg_lidar.scan_time = self.__timestep
        msg_lidar.ranges = self.__lidar_sensor.getRangeImage()
        self.__laser_publisher.publish(msg_lidar)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec = 0.0)
        self.refresh_vel()
        self.publish_odom()