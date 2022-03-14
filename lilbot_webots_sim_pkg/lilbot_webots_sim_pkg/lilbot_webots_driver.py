import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster,TransformBroadcaster 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import math
from sensor_msgs.msg import LaserScan

from rclpy.time import Time
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler

HALF_DISTANCE_BETWEEN_WHEELS = 0.0925
WHEEL_RADIUS = 0.038

class LilbotWebotsDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__time_step = 0.064

        

        



        self.__motor_wheel_left = self.__robot.getDevice("motor_wheel_left")
        self.__motor_wheel_left.setPosition(float("inf"))
        self.__motor_wheel_left.setVelocity(0)

        self.__motor_wheel_right = self.__robot.getDevice("motor_wheel_right")
        self.__motor_wheel_right.setPosition(float("inf"))
        self.__motor_wheel_right.setVelocity(0)

        # 

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node("lilbot_webots_driver")
        self.__node.create_subscription(Twist, "cmd_vel", self.__cmd_vel_callback, 1)
        

        self.__encoder_wheel_left = self.__robot.getDevice("encoder_wheel_left")
        self.__encoder_wheel_left.enable(1)
        self.__encoder_wheel_right = self.__robot.getDevice("encoder_wheel_right")
        self.__encoder_wheel_right.enable(1)

        self.__lidar= self.__robot.getLidar("lidar")
        self.__lidar.enable(1)
        self.__laser_publisher = self.__node.create_publisher(LaserScan, '/laser_scan', 1)


        ##########################
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.left_omega = 0.0
        self.right_omega = 0.0
        self.odom_pub = self.__node.create_publisher(Odometry,"odom",1)
        self.odom_timer = self.__node.create_timer(0.1, self.publish_odom)
        self.last_time = 0.0
        #########################

    def publish_odom(self):
        stamp = Time(seconds=self.__robot.getTime()).to_msg()

        self.odom_broadcaster = TransformBroadcaster(self.__node)
        time_diff_s = self.__robot.getTime() - self.last_time
        # time_diff_s = self.time_step

        left_wheel_ticks = self.__encoder_wheel_left.getValue()
        right_wheel_ticks = self.__encoder_wheel_right.getValue()

        if time_diff_s == 0.0:
            return

        # Calculate velocities
        v_left_rad = (left_wheel_ticks - self.prev_left_wheel_ticks) / time_diff_s
        v_right_rad = (right_wheel_ticks - self.prev_right_wheel_ticks) / time_diff_s
        v_left = v_left_rad * WHEEL_RADIUS
        v_right = v_right_rad * WHEEL_RADIUS
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / 2 * 2 * self.wheel_gap # (Vright - Vleft) / 2* wheel_gap


        # ################################################################
        # angle_v = self.th+omega
        # vx=v*cos(omega)
        # vy=v*sin(omega)
        # # self.get_logger().info('th = %f , v = %f , omega = %f' % (self.th ,v , omega) )
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


        self.x += v * math.cos(self.prev_angle)*time_diff_s
        self.y += v * math.sin(self.prev_angle)*time_diff_s
        self.th += omega



        ################################################################

        # Reset section
        self.prev_angle = self.th
        self.prev_left_wheel_ticks = left_wheel_ticks
        self.prev_right_wheel_ticks = right_wheel_ticks
        self.last_time = self.__robot.getTime()

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = quaternion_from_euler(0.0, 0.0, self.th)

        # first, we'll publish the transform over tf
        odom_transform = TransformStamped()
        odom_transform.header.stamp = stamp
        odom_transform.header.frame_id = 'odom'
        odom_transform.child_frame_id = 'base_link'
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
        self.laser_pub()


    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    
    def laser_pub(self):
        msg_lidar = LaserScan()
        msg_lidar.header.frame_id = 'base_link'
        stamp = Time(seconds=self.__robot.getTime()).to_msg()
        msg_lidar.header.stamp = stamp
        msg_lidar.angle_min = 0.0
        msg_lidar.angle_max = 2 * 22 / 7
        msg_lidar.angle_increment = ( 0.25 * 22 ) / (180 * 7 )
        msg_lidar.range_min = 0.12
        msg_lidar.range_max = 2.0
        msg_lidar.scan_time = 0.032
        msg_lidar.ranges = self.__lidar.getRangeImage()

        self.__laser_publisher.publish(msg_lidar)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0.0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__motor_wheel_left.setVelocity(command_motor_left)
        self.__motor_wheel_right.setVelocity(command_motor_right)