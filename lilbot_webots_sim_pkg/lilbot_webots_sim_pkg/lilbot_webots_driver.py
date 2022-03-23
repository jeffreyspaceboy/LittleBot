import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.0925
WHEEL_RADIUS = 0.038

# Based on: https://docs.ros.org/en/foxy/Tutorials/Simulators/Webots/Setting-up-a-Robot-Simulation-Webots.html

class LilbotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__motor_wheel_left = self.__robot.getMotor("motor_wheel_left")
        self.__motor_wheel_left.setPosition(float("inf"))
        self.__motor_wheel_left.setVelocity(0.0)

        self.__motor_wheel_right = self.__robot.getMotor("motor_wheel_right")
        self.__motor_wheel_right.setPosition(float("inf"))
        self.__motor_wheel_right.setVelocity(0.0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node("lilbot_webots_driver")
        self.__node.create_subscription(Twist, "cmd_vel", self.__cmd_vel_callback, 1)
        
    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec = 0.0)
        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z
        command_wheel_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_wheel_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        self.__motor_wheel_left.setVelocity(command_wheel_motor_left)
        self.__motor_wheel_right.setVelocity(command_wheel_motor_right)