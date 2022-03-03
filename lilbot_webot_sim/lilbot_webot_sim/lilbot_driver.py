import rclpy
from geometry_msgs.msg import Twist
from time import sleep

HALF_DISTANCE_BETWEEN_WHEELS = 0.094
WHEEL_RADIUS = 0.038

class LilBotDriver:
    def init(self, webots_node, properties):
        self._robot = webots_node.robot

        self._left_motor = self._robot.getDevice("left_wheel_motor")
        self._right_motor = self._robot.getDevice("right_wheel_motor")

        self._left_motor.setPosition(float("inf"))
        self._right_motor.setPosition(float("inf"))
        self._left_motor.setVelocity(0)
        self._right_motor.setVelocity(0)

        self._target_twist = Twist()

        rclpy.init(args=None)
        self._node = rclpy.create_node("lilbot_driver")
        self._node.create_subscription(Twist, "cmd_vel", self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self._target_twist = twist

    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0)

        forward_speed = self._target_twist.linear.x
        angular_speed = self._target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self._left_motor.setVelocity(command_motor_left)
        self._right_motor.setVelocity(command_motor_right)