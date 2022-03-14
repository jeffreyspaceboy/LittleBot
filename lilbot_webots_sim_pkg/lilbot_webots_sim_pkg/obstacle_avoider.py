import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


MAX_RANGE = 0.15


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__("obstacle_avoider")

        self.__publisher = self.create_publisher(Twist, "cmd_vel", 1)
        self.create_timer(0.2, self.timer_callback)
        self.speed = 0.0
        

    def timer_callback(self):
        command_message = Twist()

        if(self.speed == 0.0):
            command_message.linear.x = 0.5
            self.speed = 0.5
        else:
            command_message.linear.x = 0.0
            self.speed = 0.0

        self.__publisher.publish(command_message)


def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()