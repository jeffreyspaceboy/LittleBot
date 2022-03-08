import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class LineFollower(Node):
    def __init__(self):
        super().__init__('linefollower_cmdvel')
        # Subscribe Infra Red sensors
        # self.subs_right_ir = self.create_subscription(Float64, 'right_IR', self.right_infrared_callback, 1)
        # self.subs_left_ir = self.create_subscription(Float64, 'left_IR', self.left_infrared_callback, 1)
        # self.subs_mid_ir = self.create_subscription(Float64, 'mid_IR', self.mid_infrared_callback, 1)
        # Publish cmd vel
        self.pubs_cmdvel = self.create_publisher(Twist, 'cmd_vel', 1)

        # vehicle parameters
        self.speed = 0.0005
        self.angle_correction = 0.01

        # Initialize parameters
        self.ground_right, self.ground_mid, self.ground_left = 0, 0, 0
        self.delta = 0
        self.cmd = Twist()
        self.stop = False
        self.count = 0
        self.count_threshold = 10

    def lineFollowingModule(self):
        # Constant velocity
        # self.cmd.linear.x = self.speed

        # # Correction parameters
        self.delta = self.ground_right - self.ground_left
        # self.cmd.angular.z = self.angle_correction*self.delta

        # # Logic for stop if black line not seen .
        # if self.ground_right > 500 and self.ground_left > 500 and self.ground_mid > 500:
        #     self.count += 1
        # else:
        #     self.count = 0

        # if self.count > self.count_threshold:
        #     self.stop = True

        # if self.stop:
        #     self.cmd.linear.x = 0.0
        #     self.cmd.angular.z = 0.0

        # # Publish cmd vel
        # self.pubs_cmdvel.publish(self.cmd)
        # self.stop = False

def main(args=None):
    rclpy.init(args=args)
    ls = LineFollower()
    rclpy.spin(ls)
    ls.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()