# #! /usr/bin/env python3

# from Motor import Motor
# import math

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from geometry_msgs.msg import Twist

# class Robot(Node):
#     def __init__(self, name,
#                  pinRightFwd, pinRightRev, pinLeftFwd, pinLeftRev,
#                  wheel_diameter=67.77, wheel_base=0.15,
#                  left_max_rpm=100.0, right_max_rpm=100.0,
#                  frequency=20):

#         super().__init__(name)

#         self.frequency = frequency
#         self.left_max_rpm = left_max_rpm
#         self.right_max_rpm = right_max_rpm
#         self.wheel_diameter = wheel_diameter
#         self.wheel_base = wheel_base
#         self.rightWheel = Motor(pinRightFwd, pinRightRev, frequency)
#         self.leftWheel = Motor(pinLeftFwd, pinLeftRev, frequency)

#         self.speed = 0.0
#         self.spin = 0.0
#         self.close = 0.30  # start slowing down when this close
#         self.tooclose = 0.10   # no forward motion when this close
#         self.distance = 100.0

#         self._command_subscription = self.create_subscription( String, 'command', self._command_callback, 10)
#         self._cmd_vel_subscription = self.create_subscription( Twist, 'cmd_vel', self._cmd_vel_callback, 2)

#     def stop(self):
#         self._leftWheel.stop()
#         self._rightWheel.stop()

#     def max_speed(self):
#         '''Speed in meters per second at maximum RPM'''
#         rpm = (self._left_max_rpm + self._right_max_rpm) / 2.0
#         mps = rpm * math.pi * self._wheel_diameter / 60.0
#         return mps

#     def max_twist(self):
#         '''Rotation in radians per second at maximum RPM'''
#         return self.max_speed() / self._wheel_diameter

#     def _forward(self, speed=100):
#         self._rightWheel.forward(speed)
#         self._leftWheel.forward(speed)

#     def _reverse(self, speed=100):
#         self._rightWheel.reverse(speed)
#         self._leftWheel.reverse(speed)

#     def _left(self, speed=100):
#         self._rightWheel.reverse(speed)
#         self._leftWheel.forward(speed)

#     def _right(self, speed=100):
#         self._rightWheel.forward(speed)
#         self._leftWheel.reverse(speed)

#     def _command_callback(self, msg):
#         command = msg.data
#         if command == 'forward':
#             self._forward()
#         elif command == 'reverse':
#             self._reverse()
#         elif command == 'left':
#             self._left()
#         elif command == 'right':
#             self._right()
#         elif command == 'stop':
#             self.stop()
#         else:
#             print('Unknown command, stopping instead')
#             self.stop()

#     def _cmd_vel_callback(self, msg):
#         self.speed = msg.linear.x
#         self.spin = msg.angular.z
#         self._set_motor_speeds()

#     def _range_callback(self, msg):
#         self.distance = msg.range
#         self._set_motor_speeds()

#     def _set_motor_speeds(self):
#         # TODO: inject a stop() if no speeds seen for a while
#         #
#         # Scary math ahead.
#         #
#         # First figure out the speed of each wheel based on spin: each wheel
#         # covers self._wheel_base meters in one radian, so the target speed
#         # for each wheel in meters per sec is spin (radians/sec) times
#         # wheel_base divided by wheel_diameter
#         #
#         right_twist_mps = self.spin * self._wheel_base / self._wheel_diameter
#         left_twist_mps = -1.0 * self.spin * \
#             self._wheel_base / self._wheel_diameter
#         #
#         # Now add in forward motion.
#         #
#         left_mps = self.speed + left_twist_mps
#         right_mps = self.speed + right_twist_mps
#         #
#         # Convert meters/sec into RPM: for each revolution, a wheel travels
#         # pi * diameter meters, and each minute has 60 seconds.
#         #
#         left_target_rpm = (left_mps * 60.0) / (math.pi * self._wheel_diameter)
#         right_target_rpm = (right_mps * 60.0) / \
#             (math.pi * self._wheel_diameter)
#         #
#         left_percentage = (left_target_rpm / self._left_max_rpm) * 100.0
#         right_percentage = (right_target_rpm / self._right_max_rpm) * 100.0
#         #
#         # clip to +- 100%
#         left_percentage = max(min(left_percentage, 100.0), -100.0)
#         right_percentage = max(min(right_percentage, 100.0), -100.0)
#         #
#         # Add in a governor to cap forward motion when we're about
#         # to collide with something (but still backwards motion)
#         governor = 1.0
#         if self.distance < self.tooclose:
#             governor = 0.0
#         elif self.distance < self.close:
#             governor = (self.distance - self.tooclose) / \
#                 (self.close - self.tooclose)
#         if right_percentage > 0:
#             right_percentage *= governor
#         if left_percentage > 0:
#             left_percentage *= governor
#         #
#         self._rightWheel.run(right_percentage)
#         self._leftWheel.run(left_percentage)


# def main(args=None):

#     rclpy.init(args=args)

#     little_bot = Robot('little_bot', pinRightFwd=10, pinRightRev=9,
#                 pinLeftFwd=8, pinLeftRev=7, left_max_rpm=195,
#                 right_max_rpm=202)

#     # enable the keyboard controller:
#     # ros2 run teleop_twist_keyboard teleop_twist_keyboard
#     #
#     print("Spinning...")
#     rclpy.spin(little_bot)
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()