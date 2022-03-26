#!/usr/bin/env python

# NOTE: Change this filename to match your computers directory for the file
# sprite_file = "src/little_bot/lilbot_pygame/resource/images/lilbot_sprite.png"
sprite_file = "../resource/images/lilbot_sprite.png"

DT = 0.0

PIXELS_PER_METER = 100
SCREEN_WIDTH_M = 5.0
SCREEN_HEIGHT_M = 5.0

SCREEN_WIDTH_PX = int(SCREEN_WIDTH_M * PIXELS_PER_METER)
SCREEN_HEIGHT_PX = int(SCREEN_HEIGHT_M * PIXELS_PER_METER)


ROBOT_WIDTH_M = 0.1

ROBOT_START_X = 0.0
ROBOT_START_Y = 0.0

VEL_CHANGE_MULT = 0.1

ROBOT_START_VEL = 0.5


# COLORS:
BACKGROUND = (30,30,30)
BLACK = (0,0,0)
WHITE = (255,255,255)
GREEN = (0,255,0)
BLUE = (0,0,255)
RED = (255,0,0)
YELLOW = (255,255,0)


import sys
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformStamped
import tf_transformations


class World:
    def __init__(self, world_dimentions):
        self.height = world_dimentions[0]
        self.width = world_dimentions[1]

        pygame.display.set_caption("Little Bot")
        self.map = pygame.display.set_mode((self.width, self.height))
    
        self.font = pygame.font.Font('freesansbold.ttf', 30)
        self.text = self.font.render('default', True, WHITE, BLACK)
        self.textRect = self.text.get_rect()
        self.textRect.center = ( world_dimentions[1]-400, world_dimentions[0]-20)
    
        self.trail_set = []

    def write_info(self, left_velocity, right_velocity, theta):
        txt = f"VL = {left_velocity} | VR = {right_velocity} | THETA = {int(math.degrees(theta))}"
        self.text = self.font.render(txt, True, WHITE, BLACK)
        self.map.blit(self.text, self.textRect)

    def trail(self, position):
        for i in range(0, len(self.trail_set)-1):
            pygame.draw.line(self.map, YELLOW, (self.trail_set[i][0], self.trail_set[i][1]), (self.trail_set[i+1][0], self.trail_set[i+1][1]))
        if self.trail_set.__sizeof__()>20000:
            self.trail_set.pop(0)
        self.trail_set.append(position)

class Robot(Node):
    def __init__(self, start_position, robot_image, width):
        #ROS TF
        super().__init__('lilbot_pygame_node')
        self._tf_publisher = TransformBroadcaster(self)
        #ROS TF
        
        self.width = width

        self.x = start_position[0]
        self.y = start_position[1]
        self.theta = 0.0

        self.velocityLeft = ROBOT_START_VEL * PIXELS_PER_METER # [m/s]
        self.velocityRight = ROBOT_START_VEL * PIXELS_PER_METER # [m/s]

        self.maxSpeed = 2.0 * PIXELS_PER_METER
        self.minSpeed = 2.0 * PIXELS_PER_METER

        self.image = pygame.image.load(robot_image)
        self.image = pygame.transform.scale(self.image, (ROBOT_WIDTH_M * PIXELS_PER_METER, ROBOT_WIDTH_M * PIXELS_PER_METER))
        self.rotatedImage = self.image
        self.rectangle = self.rotatedImage.get_rect(center = (self.x, self.y))

    def draw(self, map):
        map.blit(self.rotatedImage, self.rectangle)

    def move(self, event = None):
        if event is not None:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    self.velocityLeft += VEL_CHANGE_MULT * PIXELS_PER_METER
                elif event.key == pygame.K_a:
                    self.velocityLeft -= VEL_CHANGE_MULT * PIXELS_PER_METER
                elif event.key == pygame.K_e:
                    self.velocityRight += VEL_CHANGE_MULT * PIXELS_PER_METER
                elif event.key == pygame.K_d:
                    self.velocityRight -= VEL_CHANGE_MULT * PIXELS_PER_METER
        self.x += ((self.velocityLeft + self.velocityRight)/2) * math.cos(self.theta) * DT
        self.y -= ((self.velocityLeft + self.velocityRight)/2) * math.sin(self.theta) * DT  
        self.theta += ((self.velocityRight - self.velocityLeft) / self.width) * DT

        self.rotatedImage = pygame.transform.rotozoom(self.image, math.degrees(self.theta), 1)
        self.rectangle = self.rotatedImage.get_rect(center=(self.x, self.y))  
        self.make_transforms() 

    def make_transforms(self):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'world'
        static_transformStamped.child_frame_id = 'body'
        static_transformStamped.transform.translation.x = float(self.x/ PIXELS_PER_METER)
        static_transformStamped.transform.translation.y = float(-self.y/ PIXELS_PER_METER)
        static_transformStamped.transform.translation.z = float(0)
        quat = tf_transformations.quaternion_from_euler(float(0), float(0), float(self.theta))
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self._tf_publisher.sendTransform(static_transformStamped)

rclpy.init()
pygame.init()
start_position = (ROBOT_START_X, ROBOT_START_Y)
world_dimentions = (SCREEN_HEIGHT_PX, SCREEN_WIDTH_PX)
RUNNING = True
world = World(world_dimentions)
robot = Robot(start_position, sprite_file, ROBOT_WIDTH_M * PIXELS_PER_METER)


lasttime = pygame.time.get_ticks()
while RUNNING:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            RUNNING = False
        robot.move(event)
    DT = (pygame.time.get_ticks() - lasttime) / 1000.0
    lasttime = pygame.time.get_ticks()
    pygame.display.update()
    world.map.fill(BACKGROUND)
    robot.move()
    robot.draw(world.map)
    world.trail((robot.x, robot.y))
    world.write_info(robot.velocityLeft/ PIXELS_PER_METER, robot.velocityRight/ PIXELS_PER_METER, int(robot.theta))


def main():
    print("---Starting Little Bot 2D Simulation---")

    # pass parameters and initialize node
    
    robot = Robot(start_position, sprite_file, ROBOT_WIDTH_M * PIXELS_PER_METER)
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
