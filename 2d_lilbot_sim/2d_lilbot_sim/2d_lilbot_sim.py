#!/usr/bin/env python
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
import math

# NOTE: Change this filename to match your computers directory for the file
sprite_file = "src/little_bot/2d_lilbot_sim/images/lilbot_sprite.png"

dt = 0.0

class World:
    def __init__(self, world_dimentions):
        self.background_color = (30,30,30)
        self.black = (0,0,0)
        self.white = (255,255,255)
        self.green = (0,255,0)
        self.blue = (0,0,255)
        self.red = (255,0,0)
        self.yellow = (255,255,0)

        self.height = world_dimentions[0]
        self.width = world_dimentions[1]

        pygame.display.set_caption("Little Bot")
        self.map = pygame.display.set_mode((self.width, self.height))
    
        self.font = pygame.font.Font('freesansbold.ttf', 30)
        self.text = self.font.render('default', True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = ( world_dimentions[1]-400, world_dimentions[0]-20)
    
        self.trail_set = []

    def write_info(self, left_velocity, right_velocity, theta):
        txt = f"VL = {left_velocity} | VR = {right_velocity} | THETA = {int(math.degrees(theta))}"
        self.text = self.font.render(txt, True, self.white, self.black)
        self.map.blit(self.text, self.textRect)

    def trail(self, position):
        for i in range(0, len(self.trail_set)-1):
            pygame.draw.line(self.map, self.yellow, (self.trail_set[i][0], self.trail_set[i][1]), (self.trail_set[i+1][0], self.trail_set[i+1][1]))
        if self.trail_set.__sizeof__()>20000:
            self.trail_set.pop(0)
        self.trail_set.append(position)

class Robot:
    def __init__(self, start_position, robot_image, width):
        self.meterToPixel = 3779.52
        
        self.width = width

        self.x = start_position[0]
        self.y = start_position[1]
        self.theta = 0.0

        self.velocityLeft = 0.01 * self.meterToPixel # [m/s]
        self.velocityRight = 0.01 * self.meterToPixel # [m/s]

        self.maxSpeed = 0.02 * self.meterToPixel
        self.minSpeed = 0.02 * self.meterToPixel

        self.image = pygame.image.load(robot_image)
        self.image = pygame.transform.scale(self.image, (100, 100))
        self.rotatedImage = self.image
        self.rectangle = self.rotatedImage.get_rect(center = (self.x, self.y))

    def draw(self, map):
        map.blit(self.rotatedImage, self.rectangle)

    def move(self, event = None):
        if event is not None:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    self.velocityLeft += 0.001 * self.meterToPixel
                elif event.key == pygame.K_a:
                    self.velocityLeft -= 0.001 * self.meterToPixel
                elif event.key == pygame.K_e:
                    self.velocityRight += 0.001 * self.meterToPixel
                elif event.key == pygame.K_d:
                    self.velocityRight -= 0.001 * self.meterToPixel
        self.x += ((self.velocityLeft + self.velocityRight)/2) * math.cos(self.theta) * dt
        self.y -= ((self.velocityLeft + self.velocityRight)/2) * math.sin(self.theta) * dt  
        self.theta += ((self.velocityRight - self.velocityLeft) / self.width) * dt

        self.rotatedImage = pygame.transform.rotozoom(self.image, math.degrees(self.theta), 1)
        self.rectangle = self.rotatedImage.get_rect(center=(self.x, self.y))         


pygame.init()
start_position = (200.0,200.0)
world_dimentions = (600,1200)
RUNNING = True
world = World(world_dimentions)
robot = Robot(start_position, sprite_file, 0.01 * 3779.52)


lasttime = pygame.time.get_ticks()
while RUNNING:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            RUNNING = False
        robot.move(event)
    dt = (pygame.time.get_ticks() - lasttime) / 1000.0
    lasttime = pygame.time.get_ticks()
    pygame.display.update()
    world.map.fill(world.background_color)
    robot.move()
    robot.draw(world.map)
    world.trail((robot.x, robot.y))
    world.write_info(int(robot.velocityLeft), int(robot.velocityRight), int(robot.theta))


def main():
    print("---Starting Little Bot 2D Simulation---")
    


if __name__ == '__main__':
    main()
