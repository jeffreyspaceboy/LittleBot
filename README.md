# Packages
- A Package is a collection of programs that are meant to control certain systems, sensors, or other devices on a robot.
- Packages should be created into this directory. This will allow the entire little_bot repository to be cloned, and all necesary packages will then be included.

## Resources
Understanding Nodes: https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Nodes.html  

## Details
Author: Jeffrey Fisher II  
ROS Version: ROS2 Galactic Geochelone
Last Edit: 2022/3/23 


## Webots Simulation
http://docs.ros.org/en/foxy/Tutorials/Simulators/Webots/Setting-up-a-Robot-Simulation-Webots.html
https://github.com/cyberbotics/webots_ros2/wiki/Tutorial-Create-Webots-Robot


## Useful Commands 

rm -r build install log

colcon build

source /opt/ros/galactic/setup.bash

source install/setup.bash


ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.00}}'

<!-- FOR ADDING GPIO TO CURRENT USER: (https://raspberrypi.stackexchange.com/questions/40105/access-gpio-pins-without-root-no-access-to-dev-mem-try-running-as-root)

sudo groupadd gpio
sudo usermod -a -G gpio user_name
sudo grep gpio /etc/group
sudo chown root.gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem -->




lilbot
	Overall launch and control of robot
lilbot_drivetrain
	Controls physical hardware
	Nodes:
		Motor_Wheel_Left
			Encoder_Wheel_Left
		Right_Wheel_Motor
			Right_Wheel_En
lilbot_interfaces
	Sets up all custom interfaces
lilbot_webots_sim
	Controls virtual version of the robot
