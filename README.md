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

source install/setup.bash




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
