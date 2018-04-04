# SigSent

This project is the Senior Design group 11 for the University of Central Florida College of Electrical and Computer Engineering

Team:
	Josh Franco	
	John Millner
	Jeff Strange
	Richie Wales

![Side View](right.png)

SigSent is an intelligent, multi-terrain hexapod robot built for robust, reliable security solutions. Using an evolved Neural Network using NeuroEvolution of Augementing Topologies (NEAT), SigSent determines when the road is getting rough and automatically transition from a four-legged drive mode into a walking mode using all six of its legs. This allows base operators to work at a higher level, simply providing GPS routes to the robot and leaving it to figure out the rest itself.

SigSent's gait is generated using a Genetic Algorithm to find the most optimal, stable walking motion under its given load.

SigSent features a computer vision module that runs a basic pedestrian detection algorithm so that base operators can be notified if an anomalous figure has appeared that needs addressing.

The base operator can also TeleOp with a provided joystick, listen to audio from the robot's integrated microphone, and also use a headset of their own to output audio from SigSent's own speakers for two-way communication. Diagnostic information regarding things like battery life, servo pose information, and CV/NEAT outputs are provided as well in the GUI application. The interface was created using PyQt5, Python bindings for Qt5.

![Top View](top.png)

This robot runs on a raspberry pi 3 uses the following sensors:
	Sparkfun Razor IMU 9DoF 14001
	Sparkfun Venus638FLPx
	RasPi Cam
	Hokuyo UTM-30lx LIDAR
	
To begin the robot first launch roscore on a basestation
then launch local.launch from the Raspberry Pi
followed by launching remote.launch from the basestation

ensure that environment variables such as ROS_MASTER_URI and ROS_PI are properly set and that the package is properly sourced on both computers
	source ~/SigSent/catkin_ws/devel/setup.bash
	ROS_MASTER_URI=http://192.168.1.110:11311
	ROS_IP=192.168.1.110

Sensors can be easily visualized using RVIZ
	rosrun rviz rviz
		once launcheed rviz should use the rviz configuration file found in the root directory of this github