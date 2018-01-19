# SigSent
This project is the Senior Design group 11 for the University of Central Florida College of Electrical and Computer Engineering

Team:
	Josh Franco	
	John Millner
	Jeff Strange
	Richie Wales
	
Sigsent acts as an autonomous patrol robot capable of detecting intruders on a designated path

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
