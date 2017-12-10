#!/usr/bin/env python

import serial
import rospy
import roslib

from std_msgs.msg import String, Header
from geometry_msgs import Quaternion, Vector3,  
from sensor_msgs.msg import Imu

def imu():
    IMUport     = '/dev/ttyACM0'
    IMUbaud     = 115200
    IMUparity   = serial.PARITY_NONE
    IMUstopBits = serial.STOPBITS_ONE
    IMUbyteSize = serial.EIGHTBITS
    IMUtimeout  = 1

    pub = rospy.Publisher('imu', Imu, queue_size=10)
    rospy.init_node('imu', anonymous=True)
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        #open port
        ser = serial.Serial(IMUport, IMUbaud, timeout = IMUtimeout)
        #diagnostic
        print('port is open!')

        #get raw data from serial feed
        line = ser.readline()
        
        #split it all up
        data = line.split(", ")
        
        #assign to proper values message values
        imuMsg = Imu()  #sets up ImuMessage with the right format fomr Imu()
        
        #below is taken from KristofRobot: https://github.com/KristofRobot/razor_imu_9dof/blob/indigo-devel/nodes/imu_node.py
        
        # Orientation covariance estimation:
        # Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
        # Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
        # Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
        # cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
        # i.e. variance in yaw: 0.0025
        # Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
        # static roll/pitch error of 0.8%, owing to gravity orientation sensing
        # error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
        # so set all covariances the same.
        imuMsg.orientation_covariance = [
        0.0025 , 0 , 0,
        0, 0.0025, 0,
        0, 0, 0.0025
        ]

        # Angular velocity covariance estimation:
        # Observed gyro noise: 4 counts => 0.28 degrees/sec
        # nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
        # Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
        imuMsg.angular_velocity_covariance = [
        0.02, 0 , 0,
        0 , 0.02, 0,
        0 , 0 , 0.02
        ]

        # linear acceleration covariance estimation:
        # observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
        # nonliniarity spec: 0.5% of full scale => 0.2m/s^2
        # Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
        imuMsg.linear_acceleration_covariance = [
        0.04 , 0 , 0,
        0 , 0.04, 0,
        0 , 0 , 0.04
        ]
        
        
        #convert to proper units
        
        #send out the data
        rospy.loginfo(imuData)
        pub.publish(imuData)
        rate.sleep();
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
