#!/usr/bin/env python

import rospy
import serial

from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

def imu_parser():

    #set up ports and stuff
    port = '/dev/ttyACM1'
    baud = 115200
    timeout = 1
    rospy.loginfo("starting imu node for sparfun razor imu 14001")
    
    #constants
    g_to_mSS = 9.80665
    degS_to_radS = 0.0174533
    
    #open port
    #TODO - ADD SAFETY TO PORT OPENING
    rospy.loginfo("opening serial port at port: ", port, " and at baud rate of ", baud)
    s = serial.Serial(port)
    s.baudrate = baud
    s.timeout = timeout
    
    rospy.loginfo("port has been opened!")
    
    #node/publisher setup
    pub = rospy.Publisher('imu', Imu, queue_size=10)
    rospy.init_node('razor_imu_14001', anonymous=True)
    rate = rospy.Rate(10)
    
    i = 0   #set counter for trying to read data
    
    rospy.loginfo("sleeping for 5 seconds lol")
    rospy.sleep(5)
    
    #enter loop 
    while not rospy.is_shutdown():
        
        #create message
        msg = Imu()

        #read a line
        try:
            line = s.readline()
        except serial.SerialException:
            i += 1
            rospy.logwarn("not getting data from imu, attempt: %i of 100", i)
            if i > 10:
                s.close()
                rospy.logfatal("can't connect to IMU :( ")
            continue
            
        if not line or i >100:
            i+=1
            rospy.logwarn("not getting data from imu, attempt: %i of 100", i)
            continue
                
        #parse the line
        data = line.split(", ")
        
        #set header data
        #TODO - put imu into proper frame_id
        msg.header.seq = int(data[0])
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link" #make into actually correct frame id
        
        #set data fields
        q = quaternion_from_euler(float(data[7]), float(data[8]), float(data[9]))
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        
        msg.angular_velocity.x = float(data[4]) * degS_to_radS
        msg.angular_velocity.y = float(data[5]) * degS_to_radS
        msg.angular_velocity.z = float(data[6]) * degS_to_radS
        
        msg.linear_acceleration.x = float(data[1]) * g_to_mSS
        msg.linear_acceleration.y = float(data[2]) * g_to_mSS
        msg.linear_acceleration.z = float(data[3]) * g_to_mSS
        
        #these values taken from kristofRobot: https://github.com/KristofRobot/razor_imu_9dof/blob/indigo-devel/nodes/imu_node.py
        msg.orientation_covariance = [
        0.0025 , 0 , 0,
        0, 0.0025, 0,
        0, 0, 0.0025
        ]

        msg.angular_velocity_covariance = [
        0.02, 0 , 0,
        0 , 0.02, 0,
        0 , 0 , 0.02
        ]

        msg.linear_acceleration_covariance = [
        0.04 , 0 , 0,
        0 , 0.04, 0,
        0 , 0 , 0.04
        ]
        
        #publish message
        pub.publish(msg)
        rate.sleep()
        
    #clean up
    s.close() 
    rospy.loginfo("closed port")


if __name__ == '__main__':
    try:
        imu_parser()
    except rospy.ROSInterruptException:
        pass
