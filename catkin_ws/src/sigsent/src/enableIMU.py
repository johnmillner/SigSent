#!/usr/bin/env python

import rospy
import pigpio

def enableIMU():
    rospy.init_node('enableIMU')
    p = pigpio.pi()
    p.write(23, 1)
    rospy.loginfo("IMU enabled on pin 10")

if __name__ == '__main__':
    try:
        enableIMU()
    except rospy.ROSInterruptException:
        pass
