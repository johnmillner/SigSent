#!/usr/bin/env python

# John Millner
# sigsent 2018
# enables GPIO pin that controls I2C bus for the IMU

import rospy
import pigpio

def enableIMU():
    #on current PiHat for SigSent, enable pin is on GPIO 23
    gpio_pin = 23 
    
    #enable node
    rospy.init_node('enableIMU')
    
    #init pigpio
    p = pigpio.pi()
    
    #set GPIO to HIGH, enabling i2c bus
    p.write(gpio_pin, 1)
    
    #happy little message to let us know whats going on
    rospy.loginfo("IMU enabled on pin " + gpio_pin )

if __name__ == '__main__':
    try:
        enableIMU()
    except rospy.ROSInterruptException:
        rospy.logerror("Could not enable IMU pin on GPIO pin " + gpio_pin)
        pass
