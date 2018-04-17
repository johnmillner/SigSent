#!/usr/bin/env python

# Richie Wales/John Millner
# sigsent 2018
# enables GPIO pin that controls I2C bus for the IMU

import rospy
import pigpio

def enableFG():
    #on current PiHat for SigSent, enable pin is on GPIO 4
    gpio_pin = 4
    
    #enable node
    rospy.init_node('enableFG')
    
    #init pigpio
    p = pigpio.pi()
    
    #set GPIO to HIGH, enabling i2c bus
    p.write(gpio_pin, 1)
    
    #happy little message to let us know whats going on
    rospy.loginfo("Fuel Gauge enabled on pin {}".format( gpio_pin ))

if __name__ == '__main__':
    try:
        enableFG()
    except rospy.ROSInterruptException:
        rospy.logerror("Could not enable Fuel Gauge pin on GPIO pin {}".format( gpio_pin) )
        pass
