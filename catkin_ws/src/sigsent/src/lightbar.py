#!/usr/bin/python
import rospy
import pigpio
from std_msgs.msg import Int8

class Lightbar:
    def __init__(self):
        rospy.init_node('lightbar')
        self.blind_sub = rospy.Subscriber('blind', Int8, self.callback)

        # BCM17
        self.light_pin = 17

        self.pi = pigpio.pi()
        self.pi.set_mode(self.light_pin, pigpio.OUTPUT)
	self.strobe = False
        self.flash_rate = rospy.Rate(50)
        self.current_state = 0        

    def callback(self, msg):
        if msg.data == 0:
            self.strobe = False
            self.pi.write(self.light_pin, 0)
            self.current_state = 0
        elif msg.data == 1:
	    self.strobe = False
            self.pi.write(self.light_pin, 1)
            self.current_state = 1
        else:
	    self.strobe = True
            self.current_state ^= 1
                

if __name__ == '__main__':
    try:
        lb = Lightbar()
	while not rospy.is_shutdown():
            while lb.strobe == True:
	    	lb.pi.write(lb.light_pin, lb.current_state)
		lb.current_state ^= 1
	    	lb.flash_rate.sleep()
    except rospy.ROSInterruptException:
        pass