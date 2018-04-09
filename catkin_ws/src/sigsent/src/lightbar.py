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

        self.flash_rate = rospy.Rate(10)
        self.flash_count = 6
        self.current_state = 0        

    def callback(self, msg):
        if msg.data == 0:
            self.pi.write(self.light_pin, 0)
            self.current_state = 0
        elif msg.data == 1:
            self.pi.write(self.light_pin, 1)
            self.current_state = 1
        else:
            for i in range(self.flash_count):
                self.current_state ^= 1
                self.pi.write(self.light_pin, self.current_state)
                
                self.flash_rate.sleep()

if __name__ == '__main__':
    try:
        lb = Lightbar()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
