import rospy
import pigpio
from std_msgs.msg import Int8\

# BCM17
light_pin = 11 

pi = pigpio.pi()
pi.set_mode(light_pin, pigpio.OUTPUT)

flash_rate = rospy.Rate(10)
flash_count = 6
current_state = 0

def callback(msg):
    if msg.data == 0:
        pi.write(light_pin, 0)
        current_state = 0
    elif msg.data == 1:
        pi.write(light_pin, 1)
        current_state = 1
    else:
        for i in range(flash_count):
            current_state ^= 1
            pi.write(light_pin, current_state)
            
            flash_rate.sleep()

    
if __name__ == '__main__':
    try:
        rospy.init_node('lightbar')
        rospy.Subscriber('blind', Int8, callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
