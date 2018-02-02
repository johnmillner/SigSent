#!/usr/bin/python
import rospy

from sensor_msgs.msg import NavSatFix

def callback(data):
    
    #set up publisher
    pub = rospy.Publisher('gps_goal_fix', NavSatFix, queue_size=10)
    rate = rospy.Rate(10)
    
    #translate
    pub.publish(data.Pose)
    rate.sleep()
    
    
def translator():

    #start up node
    rospy.init_node('translator',anonymous=True)

    
    
    #set up subscriber
    rospy.Subscriber('fix', NavSatFix, callback)
    
    #enter loop 
    rospy.spin()
        
        


if __name__ == '__main__':
    try:
        translator()
    except rospy.ROSInterruptException:
        pass
