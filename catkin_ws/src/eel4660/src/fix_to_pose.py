#!/usr/bin/python
import rospy

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

def callback(data):
    
    #set up publisher
    pub = rospy.Publisher('local_xy_origin', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)
    
    #translate
    msg = Pose()
    msg.header = data.header
    msg.pose.point.x = data.latitude
    msg.pose.point.y = data.longitude
    msg.pose.point.z = data.altitude
    
    #publish message
    pub.publish(msg)
    rate.sleep()

    
    
def translator():

    #start up node
    rospy.init_node('fix_to_pose',anonymous=True)

    
    
    #set up subscriber
    rospy.Subscriber('fix', NavSatFix, callback)
    
    #enter loop 
    rospy.spin()
        
        


if __name__ == '__main__':
    try:
        translator()
    except rospy.ROSInterruptException:
        pass
