#!/usr/bin/python
import rospy

from geometry_msgs.msg import PoseStamped

def callback(data):
    
    #set up publisher
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)
    
    #publish message
    pub.publish(data.target_pose)
    rate.sleep()

    
    
def translator():

    #start up node
    rospy.init_node('actionGoal_to_simpleGoal',anonymous=True)

    
    
    #set up subscriber
    rospy.Subscriber('move_base_msgs/MoveBaseAction', PoseStamped, callback)
    
    #enter loop 
    rospy.spin()
        
        


if __name__ == '__main__':
    try:
        translator()
    except rospy.ROSInterruptException:
        pass
