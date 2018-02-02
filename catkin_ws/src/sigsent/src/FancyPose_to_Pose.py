#!/usr/bin/python
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseWithCovariance, PoseStamped 

def callback(data):
    
    #set up publisher
    pub = rospy.Publisher('local_xy_origin', PoseStamped, queue_size=10)
    rate = rospy.Rate(100)
    
    #translate
    msg = PoseStamped()
    msg.pose = data.pose.pose
    msg.header = data.header
    pub.publish(msg)
    rate.sleep()
    
    
def xtra_to_pose():

    #start up node
    rospy.init_node('xtra_to_pose',anonymous=True)

    
    
    #set up subscriber
    rospy.Subscriber('robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, callback)
    
    #enter loop 
    rospy.spin()
        
        


if __name__ == '__main__':
    try:
        xtra_to_pose()
    except rospy.ROSInterruptException:
        pass
