#!/usr/bin/python
import rospy
import roslib
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

roslib.load_manifest('sigsent')
from sigsent.msg import GPSList
    
    
gpsList = list()
status = 10
first_time = True
    
def checker(data):
    global first_time
    rospy.loginfo("gpsGoalManager: checking satillite connection")
    
    #check for satilite connection, if none, wait one second and check again
    if data.status.status < 0 and first_time:
        return
        
    rospy.loginfo("gpsGoalManager: Valid GPS fix found, setting up gpsGoal node")
    
    #with valid satillite connection, set local_xy_origin
    #set up publisher
    localXYorigin = rospy.Publisher('local_xy_origin', PoseStamped, queue_size=10)
    
    msg = PoseStamped()
    msg.header = data.header
    
    msg.pose.position.x = data.latitude
    msg.pose.position.y = data.longitude
    msg.pose.position.z = data.altitude
    
    #orientation is currently zerod out, but maybe integrate imu later?
    msg.pose.orientation.x = 0
    msg.pose.orientation.y = 0
    msg.pose.orientation.z = 0
    msg.pose.orientation.w = 0
    
    first_time = False
    
    #publish message once
    localXYorigin.publish( msg )
    
    initializer.unregister() 
    
    operation()
    
           
#updates the list of gps points
def newMessage( data ):
    rospy.loginfo("gpsGoalManager: updated list of GPS goals!")
    global gpsList 
    gpsList = data.goals
    
def move_baseStatus( data ):
    global status 
    if( len( data.status_list ) > 0):
        status = data.status_list[0].status
            
def operation():
    rospy.loginfo("gpsGoalManager: Entering Operation Mode")
    rospy.Subscriber('gps_goals_list', GPSList, newMessage)
    rospy.Subscriber("move_base/status", GoalStatusArray, move_baseStatus )
    gpsGoal = rospy.Publisher('gps_goal_fix', NavSatFix, queue_size=10)
    confirmedGoal = rospy.Publisher('confirmed_goal', NavSatFix, queue_size=10)
    
    while not rospy.is_shutdown():
        
        #loop through list of GPS goals as they come
        #whenever the gps goals are updated, operation() is interupted and the list is updated
        for gpsPoint in gpsList:
            rospy.loginfo("gpsGoalManager: going to new point")
            #publish the goal to gps_goal
            gpsGoal.publish( gpsPoint )
            rospy.loginfo('List len: {}'.format(len(gpsList)))
            
            
            #wait for plan to be accepted and pathed to
            while status <= 1:
                pass      
               
            if status < 3 :
                rospy.logwarn('Plan aborted')
                continue
            
            rospy.loginfo("gpsGoalManager: Reached goal!")
            #we've made it to our goal, lets wait a little bit
            rospy.Rate(5).sleep()
            
            #set up confirmation message
            msg = gpsPoint
            msg.header.stamp = rospy.Time.now()
            
            #publish that junk
            confirmedGoal.publish( msg )
                

if __name__ == '__main__':
    try:
        #start up node
        rospy.init_node('gpsGoalManager')     
    
        #set up publisher
        gpsGoal = rospy.Publisher('gps_goal_fix', NavSatFix, queue_size=10)
           
        #check that GPS is operational and giving signals   
        initializer = rospy.Subscriber("fix", NavSatFix, checker) 
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
