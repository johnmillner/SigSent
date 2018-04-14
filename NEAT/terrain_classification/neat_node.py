#!/usr/bin/env python

from __future__ import print_function
import os
import neat
import visualize
import rospy
import pickle
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu

class NEATNode:
    def __init__(self):
        # Setup the ROS sensor subscriptions
        self.sensor_data = {}
        
        rospy.init_node('neat', anonymous=True)
        
        self.mobility_pub = rospy.Publisher('mobility_type', Bool)
        imu_sub = rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, self.callback)
        
        print('Waiting for sensor data to come in...')
        while len(self.sensor_data) < 5:
            pass

        print('Ready')
        
        with open('best_net.ann', 'rb') as net_fp:
            self.ann = pickle.load(net_fp)

    # Dequeue oldest imu data, enqueue new one at head, keep window of size 5
    def callback(self, data):
        self.sensor_data.insert(0, (data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z, 
                         data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))

        if len(self.sensor_data) > 4:
            self.sensor_data.pop()
            inputs = []

            for imu_tup in self.sensor_data:
                inputs += list(imu_tup)

            output = True if self.ann.activate(inputs) >= 0.5 else False
            self.mobility_pub.publish(output)
        
if __name__ == '__main__':
    try:
        neatNode = NEATNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
