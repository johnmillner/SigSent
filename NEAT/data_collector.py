import rospy
import json
from std_msgs.msg import Int8
from sensor_msgs.msg import Imu

class DataCollector:
    def __init__(self, filename):
        with open(filename, 'r') as imu_fp:
            self.data = json.load(imu_fp)

        # These values are free to change
        self.collection_rate = rospy.Rate(10)
        self.collection_time = rospy.Duration.from_sec(100)
        
        self.collector_sub = rospy.Subscriber('neat/data_collection', Int8, self.data_collection, queue_size=10)
        self.imu_sub = rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, self.imu_cb, queue_size=10)
        self.imu_data = None
        
        self.lock = False

    def imu_cb(self, data):
        self.imu_data = (data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z, 
                         data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z)

    def data_collection(self, msg):
        if self.lock:
            return
        
        self.lock = True

        # Mode: 0 = smooth wheels, 1 = rough wheels, 2 = smooth walking, 3 = rough walking
        mode = msg.data

        end_time = rospy.Time.now() + collection_time
        while rospy.Time.now() < end_time:
            self.data[self.imu_data] = mode
            self.collection_rate.sleep()

        with open(filename, 'w') as imu_fp:
            json.dump(dc.data, imu_fp)


        self.lock = False

if __name__ == '__main__':
    dc = None
    filename = 'imu.json'
    
    try:
        dc = DataCollector(filename)
        rospy.spin()
    except rospy.ROSInterruptException:
        # Write labeled data to the json file when this stops running
        with open(filename, 'w') as imu_fp:
            json.dump(dc.data, imu_fp)