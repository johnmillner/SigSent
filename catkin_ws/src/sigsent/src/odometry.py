import math
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu
from math import sin, cos, pi

class SigSentOdom:
    def __init__(self):
        rospy.init_node('odom_publisher')
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.imu_sub = rospy.Subscriber('imu', Imu, self.imu_cb, queue_size=50)
        self.twist_velocity = None

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vth = 0.0
        self.old_time = rospy.Time.now()
        self.current_time = rospy.Time.now()

    def imu_cb(self, imu_msg):
        dt = (self.current_time - self.old_time).to_sec()

        velocity = (imu_msg.linear_acceleration.x * dt, imu_msg.linear_acceleration.y * dt, imu_msg.linear_acceleration.z * dt)

        self.vx = vel_msg.linear.x
        self.vy = vel_msg.linear.y
        self.vth = vel_msg.angular.z

    def update_odom(self):
        current_time = rospy.Time.now()
        last_time = rospy.Time.now()

        r = rospy.Rate(1.0)

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()

            delta_th = self.vth * dt
            th += delta_th

            delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
            delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt

            x += delta_x
            y += delta_y
            
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            self.odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

            """
            odom.pose.covariance[0] = 0.00001;  // x
            odom.pose.covariance[7] = 0.00001;  // y
            odom.pose.covariance[14] = 0.00001; // z
            odom.pose.covariance[21] = 1000000000000.0; // rot x
            odom.pose.covariance[28] = 1000000000000.0; // rot y
            odom.pose.covariance[35] = 0.001; // rot z
            odom.twist.covariance = odom.pose.covariance; // needed?

            """
            # publish the message
            self.odom_pub.publish(odom)

            last_time = current_time
            r.sleep()



if __name__ == '__main__':
    try:
        sso = SigSentOdom()
    except rospy.ROSInterruptException:
        pass
