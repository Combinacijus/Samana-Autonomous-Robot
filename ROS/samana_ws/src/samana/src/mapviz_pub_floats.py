#!/usr/bin/env python
# Subscribes to many topics and publishes some of it's data to debug Float topic for mapviz to display

import rospy
from math import degrees
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from samana_msgs.msg import ImuSmall
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from random import random

imu_msg = Imu()

class FloatDebugger():
    def __init__(self):
        rospy.init_node("mapviz_pub_floats")

        rospy.Subscriber("imu_data", ImuSmall, self.imu_callback)
        rospy.Subscriber("odometry/global", Odometry, self.odom_global_callback)

        rospy.spin()
    
    def pub_float(self, topic, number):
        pub = rospy.Publisher(topic, Float32, queue_size=1)
        msg = Float32()
        msg.data = number
        pub.publish(msg)

    def imu_callback(self, data):
        if random() < 0.6: # To limit publish rate
            return

        # Publishes imu yaw corrected for mounting X to East = 0deg
        rpy = euler_from_quaternion([data.quaternion_x, data.quaternion_y, data.quaternion_z, data.quaternion_w])
        self.pub_float("debug/yaw_imu", degrees(rpy[2]) - 88) # NOTE: hardcoded transform
        self.pub_float("debug/pitch_imu", degrees(rpy[0])) # NOTE: roll axis due to mounting
    
    def odom_global_callback(self, data):
        if random() < 0.75: # To limit publish rate
            return

        # Publishes imu yaw corrected for mounting X to East = 0deg
        orient = data.pose.pose.orientation
        pos = data.pose.pose.position
        rpy = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        self.pub_float("debug/yaw_odom", degrees(rpy[2]))
        self.pub_float("debug/x", pos.x)
        self.pub_float("debug/y", pos.y)
    
if __name__ == '__main__':
    try:
        float_debugger = FloatDebugger()
    except rospy.ROSInterruptException:
        pass
