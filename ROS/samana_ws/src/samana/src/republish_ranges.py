#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
import math

range_val = 1.0

def publish_range(index):
    global range_val

    
    msg = Range()
    msg.header.frame_id = "ultrasonic_%d" % index
    print(msg.header.frame_id)
    msg.header.stamp = rospy.Time.now()
    msg.radiation_type = 0
    msg.field_of_view = math.radians(15)
    msg.min_range = 0.02
    msg.max_range = 5.0
    msg.range = range_val

    pub = rospy.Publisher("range_data", Range, queue_size=10)
    pub.publish(msg)

    range_val += 0.01
    if range_val >= 1:
        range_val = 0.2


if __name__ == "__main__":
    rospy.init_node("range_pub")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            # for i in range(1, 11):
            for i in range(1, 11):
                publish_range(i)
        except Exception as e:
            print(e)
        rate.sleep()