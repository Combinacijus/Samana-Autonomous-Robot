#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from samana_msgs.msg import Int16Array
from samana_msgs.msg import Bump
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math

range_msg = Range()
sonar_counter = 0
bump_counter = 0

# ------------------------------- FUNTIONS -------------------------------


def sonar_callback(range_data):
    global sonar_counter

    sonar_counter += 1 # Skip some callbacks
    if sonar_counter > 2:
        sonar_counter = 0
    else:
        return

    for i in range(10):
        publish_range(i+1, range_data.data[i] / 1000.0)  # Convert to meters


def publish_range(index, range):
    global range_msg
    range_msg.header.frame_id = "ultrasonic_%d" % index
    range_msg.header.stamp = rospy.Time.now()
    range_msg.range = range

    pub = rospy.Publisher("range_data_viz", Range, queue_size=10)
    pub.publish(range_msg)


def bump_callback(bump_data):
    
    # global bump_counter
    # bump_counter += 1 # Skip some callbacks
    # if bump_counter > 12:
    #     bump_counter = 0
    # else:
    #     return

    BUMP_SENSORS_COUNT = 15
    # Static variable exivalent bin_data_old
    try:
        bump_callback.bin_data_old  # Checks if variable exists
    except AttributeError:
        bump_callback.bin_data_old = '0' * BUMP_SENSORS_COUNT
        # print("DEFINED")

    # Converting int16 to string of len 15
    bin_data = '{data:0{width}b}'.format(
        data=bump_data.bump_bits, width=BUMP_SENSORS_COUNT)[::-1]
    # print(bin_data)

    marker_array = MarkerArray()
    for i in range(BUMP_SENSORS_COUNT):
        # Static marker data
        marker = Marker()
        # marker.header.stamp = bump_data.header.stamp
        marker.header.stamp = rospy.Time.now()
        marker.ns = "bump"  # Namespace
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.10
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 0.9
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.lifetime = rospy.Duration(0)

        # Dynamic marker data
        marker.header.frame_id = "bump_%d" % (i+1)  # Which tf from URDF to use
        marker.id = i + 1  # Must be different for all markers (if same overrides it)

        # On of off with simple filter (2 frames on = on)
        # if (bin_data[i] == '1' and bump_callback.bin_data_old[i] == '1'):
        if bin_data[i] == '1':
            marker.action = marker.ADD
        else:
            marker.action = marker.DELETE
        

        # Append to markers array
        marker_array.markers.append(marker)

    # Remember previous bin_data
    bump_callback.bin_data_old = bin_data

    # Publish marker array
    maker_array_pub = rospy.Publisher(
        "visualization_marker_array", MarkerArray, queue_size=100)
    maker_array_pub.publish(marker_array)


# ------------------------------- MAIN -------------------------------

if __name__ == "__main__":
    rospy.init_node("range_pub")

    # Setup static range message data
    range_msg.radiation_type = 0
    range_msg.field_of_view = math.radians(15)
    range_msg.min_range = 0.01
    range_msg.max_range = 7.0

    # Subscribe to topics
    rospy.Subscriber("sonar", Int16Array, sonar_callback)
    rospy.Subscriber("bump", Bump, bump_callback)

    # Don't exit
    rospy.spin()
