#!/usr/bin/env python
'''
    Republishes ultrasonic sensors data from Arduino to normal Range messages
    Rebublished bump sensors data from Arduino to fixed distance ranger message Range
'''

import rospy
from sensor_msgs.msg import Range
from samana_msgs.msg import Int16Array
from samana_msgs.msg import Bump
import math

range_sonar_msg = Range()
range_bump_msg = Range()
sonar_max_ranges = [0.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.5, 0.5]  # NOTE: Found experimentally
# sonar_max_ranges = [7.7, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.7]  # NOTE: Found experimentally
bump_counter = 0
sonar_pub = rospy.Publisher("range_sonar", Range, queue_size=10)
bump_pub1 = rospy.Publisher("range_bump1", Range, queue_size=10)
bump_pub2 = rospy.Publisher("range_bump2", Range, queue_size=10)
# ------------------------------- FUNTIONS -------------------------------


def sonar_callback(range_data):
    '''
        Republished received distances array to Range messages
    '''
    global sonar_pub, range_sonar_msg, sonar_max_ranges
    for i in range(10):
        range_sonar_msg.header.frame_id = "ultrasonic_%d" % (i+1)
        range_sonar_msg.header.stamp = rospy.Time.now()
        range_sonar_msg.range = range_data.data[i] / 1000.0  # Converted to meters
        range_sonar_msg.max_range = sonar_max_ranges[i]  # Set each range different

        sonar_pub.publish(range_sonar_msg)


def bump_callback(bump_data):
    '''
        Republishes received bitmask to Range messages of fixed distance ranger
    '''
    global bump_pub1, bump_pub2, range_bump_msg, bump_counter

    # bump_counter += 1
    # if bump_counter < 2:
    #     return
    # else:
    #     bump_counter = 0

    BUMP_SENSORS_COUNT = 15
    # Static variable equivalent bin_data_old
    try:
        bump_callback.bin_data_old  # Checks if variable exists
    except AttributeError:
        bump_callback.bin_data_old = '0' * BUMP_SENSORS_COUNT
        # print("DEFINED")

    # Converting int16 to string of len 15
    bin_data = '{data:0{width}b}'.format(data=bump_data.bump_bits, width=BUMP_SENSORS_COUNT)[::-1]
    # print(bin_data)

    for i in range(BUMP_SENSORS_COUNT):
        # Simple filter. Consider trigger if triggered for 2 frame in a row
        r = float("inf")  # +inf - no detection
        if bin_data[i] == '1' and bump_callback.bin_data_old[i] == '1':
            r = float("-inf")  # -inf - detection bumped
        
        # if bin_data[i] == '1':
        #     r = float("-inf")  # -inf - detection bumped
        
        # Publish bump as fixed Range
        range_bump_msg.header.frame_id = "bump_%d" % (i+1)
        range_bump_msg.header.stamp = rospy.Time.now()
        range_bump_msg.range = r

        # There's limit of 10 Range messages to display
        # https://answers.ros.org/question/11701/rviz-message-filter-queue-size/
        if i <= 7:
            bump_pub1.publish(range_bump_msg)
        else:
            bump_pub2.publish(range_bump_msg)

    # Remember previous bin_data
    bump_callback.bin_data_old = bin_data


# ------------------------------- MAIN -------------------------------

if __name__ == "__main__":
    rospy.init_node("range_pub")

    # Setup static sonar range message data
    range_sonar_msg.radiation_type = range_sonar_msg.ULTRASOUND
    range_sonar_msg.field_of_view = math.radians(15)
    range_sonar_msg.min_range = 0.02
    # NOTE: Important if max_range will be set larger than real max range
    # Map builder will pickup unexisting obstacles at the end of sonar cone
    # It is set for each sensor differently in callback function
    # range_sonar_msg.max_range = 7.0

    # Setup static bump range message data
    range_bump_msg.radiation_type = range_bump_msg.INFRARED
    range_bump_msg.field_of_view = math.radians(45)  # It's because bump sensor can bend
    range_bump_msg.min_range = 0.05
    range_bump_msg.max_range = range_bump_msg.min_range  # Because fixed distance

    # Subscribe to topics
    rospy.Subscriber("sonar", Int16Array, sonar_callback)
    rospy.Subscriber("bump", Bump, bump_callback)

    # Don't exit
    rospy.spin()
