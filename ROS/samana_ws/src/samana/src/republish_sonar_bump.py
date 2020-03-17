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

SONAR_COUNT = 10
sonar_hist = [[], ]*SONAR_COUNT

range_sonar_msg = Range()
range_bump_msg = Range()
sonar_max_ranges = [0.4] + [0.5,]*8 + [0.4]  # NOTE: Found experimentally
# sonar_max_ranges = [0.5, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.5]  # NOTE: Found experimentally
# sonar_max_ranges = [7.7, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.7]  # For testing
sonar_pub = rospy.Publisher("range_sonar", Range, queue_size=10)
bump_pub1 = rospy.Publisher("range_bump1", Range, queue_size=10)
bump_pub2 = rospy.Publisher("range_bump2", Range, queue_size=10)
# ------------------------------- FUNTIONS -------------------------------


def sonar_callback(range_data):
    '''
        Republished received distances array to Range messages
    '''
    global sonar_pub, range_sonar_msg, sonar_max_ranges, sonar_hist
    global SONAR_COUNT
    pub1 = rospy.Publisher("sonar1", Range, queue_size=5)  # TODO: delete
    for i in range(SONAR_COUNT):
        # Setup message info
        range_sonar_msg.header.frame_id = "ultrasonic_%d" % (i+1)
        range_sonar_msg.header.stamp = rospy.Time.now()
        range_sonar_msg.max_range = sonar_max_ranges[i]  # Set each max range different
        dist = range_data.data[i] / 1000.0  # Distance in meters
        range_sonar_msg.range = dist
        # If distance more than max set it to max. (For map clearing) doesn't work
        # if dist > sonar_max_ranges[i] and sonar_max_ranges[i] - dist <= 0.04:
        #     range_sonar_msg.range = sonar_max_ranges[i]

        # Filter range data

        # Keep short history of ranges
        HIST_COUNT = 2  # NOTE: tuning range filter. Sonar range history count. 2 Looks good
        sonar_hist[i].append(dist)
        if len(sonar_hist[i]) > HIST_COUNT:
            sonar_hist[i].pop(0)

        # Outlier filter: all values is history must be similar to the last one
        # NOTE: tuning range filter. Threshold in meters.
        # All history values must be is this range compare to latest one
        RANGE_THRESH = 0.15
        outlier = False
        for sh in sonar_hist[i]:
            # If atleast one reading isn't in range don't send the message
            if abs(dist - sh) > RANGE_THRESH:
                outlier = True
                break
        
        if outlier is False:
            # Reading passed outlier filter
            sonar_pub.publish(range_sonar_msg)
        
        range_sonar_msg.max_range = 10.0
        pub1.publish(range_sonar_msg)  # TODO: delete pub to debug topic


def bump_callback(bump_data):
    '''
        Republishes received bitmask to Range messages of fixed distance ranger
    '''
    global bump_pub1, bump_pub2, range_bump_msg

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
