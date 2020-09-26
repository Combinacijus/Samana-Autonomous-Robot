#!/usr/bin/env python
'''
    Republishes ultrasonic sensors data from Arduino to normal Range messages
    Rebublished bump sensors data from Arduino to fixed distance ranger Range message

    _____9___0_____
    | 8         1 |
    |7           2|
    |    SONAR    |
    |6           3|
    \             /
     \5         4/
      \_________/

    ___13_14__00_01___
    |12            02|
    |11            03|
    |      BUMP      |
    |10            04|
    |                |
    |09            05|
    \                /
     \08          06/
      \_____07_____/
'''

import rospy
from sensor_msgs.msg import Range
from samana_msgs.msg import Int16Array, Bump, ImuCalib
from std_msgs.msg import Bool
import math


def enable_front_sensors_cb(msg):
    global enable_front_sensors

    enable_front_sensors = msg.data
    rospy.loginfo("enable_front_sensors: {}".format(enable_front_sensors))


SONAR_COUNT = 10
sonar_hist = [[] for _ in range(SONAR_COUNT)]  # [[],] * SONAR_COUNT BUGFIX
temp_correction_coef = 1.0  # Updated in imu_calib_callback()
enable_front_sensors = True  # If false shortens front sonars range and disables from bump sensors

range_sonar_msg = Range()
range_bump_msg = Range()
#                    0    1     2    3    4 |  5    6    7     8    9
sonar_max_ranges = [0.2, 0.5, 0.55, 0.7, 0.5, 0.5, 0.7, 0.55, 0.5, 0.2]  # NOTE: tuning filter. Found experimentally
sonar_max_ranges_disabled = [0.03, 0.06, -1, -1, -1, -1, -1, -1, 0.06, 0.03]  # NOTE: tuning filter. Found experimentally
# sonar_max_ranges = [0.4] + [0.5] * 8 + [0.4]  # NOTE: tuning filter. Found experimentally

# Publishers
sonar_pub = rospy.Publisher("range_sonar", Range, queue_size=10)
# sonar_pub_all = rospy.Publisher("range_sonar_all", Range, queue_size=5)  # NOTE: debug
bump_pub1 = rospy.Publisher("range_bump1", Range, queue_size=10)
bump_pub2 = rospy.Publisher("range_bump2", Range, queue_size=10)

# Subscribers
rospy.Subscriber("params/enable_front_sensors", Bool, enable_front_sensors_cb)


# ------------------------------- FUNCTIONS -------------------------------

def sonar_callback(range_data):
    """
        Republishes filtered received distances array to Range messages
    """
    time_now = rospy.Time.now() - rospy.Time(0.04)  # 1 / 24Hz
    for i in range(SONAR_COUNT):
        # Setup message info
        range_sonar_msg.header.frame_id = "ultrasonic_%d" % (i + 1)
        range_sonar_msg.header.stamp = range_data.header.stamp
        range_sonar_msg.max_range = sonar_max_ranges[i]  # Set each max range different

        # Shorten front sensors range if published to do so
        if enable_front_sensors is False and sonar_max_ranges_disabled[i] != -1:
            range_sonar_msg.max_range = sonar_max_ranges_disabled[i]

        dist = range_data.data[i] / 1000.0 * temp_correction_coef  # Distance in meters corrected for temperature
        range_sonar_msg.range = dist

        # Outlier filter: values in history must be similar to the last one
        outlier = is_outlier(dist, i)

        if outlier is False:
            sonar_pub.publish(range_sonar_msg)

        # NOTE: Debug all sonar ranges
        # range_sonar_msg.max_range = 10.0
        # sonar_pub_all.publish(range_sonar_msg)


def is_outlier(reading, i=0):
    """
        Outliers filter for sonars. Considers max speed and recency of readings
        :param reading: new distance reading
        :param i: index of sonar
        :return: True if reading is outlier and should be discarded

        Readings come at 24Hz or 41.7ms
    """
    # NOTE: tuning range filter parameters
    HIST_COUNT = 6  # 6*41.7 = 250.2ms | 7*41.7 = 291.9ms
    MAX_OUTLIERS = int(HIST_COUNT // 3)  # For every 3 readings 1 outlier is allowed
    MAX_DELTA_PER_READING = 0.021  # 0.5m/s / 24Hz = 0.021m | 0.03 * 7 = 0.21
    NOISE_LEVEL = 0.0275  # Found by graphing sonar data

    # Store history of readings
    sonar_hist[i].append(reading)
    if len(sonar_hist[i]) > HIST_COUNT:
        sonar_hist[i].pop(0)

    # Find if it's outlier
    outlier = False
    outliers = 0
    for j, dist in enumerate(sonar_hist[i]):
        range_thresh = NOISE_LEVEL + MAX_DELTA_PER_READING * j
        if abs(reading - dist) > range_thresh:
            outliers += 1
            if outliers > MAX_OUTLIERS:
                outlier = True
                break

    return outlier


def imu_calib_callback(data):
    """
        For calculating correction factor for sonar sensor for different temperature
        NOTE: data from Arduino assumes 20degC
    """
    global temp_correction_coef

    JUNCTION_TEMP = 6  # Because it measures internal chip temp. With no air flow it's 8degC and ~5degC with airflow
    ASSUMED_SPEED_OF_SOUND = 343.42  # speed_of_sound = 331.3 + 0.606 * 20degC
    corrected_speed_of_sound = 331.3 + 0.606 * (data.temp - JUNCTION_TEMP)

    temp_correction_coef = corrected_speed_of_sound / ASSUMED_SPEED_OF_SOUND


def bump_callback(bump_data):
    """
        Republishes received bitmask to Range messages of fixed distance ranger
    """
    global range_bump_msg

    BUMP_SENSORS_COUNT = 15
    # Static variable equivalent bin_data_old
    try:
        bump_callback.bin_data_old  # Checks if variable exists
        bump_callback.bin_prev_posted  # Checks if variable exists
        bump_callback.last_bump_t  # Checks if variable exists
    except AttributeError:
        bump_callback.bin_data_old = '0' * BUMP_SENSORS_COUNT
        bump_callback.bin_prev_posted = [0, ] * BUMP_SENSORS_COUNT
        bump_callback.last_bump_t = [rospy.Time.now(), ] * BUMP_SENSORS_COUNT
        # print("DEFINED")

    # Converting int16 to string of len 15
    bin_data = '{data:0{width}b}'.format(data=bump_data.bump_bits, width=BUMP_SENSORS_COUNT)[::-1]
    # print(bin_data)

    for i in range(BUMP_SENSORS_COUNT):
        # Simple filter. Consider trigger if triggered for 2 frame in a row
        r = float("inf")  # +inf - no detection
        if bin_data[i] == '1' and bump_callback.bin_data_old[i] == '1':
            r = float("-inf")  # -inf - detection bumped

        if enable_front_sensors is False and i in [0, 1, 13, 14]:
            r = float("inf")  # Effectivelly disables 4 front bump sensors

        # If not the same range as previous or forced update
        if bump_callback.bin_prev_posted[i] != r or rospy.Time.now() - bump_callback.last_bump_t[i] > rospy.Duration(0.4):
            bump_callback.last_bump_t[i] = rospy.Time.now()

            # Publish bump as fixed Range
            range_bump_msg.header.frame_id = "bump_%d" % (i + 1)
            range_bump_msg.header.stamp = bump_data.header.stamp
            range_bump_msg.range = r

            # There's limit of 10 Range messages to display
            # https://answers.ros.org/question/11701/rviz-message-filter-queue-size/
            if i <= 7:
                bump_pub1.publish(range_bump_msg)
                bump_callback.bin_prev_posted[i] = range_bump_msg.range
            else:
                bump_pub2.publish(range_bump_msg)
                bump_callback.bin_prev_posted[i] = range_bump_msg.range

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
    rospy.Subscriber("imu_calib", ImuCalib, imu_calib_callback)

    # Don't exit
    rospy.spin()
