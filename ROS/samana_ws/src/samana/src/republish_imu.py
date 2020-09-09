#!/usr/bin/env python

import math
import time
import rospy
from sensor_msgs.msg import Imu
import tf2_ros

from samana_msgs.msg import ImuSmall

imu_msg = Imu()


def imu_callback(imu_data):
    global imu_msg

    imu_msg.header.stamp = imu_data.header.stamp
    imu_msg.header.frame_id = "imu"

    imu_msg.orientation.x = imu_data.quaternion_x
    imu_msg.orientation.y = imu_data.quaternion_y
    imu_msg.orientation.z = imu_data.quaternion_z
    imu_msg.orientation.w = imu_data.quaternion_w
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration_x
    imu_msg.linear_acceleration.y = imu_data.linear_acceleration_y
    imu_msg.linear_acceleration.z = imu_data.linear_acceleration_z
    imu_msg.angular_velocity.x = imu_data.angular_velocity_x
    imu_msg.angular_velocity.y = imu_data.angular_velocity_y
    imu_msg.angular_velocity.z = imu_data.angular_velocity_z

    # Add covariances NOTE: IMU tuning
    default_cov = (0.004, 0, 0, 0, 0.004, 0, 0, 0, 0.004)
    imu_msg.orientation_covariance = default_cov
    imu_msg.angular_velocity_covariance = default_cov
    imu_msg.linear_acceleration_covariance = default_cov

    imu_pub = rospy.Publisher("imu", Imu, queue_size=10)
    imu_pub.publish(imu_msg)
        

if __name__ == '__main__':
    try:
        rospy.init_node("imu_repub")

        rospy.Subscriber("imu_data", ImuSmall, imu_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
