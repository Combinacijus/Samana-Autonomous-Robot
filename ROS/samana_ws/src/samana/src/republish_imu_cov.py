#!/usr/bin/env python

'''
Adds covariances to /android/imu messages
'''

import rospy
from sensor_msgs.msg import Imu


def imu_callback(imu_data):
    def set_covariances(tup, cov):
        tmp = list(tup)
        tmp[0] = cov
        tmp[4] = cov
        tmp[8] = cov
        return tuple(tmp)
    
    # Add covariances 
    imu_data.orientation_covariance = set_covariances(imu_data.orientation_covariance, 0.01)
    imu_data.angular_velocity_covariance = set_covariances(imu_data.angular_velocity_covariance, 0.01)
    imu_data.linear_acceleration_covariance = set_covariances(imu_data.linear_acceleration_covariance, 0.01)

    # print(imu_data)
    # print(imu_data.orientation_covariance)
    # print(imu_data.angular_velocity_covariance)
    # print(imu_data.linear_acceleration_covariance)

    imu_pub = rospy.Publisher("imu", Imu, queue_size=10)
    imu_pub.publish(imu_data)


if __name__ == "__main__":
    try:
        rospy.init_node("imu_repub_cov")
        rospy.Subscriber("android/imu", Imu, imu_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass