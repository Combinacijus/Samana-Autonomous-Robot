#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
import math
import time

from samana_msgs.msg import ImuSmall

imu_msg = Imu()
pos_x = pos_y = pos_z = 0
vel_x = vel_y = vel_z = 0
cal_x = cal_y = cal_z = 0
counter = 0
calib_count = 250

# For transforming imu to base_link
tf_buffer = 1
tf_listener = 1


def imu_callback(imu_data):
    global tf_buffer, tf_listener

    imu_msg.header.stamp = imu_data.header.stamp
    imu_msg.header.frame_id = "imu_link"

    imu_msg.orientation.x = imu_data.quaternion_x
    imu_msg.orientation.y = imu_data.quaternion_y
    imu_msg.orientation.z = imu_data.quaternion_z
    imu_msg.orientation.w = imu_data.quaternion_w
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration_x
    imu_msg.linear_acceleration.y = imu_data.linear_acceleration_y
    imu_msg.linear_acceleration.z = imu_data.linear_acceleration_z
    imu_msg.angular_velocity.x = math.radians(imu_data.angular_velocity_x)
    imu_msg.angular_velocity.y = math.radians(imu_data.angular_velocity_z)
    imu_msg.angular_velocity.z = math.radians(imu_data.angular_velocity_y)
    # Add covariances NOTE: IMU tuning
    default_cov = (0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01)
    imu_msg.orientation_covariance = default_cov
    imu_msg.angular_velocity_covariance = default_cov
    imu_msg.linear_acceleration_covariance = default_cov

    imu_pub = rospy.Publisher("imu", Imu, queue_size=10)
    imu_pub.publish(imu_msg)

    # Transform map to imu_link -----------------------------------------------------------------
    FRAME_ID = "map"
    CHILD_FRAME_ID = "base_link"
    
    br = tf2_ros.TransformBroadcaster()
    imu_tr = geometry_msgs.msg.TransformStamped()
    
    # p2 = tf2_geometry_msgs.tf2_geometry_msgs.PoseStamped()
    imu_tr.header.stamp = imu_data.header.stamp
    imu_tr.header.frame_id = FRAME_ID
    imu_tr.child_frame_id = CHILD_FRAME_ID
    imu_tr.transform.translation.x = 0
    imu_tr.transform.translation.y = 0
    imu_tr.transform.rotation.x = imu_data.quaternion_x
    imu_tr.transform.rotation.y = imu_data.quaternion_y
    imu_tr.transform.rotation.z = imu_data.quaternion_z
    imu_tr.transform.rotation.w = imu_data.quaternion_w

    imu_to_base_tr = tf_buffer.lookup_transform(
        "imu_link", "base_link", rospy.Time(), rospy.Duration(1.0))

    # Empty pose
    pose_transformed = geometry_msgs.msg.PoseStamped()
    pose_transformed.header = imu_tr.header
    pose_transformed.pose.orientation.w = 1

    # Transforming
    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_transformed, imu_to_base_tr)
    # imu_tr.transform.translation.x = 0
    # imu_tr.transform.translation.y = 0
    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_transformed, imu_tr)

    # Converting back to TransformStamped for broadcasting
    final_transform = geometry_msgs.msg.TransformStamped()
    final_transform.header = pose_transformed.header
    final_transform.header.frame_id = FRAME_ID
    final_transform.child_frame_id = CHILD_FRAME_ID
    final_transform.transform.rotation = pose_transformed.pose.orientation
    # final_transform.transform.translation = pose_transformed.pose.position

    br.sendTransform(final_transform)


if __name__ == '__main__':
    print("START")
    try:
        rospy.init_node("imu_repub")

        rospy.Subscriber("imu_data", ImuSmall, imu_callback)

        # For transforming imu to base_link
        tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))  # tf buffer length
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
