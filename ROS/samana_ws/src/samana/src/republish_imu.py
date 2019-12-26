#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import tf
import math

from samana_msgs.msg import ImuSmall

imu_msg = Imu()
pos_x = pos_y = pos_z = 0
vel_x = vel_y = vel_z = 0
cal_x = cal_y = cal_z = 0
counter = 0
calib_count = 250


def imu_callback(imu_data):
    # global pos_x, pos_y, pos_z,\
    #     vel_x, vel_y, vel_z, \
    #     cal_x, cal_y, cal_z, \
    #     counter

    # if (counter <= calib_count):
    #     # print('calib')
    #     cal_x += imu_data.linear_acceleration_x
    #     cal_y += imu_data.linear_acceleration_y
    #     cal_z += imu_data.linear_acceleration_z
    #     counter += 1
    # if (counter == calib_count + 1):
    #     # print('set')
    #     cal_x /= 500
    #     cal_y /= 500
    #     cal_z /= 500
    #     counter += 1
    #     print("%f %f %f" % (cal_x, cal_y, cal_z))
    # elif (counter > calib_count):
    #     # print('pose')
    #     vel_x += imu_data.linear_acceleration_x - cal_x
    #     vel_y += imu_data.linear_acceleration_y - cal_y
    #     vel_z += imu_data.linear_acceleration_z - cal_z

    #     pos_x += vel_x
    #     pos_y += vel_y
    #     pos_z += vel_z

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

        imu_pub = rospy.Publisher("imu", Imu, queue_size=10)
        imu_pub.publish(imu_msg)

        # br = tf.TransformBroadcaster()
        # transf = tf.Transformer()
        # m = geometry_msgs.msg.TransformStamped()
        # m.header.stamp = imu_data.header.stamp
        # m.header.frame_id = "THISFRAME"
        # m.child_frame_id = "CHILDFRAME"
        # m.transform.rotation.x = imu_data.quaternion_x  
        # m.transform.rotation.y = imu_data.quaternion_y  
        # m.transform.rotation.z = imu_data.quaternion_z  
        # m.transform.rotation.w = imu_data.quaternion_w
        # transf.setTransform(m)
        # br.sendTransformMessage(m)
        
        # imu_pose_pub.publish(pose_data)


if __name__ == '__main__':
    print("START")
    try:
        rospy.init_node("imu_repub")

        rospy.Subscriber("imu_data", ImuSmall, imu_callback)
        

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
