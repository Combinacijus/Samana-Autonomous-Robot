#!/usr/bin/env python

import rospy
import geometry_msgs
from geometry_msgs.msg import Pose
import tf

from samana_msgs.msg import ImuSmall

pose_data = Pose()
pos_x = pos_y = pos_z = 0
vel_x = vel_y = vel_z = 0
cal_x = cal_y = cal_z = 0
counter = 0
calib_count = 250


def callback(imu_data):
    global pos_x, pos_y, pos_z,\
        vel_x, vel_y, vel_z, \
        cal_x, cal_y, cal_z, \
        counter

    if (counter <= calib_count):
        # print('calib')
        cal_x += imu_data.linear_acceleration_x
        cal_y += imu_data.linear_acceleration_y
        cal_z += imu_data.linear_acceleration_z
        counter += 1
    if (counter == calib_count + 1):
        # print('set')
        cal_x /= 500
        cal_y /= 500
        cal_z /= 500
        counter += 1
        print("%f %f %f" % (cal_x, cal_y, cal_z))
    elif (counter > calib_count):
        # print('pose')
        vel_x += imu_data.linear_acceleration_x - cal_x
        vel_y += imu_data.linear_acceleration_y - cal_y
        vel_z += imu_data.linear_acceleration_z - cal_z

        pos_x += vel_x
        pos_y += vel_y
        pos_z += vel_z

        pose_data.orientation.x = imu_data.quaternion_x
        pose_data.orientation.y = imu_data.quaternion_y
        pose_data.orientation.z = imu_data.quaternion_z
        pose_data.orientation.w = imu_data.quaternion_w
        pose_data.position.x = pos_x
        pose_data.position.y = pos_y
        pose_data.position.z = pos_z

        br = tf.TransformBroadcaster()
        transf = tf.Transformer()
        m = geometry_msgs.msg.TransformStamped()
        m.header.stamp = imu_data.header.stamp
        m.header.frame_id = "THISFRAME"
        m.child_frame_id = "CHILDFRAME"
        m.transform.rotation.x = imu_data.quaternion_x  
        m.transform.rotation.y = imu_data.quaternion_y  
        m.transform.rotation.z = imu_data.quaternion_z  
        m.transform.rotation.w = imu_data.quaternion_w
        transf.setTransform(m)
        br.sendTransformMessage(m)
        
        imu_pose_pub.publish(pose_data)


if __name__ == '__main__':
    print("START")
    try:
        rospy.init_node("imu_pose", anonymous=True)

        rospy.Subscriber("imu_data", ImuSmall, callback)
        imu_pose_pub = rospy.Publisher("imu_pose", Pose, queue_size=10)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
