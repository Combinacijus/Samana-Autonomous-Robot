#!/usr/bin/env python

'''
    When OdometrySmall message is received translates it
    to Odometry message and published to odom_raw topic
'''

import rospy
from samana_msgs.msg import OdometrySmall
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler
import math


class OdomRepub:
    def __init__(self):
        self.posx = 0
        self.posy = 0
        self.theta = 0

        self.get_params_or_default()

        # ROS stuff
        try:
            rospy.init_node("odom_repub", anonymous=True)

            if rospy.has_param('base_width'):
                pass

            rospy.Subscriber("odom", OdometrySmall, self.odom_cb)
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    def get_params_or_default(self):
        '''
            Reads parameters from parameter server or
            if not found sets default values
        '''
        # Default params
        self.base_width = 0.8  # In meters
        self.rot_per_m_left = 2.1
        self.rot_per_m_right = 2.1

        # Updated params from param server if available
        if rospy.has_param("base_width"):
            self.base_width = rospy.get_param("base_width", )
        if rospy.has_param("rot_per_m_left"):
            self.base_width = rospy.get_param("rot_per_m_left")
        if rospy.has_param("rot_per_m_right"):
            self.base_width = rospy.get_param("rot_per_m_right")

    def odom_cb(self, odom_data):
        '''
            Converts OdometrySmall to Odometry message and publishes to odom_raw topic
        '''
        dt = odom_data.dt / 1000000.0  # Delta time converted to seconds

        msg = Odometry()
        msg.header.stamp = odom_data.header.stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Foward kinematics
        v_left = odom_data.rps1 / self.rot_per_m_right  # Wheel speed in m/s
        v_right = odom_data.rps2 / self.rot_per_m_left  # Wheel speed in m/s
        vx = (v_right + v_left) / 2  # Speed forward in m/s
        omega = (v_right - v_left) / self.base_width  # Angular speed in rad/s

        # Clamped angle
        self.theta += omega * dt
        if self.theta > math.pi:
            self.theta -= 2*math.pi
        elif self.theta < -math.pi:
            self.theta += 2*math.pi

        # print("Theta: %f, Omega %f" %(self.theta, omega))
        # print("vL: %f, vR %f" %(v_left, v_right))

        # Position
        self.posx += vx * math.cos(self.theta) * dt
        self.posy += vx * math.sin(self.theta) * dt
        msg.pose.pose.position = Point(self.posx, self.posy, 0)  # x - fwd
        # Angle
        quat = quaternion_from_euler(0, 0, self.theta)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        # Linear speed
        msg.twist.twist.linear.x = vx
        # msg.twist.twist.linear.y = 0  # Default already zero

        # Angular speed
        msg.twist.twist.angular.z = omega

        pub = rospy.Publisher("odom_raw", Odometry, queue_size=10)
        pub.publish(msg)


if __name__ == "__main__":
    odom_repub = OdomRepub()  # Goes into infinte loop for ROS
