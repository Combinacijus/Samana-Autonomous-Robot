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
from std_srvs.srv import Empty, EmptyResponse
import math


class OdomRepub:
    def __init__(self):
        self.reset_vals()
        self.get_params_or_default()

    def init_ros(self):
        try:
            rospy.init_node("odom_repub", anonymous=True)

            s = rospy.Service("reset_vals", Empty, self.handle_reset_vals)
            rospy.Subscriber("odom_data", OdometrySmall, self.odom_cb)
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    def reset_vals(self):
        self.posx = 0
        self.posy = 0
        self.theta = 0
        self.theta_debug = 0
        self.rot1 = 0
        self.rot2 = 0

    def get_params_or_default(self):
        '''
            Reads parameters from parameter server or
            if not found sets default values
        '''
        # NOTE: odom tuning
        # Default params
        # base_width = 1, -3600deg:
        # -1885.885155 -1886.326100
        self.base_width = 0.523856988
        # 4m straight track rotation:
        # 15.985200 15.980833 15.968500 15.985000 15.987700
        # 15.936250 15.929167 15.991667 15.959583 15.974167
        self.rot_per_m_left = 3.99536165
        self.rot_per_m_right = 3.9895417

        # Updated params from param server if available
        if rospy.has_param("base_width"):
            self.base_width = rospy.get_param("base_width", )
        if rospy.has_param("rot_per_m_left"):
            self.base_width = rospy.get_param("rot_per_m_left")
        if rospy.has_param("rot_per_m_right"):
            self.base_width = rospy.get_param("rot_per_m_right")

    def odom_cb(self, odom_data):
        '''
            Converts OdometrySmall to Odometry message and publishes to odom topic
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

        pub = rospy.Publisher("odom", Odometry, queue_size=10)
        pub.publish(msg)

        # self.calibration_debug_data(odom_data)

    def calibration_debug_data(self, od):
        v_left = od.rps1 / self.rot_per_m_right  # Wheel speed in m/s
        v_right = od.rps2 / self.rot_per_m_left  # Wheel speed in m/s
        omega = v_right - v_left  # Angular speed in rad/s
        self.theta_debug += omega * od.dt / 1000000.0
        self.rot1 += od.delta_ticks1 / 2400.0
        self.rot2 += od.delta_ticks2 / 2400.0
        print("r1: %f, r2: %f, th: %f, thd: %f" % (self.rot1, self.rot2, self.theta * 180 / math.pi, self.theta_debug * 180 / math.pi))

    def handle_reset_vals(self, _):
        self.reset_vals()
        return EmptyResponse()

if __name__ == "__main__":
    try:
        odom_repub = OdomRepub()  # Goes into infinte loop for ROS
        odom_repub.init_ros()
    except rospy.ROSInterruptException:
        pass
