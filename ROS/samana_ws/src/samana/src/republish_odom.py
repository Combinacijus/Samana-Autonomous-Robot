#!/usr/bin/env python

'''
    When OdometrySmall message is received translates it
    to Odometry message adjusts depending on pitch and calibration
    and publishes to odom_raw topic
'''

import rospy
from samana_msgs.msg import OdometrySmall, ImuSmall
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty, EmptyResponse
from tf.transformations import euler_from_quaternion
import math


class OdomRepub:
    def __init__(self):
        self.odom_pitch_correction_coef = 1.0  # How much distance travelled forward differ due to pitch angle

        self.reset_odom()
        self.get_params_or_default()
        self.debug = False  # NOTE: set tot true for more debug info

    def init_ros(self):
        try:
            rospy.init_node("odom_repub", anonymous=True)

            s = rospy.Service("reset_odom", Empty, self.handle_reset_odom)
            rospy.Subscriber("odom_data", OdometrySmall, self.odom_cb)
            rospy.Subscriber("imu_data", ImuSmall, self.imu_callback)
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    def reset_odom(self):
        self.posx = 0
        self.posy = 0
        self.theta = 0
        self.theta_raw = 0
        self.theta_cal = 0
        self.rot1 = 0
        self.rot2 = 0

    def get_params_or_default(self):
        '''
            Reads parameters from parameter server or
            if not found sets default values

            NOTE: tuning. How to tune Odometry:
            - self.debug = True
            - rosservice call /reset_odom

            - drive forward in known length track eg. real_len = 4m
            - count number of encoder rotations for both wheels rl and rr
            - self.rot_per_m_l = rl / real_len; self.rot_per_mr = rr / real_len

            - spin in place 10 times (real_th = 3600) get th_raw
            - self.base_width = th_raw / real_th
        '''

        # Default params
        self.surface_factor = 1.0  # On rough surface robot travels less than on flat surface. Bigger = less distance
        # 3600deg: 1884.43241407 1884.67177595 1885.09056770  avg-> 1884.7315859066666
        # 3m L: 11.977917 11.967500 11.966250   avg-> 11.9705556666
        # 3m R: 11.970833 11.975200 11.965417   avg-> 11.9704833333
        self.base_width = 0.5235365516407408
        self.rot_per_m_left = 3.9901852222222223   # old: 3.99536165
        self.rot_per_m_right = 3.9901611111111115  # old: 3.9895417

        # Updated params from param server if available
        if rospy.has_param("base_width"):
            self.base_width = rospy.get_param("base_width")
        if rospy.has_param("rot_per_m_left"):
            self.rot_per_m_left = rospy.get_param("rot_per_m_left")
        if rospy.has_param("rot_per_m_right"):
            self.rot_per_m_right = rospy.get_param("rot_per_m_right")
        if rospy.has_param("surface_factor"):
            self.rot_per_m_right = rospy.get_param("surface_factor")

        self.rot_per_m_left *= self.surface_factor
        self.rot_per_m_right *= self.surface_factor
        self.base_width *= self.surface_factor  # Not sure if it's right

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
        vx = (v_right + v_left) / 2 * self.odom_pitch_correction_coef  # Speed forward in m/s
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

        # Covariance | Make diagonal covariance matrix
        default_cov = [0.0, ]*36
        for i in range(0, 6):
            default_cov[i + i*6] = 0.00001  # NOTE: tuning robot localization

        msg.twist.covariance = tuple(default_cov)
        msg.pose.covariance = tuple(default_cov)

        pub = rospy.Publisher("odom", Odometry, queue_size=10)
        pub.publish(msg)

        if self.debug is True:
            self.calibration_debug_data(odom_data)

    def imu_callback(self, imu_data):
        MIN_PITCH = 0.020  # In radians when to start correcting

        qx = imu_data.quaternion_x
        qy = imu_data.quaternion_y
        qz = imu_data.quaternion_z
        qw = imu_data.quaternion_w
        (pitch, roll, yaw) = euler_from_quaternion([qx, qy, qz, qw])  # PRY instead of RPY due to sensor mounting orientation

        if abs(pitch) < MIN_PITCH:
            pitch = 0.00

        self.odom_pitch_correction_coef = math.cos(pitch)

    def calibration_debug_data(self, od):
        # 1 - left, 2 - right
        TICKS_PER_ROTATION = 2400.0

        v_left = od.rps1 / self.rot_per_m_right  # Wheel speed in m/s
        v_right = od.rps2 / self.rot_per_m_left  # Wheel speed in m/s

        self.theta_raw += (v_right - v_left) * od.dt / 1000000.0  # base_width = 1
        th_raw = math.degrees(self.theta_raw)
        
        self.theta_cal += (v_right - v_left) / self.base_width * od.dt / 1000000.0
        th_cal = math.degrees(self.theta_cal)  # With current base_width calibration

        th = math.degrees(self.theta)  # With current base_width bounded to +-180

        self.rot1 += od.delta_ticks1 / TICKS_PER_ROTATION
        self.rot2 += od.delta_ticks2 / TICKS_PER_ROTATION

        print("vl: {:2.3f}, vr: {:2.3f}, rl: {:3.6f}, rr: {:3.6f}, th: {:4.2f}, th_raw: {:4.8f}, th_cal: {:4.8f}, p_corr: {:1.3f}"
              .format(v_left, v_right, self.rot1, self.rot2, th, th_raw, th_cal, self.odom_pitch_correction_coef))

    def handle_reset_odom(self, _):
        self.reset_odom()
        return EmptyResponse()


if __name__ == "__main__":
    try:
        odom_repub = OdomRepub()  # Goes into infinte loop for ROS
        odom_repub.init_ros()
    except rospy.ROSInterruptException:
        pass
