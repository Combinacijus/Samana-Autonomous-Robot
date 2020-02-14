#!/usr/bin/env python
'''
Gintaras Grebliunas
combinacijus@gmail.com
mototors_controller ROS Node
TODO modify description
Gathers data from sensors and tells hoverboard motors what to do

Modes are switched in rc_callback()
On failsafe should go to autonomous mode because failsafe values of RX
'''

import rospy
import math
from samana_msgs.msg import Int16Array
from samana_msgs.msg import Teleop
from samana_msgs.msg import RCModes
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class MotorsController:
    def __init__(self):
        # Constants
        self.MAX_SPEED = rospy.get_param("hoverboard/max_speed", 200)
        self.MAX_STEER = rospy.get_param("hoverboard/max_steer", 170)
        self.MAX_ACCEL_LIN = rospy.get_param("hoverboard/max_accel_lin", 400)
        self.MAX_ACCEL_ANG = rospy.get_param("hoverboard/max_accel_ang", 400)
        self.CMD_VEL_TIMEOUT = rospy.Duration(4)  # TODO Change to 0.3
        self.SWITCH_TRIG = 950  # Trigger value for a switch
        self.TELEOP_RATE = 40  # Command sending to motor frequency

        # Variables for motor control
        self.speed_rc = 0
        self.steer_rc = 0

        self.allow_rc = False
        self.armed = False  # If false ignore RC
        self.auton_mode = False  # If True read /cmd_vel
        self.last_cmd_vel_stale = True  # If True don't use cmd_vel
        self.last_cmd_vel_update = rospy.Time()
        self.last_rc_teleop_update = rospy.Time()

    def init_ros(self):
        rospy.init_node('motors_controller', anonymous=True)

        # Publishers
        self.teleop_pub = rospy.Publisher('teleop', Teleop, queue_size=10)
        self.setpoint_vel_pub = rospy.Publisher(
            'vel_pid/setpoint', Float64, queue_size=1)
        self.state_vel_pub = rospy.Publisher(
            'vel_pid/state', Float64, queue_size=1)
        self.setpoint_yaw_pub = rospy.Publisher(
            'yaw_pid/setpoint', Float64, queue_size=1)
        self.state_yaw_pub = rospy.Publisher(
            'yaw_pid/state', Float64, queue_size=1)
        self.pub_audio = rospy.Publisher(
            'text_to_speech', String, queue_size=5)

        # Messages
        self.cmd_vel = Twist()
        self.teleop = Teleop()
        self.teleop_prev = Teleop() # Used for measuring change
        self.rc_teleop = Teleop()
        self.pid_teleop = Teleop()
        self.float64 = Float64()

        # Subscribers
        rospy.Subscriber("rc/modes", RCModes, self.rc_modes_calback)
        rospy.Subscriber("rc/teleop", Teleop, self.rc_teleop_callback)
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("vel_pid/control_effort",
                         Float64, self.pid_vel_callback)
        rospy.Subscriber("yaw_pid/control_effort",
                         Float64, self.pid_yaw_callback)

        # Infinite loop
        self.control_motors()

    def clamp(self, _val, _min, _max):
        return min(_max, max(_min, _val))

    def rc_modes_calback(self, modes):
        '''
            Updates RC modes
        '''
        self.allow_rc = modes.allow_rc
        self.armed = modes.armed
        self.auton_mode = modes.auton_mode

    def rc_teleop_callback(self, data):
        self.rc_teleop = data
        self.last_rc_teleop_update = rospy.Time()

    def cmd_vel_callback(self, cmd_vel):
        '''
            Save cmd_vel to a variable for later use in control funciton
            Publish velocity and yaw speed for a pid node if auton_mode is true
        '''
        self.cmd_vel = cmd_vel
        self.last_cmd_vel_update = rospy.Time()

        if self.auton_mode is True:
            # Velocity setpoint
            self.float64.data = cmd_vel.linear.x
            self.setpoint_vel_pub.publish(self.float64)

            # Yaw speed setpoint
            self.float64.data = cmd_vel.angular.z
            self.setpoint_yaw_pub.publish(self.float64)

    def odom_callback(self, odom):
        # State for velocity PID
        self.float64.data = odom.twist.twist.linear.x
        self.state_vel_pub.publish(self.float64)

        # State for yaw speed PID
        self.float64.data = odom.twist.twist.angular.z
        self.state_yaw_pub.publish(self.float64)

    def pid_vel_callback(self, effort):
        self.pid_teleop.speed = effort.data * 1000

    def pid_yaw_callback(self, effort):
        self.pid_teleop.steer = effort.data * 1000

    def control_motors(self):
        '''
        Publishes to /teleop speed and steer command for hoverboard motors
        Mode RC: Reads data from /rc
        Mode Autonomous: Reads data from /cmd_vel
        '''

        def limit_accel():
            '''
                Limits self.teleop speed and steer change in unit/sec^2
            '''
            sign = lambda x: math.copysign(1, x) # sign function

            print(self.MAX_ACCEL_LIN)
            # Limit linear acceleration
            max_accel_lin = self.MAX_ACCEL_LIN / self.TELEOP_RATE
            teleop_accel_lin = self.teleop.speed - self.teleop_prev.speed
            if abs(teleop_accel_lin) > max_accel_lin: # To fast accel
                self.teleop.speed = self.teleop_prev.speed + (max_accel_lin * sign(teleop_accel_lin))
            self.teleop_prev.speed = self.teleop.speed


            # Limit angular acceleration
            max_accel_ang = self.MAX_ACCEL_ANG / self.TELEOP_RATE
            teleop_accel_ang = self.teleop.steer - self.teleop_prev.steer
            if abs(teleop_accel_ang) > max_accel_ang: # To fast accel
                self.teleop.steer = self.teleop_prev.steer + (max_accel_ang * sign(teleop_accel_ang))
            self.teleop_prev.steer = self.teleop.steer

            

        # In Hz should be atleast 20. Should be as fast as RC input
        # At 50Hz or above sync is lost! (softSerial baud 9600)
        # If faster update rate is needed try higher baud rate for hoverboard serial
        rate = rospy.Rate(self.TELEOP_RATE)  # Recommended 40Hz
        while not rospy.is_shutdown():  # ---------------------------Infinite loop
            if self.allow_rc is True and self.armed is True:
                self.teleop = self.rc_teleop
            elif self.auton_mode is True:  # ------------------------Autonomous control
                # Brief: Executes /cmd_vel commands
                # Reads /cmd_vel in callback
                # Uses PID to find apropriate teleop commands
                # Sets teleop commands

                # Check if /cmd_vel is stale and should not be used
                # if rospy.Time.now() - self.last_cmd_vel_update > self.CMD_VEL_TIMEOUT: # cmd_vel is stale
                #     if self.last_cmd_vel_stale is not True: # Audio feedback
                #         self.pub_audio.publish("Velocity command stale")
                #     self.last_cmd_vel_stale = True

                #     self.teleop.speed = 0
                #     self.teleop.steer = 0
                # else:  # cmd_vel is good
                #     # ONLY IF AUTON MODE IS TRUE
                #     # Publish cmd_vel velocity to setpoint topic
                #     # Sub and Publish state vel to state_vel topic
                #     # Sub to effort_vel topic
                #     # Set effort to teleop

                #     self.last_cmd_vel_stale = False

                #     self.teleop.speed = self.pid_teleop.speed
                #     print('auton')
                #     print(self.teleop.speed)
                #     # self.teleop.speed = self.cmd_vel.linear.x
                #     # self.teleop.steer = self.cmd_vel.angular.z

                self.teleop.speed = self.pid_teleop.speed
                self.teleop.steer = self.pid_teleop.steer

                limit_accel() # Limit acceleration from pid linear and angular

                print('auton %f %f' % (self.teleop.speed, self.teleop.steer))
            else:  # ------------------------------------------------Fully disarmed no control
                self.teleop.speed = 0
                self.teleop.steer = 0

            # Global clamp
            self.teleop.speed = self.clamp(
                self.teleop.speed, -self.MAX_SPEED, self.MAX_SPEED)
            self.teleop.steer = self.clamp(
                self.teleop.steer, -self.MAX_STEER, self.MAX_STEER)

            # Publish teleop to hoverboard
            self.teleop_pub.publish(self.teleop)

            rate.sleep()


if __name__ == '__main__':
    mot_ctrl = MotorsController()

    try:
        mot_ctrl.init_ros()
    except rospy.ROSInterruptException:
        pass
