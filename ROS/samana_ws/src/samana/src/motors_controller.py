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
from samana_msgs.msg import Int16Array
from samana_msgs.msg import Teleop
from samana_msgs.msg import RCModes
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MotorsController:
    def __init__(self):
        # Constants
        self.MAX_SPEED = rospy.get_param("hoverboard/max_speed", 200)
        self.MAX_STEER = rospy.get_param("hoverboard/max_steer", 150)
        self.CMD_VEL_TIMEOUT = rospy.Duration(0.3)
        self.SWITCH_TRIG = 950  # Trigger value for a switch

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
        self.pub_audio = rospy.Publisher(
            'text_to_speech', String, queue_size=5)

        # Messages
        self.cmd_vel = Twist()
        self.teleop = Teleop()
        self.rc_teleop = Teleop()  # Placeholder for rc commands

        # Subscribers
        rospy.Subscriber("rc/modes", RCModes, self.rc_modes_calback)
        rospy.Subscriber("rc/teleop", Teleop, self.rc_teleop_callback)
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

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
        ''' Save cmd_vel to a variable for later use in control funciton'''
        self.cmd_vel = cmd_vel
        self.last_cmd_vel_update = rospy.Time()

    def control_motors(self):
        '''
        Publishes to /teleop speed and steer command for hoverboard motors
        Mode RC: Reads data from /rc
        Mode Autonomous: Reads data from /cmd_vel
        '''

        # In Hz should be atleast 20. Should be as fast as RC input
        # At 50Hz or above sync is lost! (softSerial baud 9600)
        # If faster update rate is needed try higher baud rate for hoverboard serial
        rate = rospy.Rate(40)  # Recommended 40Hz

        while not rospy.is_shutdown():  # Infinite loop
            if self.allow_rc is True and self.armed is True:
                self.teleop = self.rc_teleop
            elif self.auton_mode is True:  # Autonomous control
                # Brief: Executes /cmd_vel commands
                # Reads /cmd_vel in callback
                # Uses PID to find apropriate teleop commands
                # Sets teleop commands

                if rospy.Time.now() - self.last_cmd_vel_update > self.CMD_VEL_TIMEOUT:
                    # /cmd_vel is stale and should not be used
                    # rospy.loginfo_throttle(1, "cmd_vel stale!")

                    # Audio feedback
                    if self.last_cmd_vel_stale is not True:
                        self.pub_audio.publish("Velocity command stale")
                    self.last_cmd_vel_stale = True

                    self.teleop.speed = 0
                    self.teleop.steer = 0
                else:  # cmd_vel is good
                    self.last_cmd_vel_stale = False
                    self.teleop.speed = self.cmd_vel.linear.x
                    self.teleop.steer = self.cmd_vel.angular.z

            else:  # Fully disarmed no control
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
