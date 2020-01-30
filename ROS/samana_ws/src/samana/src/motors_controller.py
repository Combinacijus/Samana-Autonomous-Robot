#!/usr/bin/env python
'''
Gintaras Grebliunas
combinacijus@gmail.com
motors_controller ROS Node
Gathers data from sensors and tells hoverboard motors what to do

Modes are switched in rc_callback()
On failsafe should go to autonomous mode because failsafe values of RX
'''

import rospy
from samana_msgs.msg import Int16Array
from samana_msgs.msg import Teleop
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MotorsController:
    def __init__(self):
        # Constants
        self.MAX_SPEED = 1000
        self.MAX_STEER = 1000
        self.MAX_RC_SPEED = 1000
        self.MAX_RC_STEER = 1000
        self.MAX_AUTON_SPEED = 100
        self.MAX_AUTON_STEER = 100
        self.CMD_VEL_TIMEOUT = rospy.Duration(0.3)
        self.SWITCH_TRIG = 950  # Trigger value for a switch

        # Variables for motor control
        self.speed_rc = 0
        self.steer_rc = 0
        self.cmd_vel = Twist()
        self.last_cmd_vel_update = rospy.Time()
        self.use_rc = False  # If false ignore RC
        self.last_use_rc = False
        self.auton_mode = False  # If True read /cmd_vel
        self.last_auton_mode = False
        self.last_cmd_vel_stale = True  # If True don't use cmd_vel

    def init_ros(self):
        rospy.init_node('motors_controller', anonymous=True)
        self.pub_audio = rospy.Publisher(
            'text_to_speech', String, queue_size=5)

        self.init_subscribers()
        self.control_motors()  # Infinite loop

    def clamp(self, _val, _min, _max):
        return min(_max, max(_min, _val))

    def rc_callback(self, rc):
        '''
        Radio controller data callback function
        CH1: Roll | CH2: Pitch | CH3: Throtlle | CH4: Yaw
        CH5: Switches | CH6: 2-Pos-Switch | CH7: Knob
        CH1 - rc.data[0] etc...
        '''
        # rospy.loginfo(rospy.get_caller_id() + "  RC   %s", rc.data)

        # Switch RC control and Autonomous mode on or off by 2 pos-switch
        if (rc.data[5] >= self.SWITCH_TRIG):
            self.use_rc = True
        else:  # Disarmed don't use RC
            self.use_rc = False
            self.speed_rc = 0
            self.steer_rc = 0

            # Also check for autonomous mode
            if abs(rc.data[4]) < 50:
                self.auton_mode = True
            else:
                self.auton_mode = False

            return

        # RC to speed and steer
        power_coef = (rc.data[6] + 1000) / 2000.0  # Knob channel
        self.steer_rc = rc.data[0] * power_coef
        self.speed_rc = rc.data[1] * power_coef

        # Adding throttle channel
        # if (rc.data[2] > 0 and power_coef >= 0.2):
        #     self.speed_rc += rc.data[2] / 4.0

        # Add deadzone for RC inputs
        if (abs(self.steer_rc) < 20):
            self.steer_rc = 0
        if (abs(self.speed_rc) < 20):
            self.speed_rc = 0

    def hov_callback(self, hov):
        '''Hoverboard debug data callback function'''
        # rospy.loginfo(rospy.get_caller_id() + "  HOV  %s", hov.data)

    def cmd_vel_callback(self, cmd_vel):
        ''' Save cmd_vel to a variable for later use in control funciton'''
        self.cmd_vel = cmd_vel
        self.last_cmd_vel_update = rospy.Time.now()

    def init_subscribers(self):
        '''Subscribe to all required topics'''
        rospy.Subscriber("hov", Int16Array, self.hov_callback)
        rospy.Subscriber("rc", Int16Array, self.rc_callback)
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

    def control_motors(self):
        '''
        Publishes to /teleop speed and steer command for hoverboard motors
        Mode RC: Reads data from /rc
        Mode Autonomous: Reads data from /cmd_vel
        '''

        pub = rospy.Publisher('teleop', Teleop, queue_size=10)

        # In Hz should be atleast 20. Should be as fast as RC input
        # At 50Hz or above sync is lost! (softSerial baud 9600)
        # If faster update rate is needed try higher baud rate for hoverboard serial
        rate = rospy.Rate(40)  # Recommended 40Hz

        teleop = Teleop()
        while not rospy.is_shutdown():  # Infinite loop
            # TODO Failsafe
            if self.use_rc is True:
                self.auton_mode = False

                # Audio feedback
                if self.last_use_rc is not True:
                    self.pub_audio.publish("Hoverboard armed")

                teleop.speed = self.speed_rc
                teleop.steer = self.steer_rc
            elif self.auton_mode is True:  # Autonomous control
                # Brief: Executes /cmd_vel commands
                # Reads /cmd_vel in callback
                # Uses PID to find apropriate teleop commands
                # Sets teleop commands

                # Audio feedback
                if self.last_auton_mode is not True:
                    self.pub_audio.publish("Autonomous mode")

                if rospy.Time.now() - self.last_cmd_vel_update > self.CMD_VEL_TIMEOUT:
                    # /cmd_vel is stale and should not be used
                    # rospy.loginfo_throttle(1, "cmd_vel stale!")

                    # Audio feedback
                    if self.last_cmd_vel_stale is not True:
                        self.pub_audio.publish("Velocity command stale")
                    self.last_cmd_vel_stale = True

                    teleop.speed = 0
                    teleop.steer = 0
                else:  # cmd_vel is good
                    self.last_cmd_vel_stale = False
                    teleop.speed = self.cmd_vel.linear.x
                    teleop.steer = self.cmd_vel.angular.z

            else:  # Fully disarmed no control
                # Audio feedback
                if self.last_use_rc is not False or self.last_auton_mode is not False:
                    self.pub_audio.publish("Hoverboard disarmed")

                teleop.speed = 0
                teleop.steer = 0

            # Setting old values
            self.last_use_rc = self.use_rc
            self.last_auton_mode = self.auton_mode

            # Safety
            if self.use_rc is True and self.auton_mode is True:
                teleop.speed = 0
                teleop.steer = 0
            
            # Clamp values
            max_speed = 0
            max_steer = 0
            if self.use_rc is True:
                max_steer = self.MAX_RC_STEER
                max_speed = self.MAX_RC_SPEED
            if self.auton_mode is True:
                max_steer = self.MAX_AUTON_STEER
                max_speed = self.MAX_AUTON_SPEED
            
            # Global clamp
            max_speed = min(self.MAX_SPEED, max_speed)
            max_steer = min(self.MAX_STEER, max_steer)

            teleop.speed = self.clamp(teleop.speed, -max_speed, max_speed)
            teleop.steer = self.clamp(teleop.steer, -max_steer, max_steer)

            pub.publish(teleop)
            rate.sleep()


if __name__ == '__main__':
    mot_ctrl = MotorsController()

    try:
        mot_ctrl.init_ros()
    except rospy.ROSInterruptException:
        pass
