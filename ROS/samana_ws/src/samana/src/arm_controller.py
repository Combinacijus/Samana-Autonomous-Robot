#!/usr/bin/env python
'''
Gintaras Grebliunas
combinacijus@gmail.com
arm_controller ROS Node
Gathers data from sensors and tells arm what to do
'''
# TODO overcurrent limit switch

import rospy
from samana_msgs.msg import Int16Array
from samana_msgs.msg import RCModes
from samana_msgs.msg import ArmData
from samana_msgs.msg import ArmCmd
from std_msgs.msg import String


class ArmController:
    def __init__(self):
        # NOTE: must be same as in rc_main.py
        self.ARM_MODE_NONE = 0
        self.ARM_MODE_FORCE = 1
        self.ARM_MODE_LIMIT = 2
        self.GRABBER_STOP = 0
        self.GRABBER_OPEN = 1
        self.GRABBER_CLOSE = 2
        self.LIFTER_STOP = 0
        self.LIFTER_RAISE = 1
        self.LIFTER_LOWER = 2

        # Variables
        self.grabber_cmd = 0  # 0 - stop; 1 - open; 2 - close
        self.lifter_cmd = 0  # 0 - stop; 1 - raise; 2 - lower
        self.current_grabber = 0
        self.current_lifter = 0
        self.switch_go = -1
        self.switch_gc = -1
        self.switch_lu = -1
        self.switch_ld = -1
        self.rc_data = []
        self.last_rc_arm_cmd_update = rospy.Time()

        # Modes
        self.allow_rc = False
        self.armed = False
        self.auton_mode = False
        self.arm_mode = 0

    def init_ros(self):
        rospy.init_node('arm_controller', anonymous=True)

        # Publishers
        self.pub_arm = rospy.Publisher('arm_cmd', ArmCmd, queue_size=10)
        self.pub_audio = rospy.Publisher(
            'text_to_speech', String, queue_size=10)

        # Messages
        self.arm_cmd_msg = ArmCmd()
        self.rc_arm_cmd_msg = ArmCmd()

        # Subscribers
        rospy.Subscriber("arm_data", ArmData, self.arm_data_callback)
        rospy.Subscriber("rc/modes", RCModes, self.rc_modes_calback)
        rospy.Subscriber("rc/arm_cmd", ArmCmd, self.rc_arm_cmd_callback)
        rospy.Subscriber("rc", Int16Array, self.rc_calback)

        # Infinite loop
        self.control_arm()

    def rc_modes_calback(self, modes):
        '''
            Updates RC modes
        '''
        self.allow_rc = modes.allow_rc
        self.armed = modes.armed
        self.auton_mode = modes.auton_mode
        self.arm_mode = modes.arm_mode

    def rc_arm_cmd_callback(self, data):
        self.rc_arm_cmd_msg = data
        self.last_rc_arm_cmd_update = rospy.Time()

    def rc_calback(self, rc):
        '''
        Radio controller data callback function
        CH1: Roll | CH2: Pitch | CH3: Throtlle | CH4: Yaw
        CH5: Switches | CH6: 2-Pos-Switch | CH7: Knob
        CH1 - rc.data[0] etc...
        '''

        # Copying data for global scope
        self.rc_data = []
        for d in rc.data:
            self.rc_data.append(d)

    def arm_data_callback(self, msg):
        '''Arm data callback function'''
        # rospy.loginfo(rospy.get_caller_id() + "  ARM  %s", msg.data)

        self.current_grabber = msg.current_grabber
        self.current_lifter = msg.current_lifter

        # Decode limit switches data
        # Converting int8 to string of len 4
        bin_data = '{data:0{width}b}'.format(
            data=msg.limit_switches, width=4)[::-1]
        self.switch_go = int(bin_data[0])
        self.switch_gc = int(bin_data[1])
        self.switch_lu = int(bin_data[2])
        self.switch_ld = int(bin_data[3])

    def control_arm(self):
        '''
            Publish to /arm_cmd to control arm motors
           /arm_mode (int)
                0: No move
                1: arm commands will be forced
                2: arm will move until limit or overcurrent switch or disarm
        '''

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.allow_rc is True and self.armed is True and self.arm_mode != self.ARM_MODE_NONE:
                if self.arm_mode == self.ARM_MODE_FORCE:  # Force commands
                    self.grabber_cmd = self.rc_arm_cmd_msg.grabber_cmd
                    self.lifter_cmd = self.rc_arm_cmd_msg.lifter_cmd
                elif self.arm_mode == self.ARM_MODE_LIMIT:  # Drive until limit switch
                    # This or gives ability to get in undefined state which needs rearming to get out
                    # It's not fixed so it blocks motors from being reversed instantly which is bad for motors
                    self.grabber_cmd |= self.rc_arm_cmd_msg.grabber_cmd
                    self.lifter_cmd |= self.rc_arm_cmd_msg.lifter_cmd

                    # Stop when limit switch is hit
                    if (self.grabber_cmd == self.GRABBER_OPEN and self.switch_go) or \
                            (self.grabber_cmd == self.GRABBER_CLOSE and self.switch_gc):
                        self.grabber_cmd = self.GRABBER_STOP
                    if (self.lifter_cmd == self.LIFTER_RAISE and self.switch_lu) or \
                            (self.lifter_cmd == self.LIFTER_LOWER and self.switch_ld):
                        self.lifter_cmd = self.LIFTER_STOP
            else:  # Fully disarmed no control
                # Stop the arm
                self.grabber_cmd = self.GRABBER_STOP
                self.lifter_cmd = self.LIFTER_STOP

            self.arm_cmd_msg.grabber_cmd = self.grabber_cmd
            self.arm_cmd_msg.lifter_cmd = self.lifter_cmd

            self.pub_arm.publish(self.arm_cmd_msg)
            rate.sleep()


if __name__ == '__main__':
    arm_controller = ArmController()

    try:
        arm_controller.init_ros()
    except rospy.ROSInterruptException:
        pass
