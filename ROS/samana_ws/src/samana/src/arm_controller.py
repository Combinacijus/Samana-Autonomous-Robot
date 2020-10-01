#!/usr/bin/env python
'''
Gintaras Grebliunas
combinacijus@gmail.com
arm_controller ROS Node
Gathers data from sensors and tells arm what to do
Or executes actionlib command
'''

import rospy
from samana_msgs.msg import Int16Array
from samana_msgs.msg import RCModes
from samana_msgs.msg import ArmData
from samana_msgs.msg import ArmCmd
from std_msgs.msg import String
import actionlib
from samana_msgs.msg import ArmControlAction, ArmControlGoal, ArmControlResult
import time


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

        # NOTE: tuning
        self.current_grabber_max = 950
        self.current_lifter_max = 950
        self.grabber_timeout = 16  # In sec
        self.lifter_timeout = 60  # In sec
        self.min_timeout = 1.5  # In sec
        self.overcurrent_timeout = 0.3  # In sec
        self.small_raise_time = 0.5  # In sec

        # Variables
        self.current_grabber = 0
        self.current_lifter = 0
        self.grabber_cmd = 0  # 0 - stop; 1 - open; 2 - close
        self.lifter_cmd = 0  # 0 - stop; 1 - raise; 2 - lower
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

        # Actionlib server
        self.server = actionlib.SimpleActionServer("arm_control", ArmControlAction, self.execute_action, auto_start=False)

        self.COMMAND_TEXT = ["NONE", "GRABBER_OPEN_FULLY", "GRABBER_CLOSE_FULLY",
                             "LIFTER_RAISE_FULLY", "LIFTER_LOWER_FULLY", "LIFTER_RAISE_SMALL", "OPEN_AND_LOWER"]

    def init_ros(self):
        rospy.init_node("arm_controller")

        # Publishers
        self.pub_arm = rospy.Publisher("arm_cmd", ArmCmd, queue_size=10)
        self.pub_audio = rospy.Publisher("text_to_speech", String, queue_size=10)

        # Messages
        self.arm_cmd_msg = ArmCmd()
        self.rc_arm_cmd_msg = ArmCmd()

        # Subscribers
        rospy.Subscriber("arm_data", ArmData, self.arm_data_callback)
        rospy.Subscriber("rc/modes", RCModes, self.rc_modes_calback)
        rospy.Subscriber("rc/arm_cmd", ArmCmd, self.rc_arm_cmd_callback)
        rospy.Subscriber("rc", Int16Array, self.rc_calback)

        # Actionlib server
        self.server.start()

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
        bin_data = '{data:0{width}b}'.format(data=msg.limit_switches, width=4)[::-1]
        self.switch_go = int(bin_data[0])
        self.switch_gc = int(bin_data[1])
        self.switch_lu = int(bin_data[2])
        self.switch_ld = int(bin_data[3])

    def limit_switch_protection(self):
        """ Stop when limit switch is hit """
        if (self.grabber_cmd == self.GRABBER_OPEN and self.switch_go) or \
                (self.grabber_cmd == self.GRABBER_CLOSE and self.switch_gc):
            self.grabber_cmd = self.GRABBER_STOP

        if (self.lifter_cmd == self.LIFTER_RAISE and self.switch_lu) or \
                (self.lifter_cmd == self.LIFTER_LOWER and self.switch_ld):
            self.lifter_cmd = self.LIFTER_STOP

    def overcurrent_protection(self):
        """ 
        Stop on overcurrent for some time
        :return: grabber and lifter overcurrent flag [goc, loc]
        """
        goc = False  # Grabber overcurrent flag
        loc = False  # Lifter overcurrent flag

        grabber_oc_start = rospy.Time.now()  # oc - overcurrent
        while abs(self.current_grabber) > self.current_grabber_max:
            if (rospy.Time.now() - grabber_oc_start).to_sec() > self.overcurrent_timeout:
                self.grabber_cmd = self.GRABBER_STOP  # To long in overcurrent
                goc = True
                break
            time.sleep(self.overcurrent_timeout / 5.0)

        lifter_oc_start = rospy.Time.now()  # oc - overcurrent
        while abs(self.current_lifter) > self.current_lifter_max:
            if (rospy.Time.now() - lifter_oc_start).to_sec() > self.overcurrent_timeout:
                self.lifter_cmd = self.LIFTER_STOP  # To long in overcurrent
                loc = True
                break
            time.sleep(self.overcurrent_timeout / 5.0)

        return goc, loc

    def publish_arm_cmd(self, grabber_cmd, lifter_cmd):
        self.arm_cmd_msg.grabber_cmd = grabber_cmd
        self.arm_cmd_msg.lifter_cmd = lifter_cmd

        self.pub_arm.publish(self.arm_cmd_msg)

    def control_arm(self):
        '''
            Publish to /arm_cmd to control arm motors
           /arm_mode (int):
                0: No move
                1: arm commands will be forced
                2: arm will move until limit or overcurrent switch or disarm
        '''

        prev_armed = True
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.allow_rc is True and self.armed is True and self.arm_mode != self.ARM_MODE_NONE:
                if self.arm_mode in [self.ARM_MODE_FORCE, self.ARM_MODE_LIMIT]:
                    prev_armed = True

                if self.arm_mode == self.ARM_MODE_FORCE:  # Force commands
                    self.grabber_cmd = self.rc_arm_cmd_msg.grabber_cmd
                    self.lifter_cmd = self.rc_arm_cmd_msg.lifter_cmd
                elif self.arm_mode == self.ARM_MODE_LIMIT:  # Drive until limit switch
                    # This or "|" gives ability to get in undefined state which needs rearming to get out
                    # It blocks motors from being reversed instantly which is bad for motors
                    self.grabber_cmd |= self.rc_arm_cmd_msg.grabber_cmd
                    self.lifter_cmd |= self.rc_arm_cmd_msg.lifter_cmd

                    self.limit_switch_protection()
                    self.overcurrent_protection()

                self.publish_arm_cmd(self.grabber_cmd, self.lifter_cmd)
            elif prev_armed is True:  # Fully disarmed send stop command once
                # Stop the arm
                prev_armed = False
                self.grabber_cmd = self.GRABBER_STOP
                self.lifter_cmd = self.LIFTER_STOP
                self.publish_arm_cmd(self.grabber_cmd, self.lifter_cmd)

            rate.sleep()

    def execute_action(self, goal):
        """ Callback for actionlib server call """
        time_start = rospy.Time.now()

        def time_elapsed():
            return (rospy.Time.now() - time_start).to_sec()

        self.pub_audio.publish("{}".format(self.COMMAND_TEXT[goal.arm_command]))

        goc = False  # Grabber overcurrent flag
        loc = False  # Lifter overcurrent flag

        rate = rospy.Rate(10)
        while True:  # Loop until arm is fully stopped or preempt is requested
            # Preempt check
            if self.server.is_preempt_requested():
                rospy.loginfo("PREEMPT arm_control")
                self.server.set_preempted(ArmControlResult(time_elapsed()))
                return

            # Stop all before setting commands
            self.grabber_cmd = self.GRABBER_STOP
            self.lifter_cmd = self.LIFTER_STOP

            # Set commands
            if self.auton_mode is True and self.armed is False:
                if goal.arm_command == ArmControlGoal.GRABBER_OPEN_FULLY:
                    self.grabber_cmd = self.GRABBER_OPEN

                elif goal.arm_command == ArmControlGoal.GRABBER_CLOSE_FULLY:
                    self.grabber_cmd = self.GRABBER_CLOSE

                elif goal.arm_command == ArmControlGoal.LIFTER_RAISE_FULLY:
                    self.lifter_cmd = self.LIFTER_RAISE

                elif goal.arm_command == ArmControlGoal.LIFTER_LOWER_FULLY:
                    self.lifter_cmd = self.LIFTER_LOWER

                elif goal.arm_command == ArmControlGoal.LIFTER_RAISE_SMALL:
                    self.lifter_cmd = self.LIFTER_RAISE
                    # Custom timeout for short lift
                    if time_elapsed() > self.small_raise_time:
                        self.lifter_cmd = self.LIFTER_STOP

                elif goal.arm_command == ArmControlGoal.OPEN_AND_LOWER:
                    if not goc:  # Set only if overcurrent flag was not triggered
                        self.grabber_cmd = self.GRABBER_OPEN
                    if not loc:  # Set only if overcurrent flag was not triggered
                        self.lifter_cmd = self.LIFTER_LOWER
            
            # Give some time in case it gets stuck at the start
            if time_elapsed() > self.min_timeout or goal.arm_command == ArmControlGoal.LIFTER_RAISE_SMALL:
                # Stop on limit switch
                self.limit_switch_protection()

                # Stop on overcurrent. Don't update overcurrent flags if it's already true
                _goc, _loc = self.overcurrent_protection()
                goc = goc or _goc
                loc = loc or _loc

                # Stop on timeout
                if time_elapsed() > self.grabber_timeout:
                    self.grabber_cmd = self.GRABBER_STOP

                if time_elapsed() > self.lifter_timeout:
                    self.lifter_cmd = self.LIFTER_STOP

                # Publish command to arm
                self.publish_arm_cmd(self.grabber_cmd, self.lifter_cmd)

                # Finish action if both grabber and lifter stoped
                if self.grabber_cmd == self.GRABBER_STOP and self.lifter_cmd == self.LIFTER_STOP:
                    break

            elif goal.arm_command != ArmControlGoal.LIFTER_RAISE_SMALL:
                # It's first time it's run so do some fast switching to unstuck arm
                repeat = 2
                for i in range(repeat):
                    self.publish_arm_cmd(self.GRABBER_STOP, self.LIFTER_STOP)
                    time.sleep(self.min_timeout/(repeat - 0.01)/2.0)

                    self.limit_switch_protection()  # Don't spam if already on limit switch

                    self.publish_arm_cmd(self.grabber_cmd, self.lifter_cmd)
                    time.sleep(self.min_timeout/(repeat - 0.01)/2.0)

            # Repeat at constant rate
            rate.sleep()

        # Command finished
        rospy.loginfo("DONE arm_control {}".format(self.COMMAND_TEXT[goal.arm_command]))
        self.pub_audio.publish("Done")
        self.server.set_succeeded(result=ArmControlResult(time_elapsed()))


if __name__ == '__main__':
    arm_controller = ArmController()

    try:
        arm_controller.init_ros()
    except rospy.ROSInterruptException:
        pass
