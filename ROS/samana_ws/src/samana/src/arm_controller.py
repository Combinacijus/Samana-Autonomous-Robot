#!/usr/bin/env python
'''
Gintaras Grebliunas
combinacijus@gmail.com
arm_controller ROS Node
Gathers data from sensors and tells arm what to do
'''
# TODO overcurrent limit switch
# TODO control via parameters

import rospy
from samana_msgs.msg import Int16Array
from samana_msgs.msg import ArmData
from samana_msgs.msg import ArmCmd
from std_msgs.msg import String


class ArmController:
    def __init__(self):
        # NOTE: important constants for arm commands
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
        self.last_arm_mode = 0
        self.last_use_rc = False

    def init_ros(self):
        self.pub_arm = rospy.Publisher('arm_cmd', ArmCmd, queue_size=10)
        self.pub_audio = rospy.Publisher('text_to_speech', String, queue_size=10)

        rospy.init_node('arm_controller', anonymous=True)
        self.init_subscribers()

        # Different RC control modes
        rospy.set_param("/use_rc", False)
        rospy.set_param("/arm_mode", 0)

        # Infinite loop
        self.control_arm()

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

        # Arming. Switch RC control on or of by 2 pos-switch
        if (rc.data[5] < -50):
            rospy.set_param("/use_rc", False)
        elif (rc.data[5] > 50):
            rospy.set_param("/use_rc", True)

        # Both switches down or both up
        if (rc.data[4] < -950):
            rospy.set_param("/arm_mode", 1)
        elif (rc.data[4] > 950):
            rospy.set_param("/arm_mode", 2)
        else:
            rospy.set_param("/arm_mode", 0)

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

    def init_subscribers(self):
        '''Subscribe to all required topics'''
        rospy.Subscriber("arm_data", ArmData, self.arm_data_callback)
        rospy.Subscriber("rc", Int16Array, self.rc_calback)

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
            # Setup message for pusblishing
            arm_cmd = ArmCmd()

            # Say if armmed or disarmed
            use_rc = rospy.get_param("/use_rc")
            if use_rc != self.last_use_rc:
                self.last_use_rc = use_rc
                postfix = ["disarmed", "armed"]
                txt = "Arm "
                if use_rc:
                    txt += "armmed "
                    if rospy.get_param("/arm_mode") == 0:
                        txt += "no mode selected"
                else:
                    txt += "disarmed "

                print(txt)
                self.pub_audio.publish(txt)
            
            # RC control
            if use_rc is True:  
                # Say arm mode
                arm_mode = rospy.get_param("/arm_mode")
                if arm_mode != self.last_arm_mode and arm_mode != 0:
                    self.last_arm_mode = arm_mode
                    
                    # Assemble message
                    postfix = ["nothing", "force", "auto stop"]
                    txt = "Arm %s" % (postfix[arm_mode])
                    print(txt)
                    self.pub_audio.publish(txt)
                
                # Execute commands from RC
                if arm_mode == 1:  # Force commands
                    # Yaw channel controls grabber
                    if self.rc_data[3] < -600:
                        self.grabber_cmd = self.GRABBER_OPEN
                    elif self.rc_data[3] > 600:
                        self.grabber_cmd = self.GRABBER_CLOSE
                    else:
                        self.grabber_cmd = self.GRABBER_STOP

                    # Throttle channel controls lifter
                    if self.rc_data[2] > 600:
                        self.lifter_cmd = self.LIFTER_LOWER
                    elif self.rc_data[2] < -600:
                        self.lifter_cmd = self.LIFTER_RAISE
                    else:
                        self.lifter_cmd = self.LIFTER_STOP
                elif arm_mode == 2:  # Drive until limit switch
                    # Yaw channel controls grabber
                    if self.rc_data[3] < -600 and not self.switch_go:
                        self.grabber_cmd = self.GRABBER_OPEN
                    elif self.rc_data[3] > 600 and not self.switch_gc:
                        self.grabber_cmd = self.GRABBER_CLOSE

                    # Throttle channel controls lifter
                    if self.rc_data[2] > 600 and not self.switch_lu:
                        self.lifter_cmd = self.LIFTER_LOWER
                    elif self.rc_data[2] < -600 and not self.switch_ld:
                        self.lifter_cmd = self.LIFTER_RAISE

                    # Stop when limit switch is hit
                    if self.grabber_cmd == self.GRABBER_OPEN and self.switch_go:
                        self.grabber_cmd = self.GRABBER_STOP
                    elif self.grabber_cmd == self.GRABBER_CLOSE and self.switch_gc:
                        self.grabber_cmd = self.GRABBER_STOP
                    if self.lifter_cmd == self.LIFTER_RAISE and self.switch_lu:
                        self.lifter_cmd = self.LIFTER_STOP
                    elif self.lifter_cmd == self.LIFTER_LOWER and self.switch_ld:
                        self.lifter_cmd = self.LIFTER_STOP
            else:  # No RC control
                self.grabber_cmd = self.GRABBER_STOP
                self.lifter_cmd = self.LIFTER_STOP

            arm_cmd.grabber_cmd = self.grabber_cmd
            arm_cmd.lifter_cmd = self.lifter_cmd

            self.pub_arm.publish(arm_cmd)
            rate.sleep()


if __name__ == '__main__':
    arm_controller = ArmController()

    try:
        arm_controller.init_ros()
    except rospy.ROSInterruptException:
        pass
