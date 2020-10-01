#!/usr/bin/env python
'''
Gintaras Grebliunas
combinacijus@gmail.com

Single node which manages remote control commands for all controllers

Calculates and publishes allow_rc, armed etc. modes and pubslihes it

Calculate teleop and arm commands and pusblishes it to /rc namespace
for use by controllers in manual mode

On failsafe should go to autonomous mode because failsafe values of RX
'''

import rospy
from samana_msgs.msg import Int16Array
from samana_msgs.msg import Teleop
from samana_msgs.msg import RCModes
from samana_msgs.msg import ArmCmd
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest


class RCMain:
    def __init__(self):
        self.RC_MODE = 1         # Either mode 1 or 2 NOTE: set to the radio transmitter mode
        self.SWITCH_TRIG = 900   # Trigger value for a switch
        self.CHANNEL_TRIG = 800  # Trigger value for continuous value

        self.rc_timeout = rospy.Duration(0.1)

        # Teleop
        self.MAX_SPEED = 1000
        self.MAX_STEER = 1000
        self.DEADZONE = 20  # Sticks deadzone
        self.speed = 0
        self.steer = 0
        self.last_teleop_stop = False  # Was the last teleop msg stop

        # Arm                   NOTE: must be same as in arm_controller.py
        self.ARM_MODE_NONE = 0
        self.ARM_MODE_FORCE = 1
        self.ARM_MODE_LIMIT = 2
        self.GRABBER_STOP = 0
        self.GRABBER_OPEN = 1
        self.GRABBER_CLOSE = 2
        self.LIFTER_STOP = 0
        self.LIFTER_RAISE = 1
        self.LIFTER_LOWER = 2
        self.grabber_cmd = self.GRABBER_STOP
        self.lifter_cmd = self.LIFTER_STOP
        self.last_arm_cmd_stop = False

        # Modes
        self.allow_rc = rospy.get_param("init_allow_rc", True)
        self.armed = False
        self.auton_mode = False
        self.arm_mode = self.ARM_MODE_NONE

        # Counters for limiting publish rate
        self.arm_cmd_counter = 0
        self.rc_modes_counter = 0

        # Other
        self.last_rc_update = rospy.Time()  # Last time msg received from Arduino
        self.last_rc_stop = False           # Was the last rc msg stop

    def init_ros(self):
        rospy.init_node('rc_main')

        # Publishers
        self.pub_audio = rospy.Publisher('text_to_speech', String, queue_size=5)
        self.modes_pub = rospy.Publisher('/rc/modes', RCModes, queue_size=1, latch=True)
        self.teleop_pub = rospy.Publisher('/rc/teleop', Teleop, queue_size=1, latch=True)
        self.arm_cmd_pub = rospy.Publisher('/rc/arm_cmd', ArmCmd, queue_size=1, latch=True)

        # Messages
        self.modes_msg = RCModes()
        self.teleop_msg = Teleop()
        self.arm_cmd_msg = ArmCmd()

        # Subscribers
        rospy.Subscriber("rc", Int16Array, self.rc_callback)

        # Services
        rospy.Service("set_allow_rc", SetBool, self.handle_set_allow_rc)

        # Timers
        rospy.Timer(rospy.Duration(0.5), self.health_check)

        rospy.spin()

    def clamp(self, _val, _min, _max):
        return min(_max, max(_min, _val))

    def health_check(self, _):
        '''
            Checks if RC commands aren't too old
            If it is. It publishes off state to all rc topics 
            This will trigger when Arduino controlling hoverboard is disconnected
            When RX is disconnected Arduino prints error: FAILSAFE! and publishes default rc channel values
            which supossed to put hoverboard into autonomous mode 
        '''

        if self.last_rc_stop is not True:
            if rospy.Time.now() - self.last_rc_update > self.rc_timeout:
                # Arduino disconnected stop all RC
                self.publish_teleop_stop()
                self.publish_rc_modes_disable_all()
                self.publish_arm_cmd_stop()
                self.pub_audio.publish("Hoverboard disconnected")
                # print("dt: {}  tout: {}".format(rospy.Time.now() - self.last_rc_update, self.rc_timeout))
                self.last_rc_stop = True

    def rc_modes_publish(self):
        # Setup message
        self.modes_msg.allow_rc = self.allow_rc
        self.modes_msg.armed = self.armed
        self.modes_msg.auton_mode = self.auton_mode
        self.modes_msg.arm_mode = self.arm_mode
        # Publish the message
        self.modes_pub.publish(self.modes_msg)

    def publish_rc_modes_disable_all(self):
        '''
            Disables all rc_modes and publishes it
        '''
        self.armed = False
        self.auton_mode = True
        self.arm_mode = self.ARM_MODE_NONE
        self.rc_modes_publish()

    def rc_modes_update(self, rc):
        '''
            Finds rc modes: allow_rc, armed, auton_mode, arm_mode
            And published modes to rc modes topic
            Also publishes to a text to speech topic on mode change
        '''

        # Lower publishing rate
        self.rc_modes_counter += 1
        if self.rc_modes_counter >= 5:
            self.rc_modes_counter = 0
        else:
            return

        if self.allow_rc is True:
            # -----------------------ARMED MODE--------------------------
            # Switch arm by 2 pos-switch
            if rc.data[5] >= self.SWITCH_TRIG:
                if self.armed is False:
                    self.armed = True
                    self.auton_mode = False
                    self.pub_audio.publish("Armed")
            elif self.armed is True:  # Disarmed don't use RC
                self.armed = False
                self.pub_audio.publish("Disarmed")

            # ----------------------AUTONOMOUS MODE----------------------
            # NOTE: Check for autonomous mode (conditions are same as failsafe)
            if self.armed is False:
                # If both 3-pos switches at the middle
                if abs(rc.data[4]) < 50:
                    if self.auton_mode is False:
                        self.auton_mode = True
                        self.pub_audio.publish("Autonomous mode")
                elif self.auton_mode is True:
                    self.auton_mode = False
                    self.pub_audio.publish("Fully disarmed")

            # -----------------------ARM MODE----------------------------
            # Both 3-pos switches down or up
            if rc.data[4] < -self.SWITCH_TRIG:  # Both up
                if self.arm_mode != self.ARM_MODE_FORCE:
                    self.arm_mode = self.ARM_MODE_FORCE
                    if self.armed is True:
                        self.pub_audio.publish("Arm mode force")
            elif rc.data[4] > self.SWITCH_TRIG:  # Both down
                if self.arm_mode != self.ARM_MODE_LIMIT:
                    self.arm_mode = self.ARM_MODE_LIMIT
                    if self.armed is True:
                        self.pub_audio.publish("Arm mode limit")
            elif self.arm_mode != self.ARM_MODE_NONE:
                self.arm_mode = self.ARM_MODE_NONE
                if self.armed is True:
                    self.pub_audio.publish("Arm mode disabled")

            # -----------------------PUBLISH-----------------------------
            self.rc_modes_publish()  # Publish RC modes
        else:  # -----------RC not allowed disable all modes-------------
            self.publish_rc_modes_disable_all()  # Publishes inside

    def publish_teleop_stop(self):
        self.teleop_msg.speed = 0
        self.teleop_msg.steer = 0
        self.teleop_pub.publish(self.teleop_msg)

    def rc_teleop_update(self, rc):
        '''
            Calculates teleop control values from RC input
            And publishes it to /rc/teleop topic if it is allowed
            When RC is not allowed it publishes one message to the topic to stop
        '''

        # ------------------------Calculating control values------------------------
        if self.allow_rc is True and self.armed is True:
            # RC to speed and steer
            power_coef = (rc.data[6] + 1000) / 2000.0  # Knob channel
            self.speed = rc.data[1] * power_coef  # Pitch
            if self.RC_MODE == 1:
                self.steer = -rc.data[3] * power_coef  # Roll | Negative to conform to REP that CCW is positive
            elif self.RC_MODE == 2:
                self.steer = -rc.data[0] * power_coef  # Negative to conform to REP that CCW is positive
            else:
                rospy.logerr("RADIO TRANSMITTER WRONG MODE SELECTED: {}".format(self.RC_MODE))

            # Add deadzone for RC inputs
            if (abs(self.speed) < self.DEADZONE):
                self.speed = 0
            if (abs(self.steer) < self.DEADZONE):
                self.steer = 0

            # Clamp values to accepted range
            self.speed = self.clamp(self.speed, -self.MAX_SPEED, self.MAX_SPEED)
            self.steer = self.clamp(self.steer, -self.MAX_STEER, self.MAX_STEER)
        else:  # Hoverboard disarmed or RC is not allowed
            self.speed = 0
            self.steer = 0

        # ------------------------Publishing control values------------------------
        can_publish = \
            self.allow_rc is True and \
            self.armed is True and \
            self.auton_mode is False

        if can_publish is True:  # Publish teleop message to /rc namespace
            self.last_teleop_stop = False
            self.teleop_msg.speed = self.speed
            self.teleop_msg.steer = self.steer
            self.teleop_pub.publish(self.teleop_msg)
        elif self.last_teleop_stop is False:  # Publish stop message once
            self.last_teleop_stop = True
            self.publish_teleop_stop()

    def publish_arm_cmd_stop(self):
        self.arm_cmd_msg.grabber_cmd = self.GRABBER_STOP
        self.arm_cmd_msg.lifter_cmd = self.LIFTER_STOP
        self.arm_cmd_pub.publish(self.arm_cmd_msg)

    def rc_arm_update(self, rc):
        '''
            Calculates arm commands from RC input
            And pubslishes to /rc/arm_cmd
        '''

        # Lower publishing rate
        self.arm_cmd_counter += 1
        if self.arm_cmd_counter >= 5:
            self.arm_cmd_counter = 0
        else:
            return

        # ------------------------Calculating control values------------------------
        if self.allow_rc is True and self.armed is True:
            # Yaw channel controls grabber
            if self.RC_MODE == 1:
                if rc.data[0] < -self.CHANNEL_TRIG:
                    self.grabber_cmd = self.GRABBER_OPEN
                elif rc.data[0] > self.CHANNEL_TRIG:
                    self.grabber_cmd = self.GRABBER_CLOSE
                else:
                    self.grabber_cmd = self.GRABBER_STOP
            elif self.RC_MODE == 2:
                if rc.data[3] < -self.CHANNEL_TRIG:
                    self.grabber_cmd = self.GRABBER_OPEN
                elif rc.data[3] > self.CHANNEL_TRIG:
                    self.grabber_cmd = self.GRABBER_CLOSE
                else:
                    self.grabber_cmd = self.GRABBER_STOP
            else:
                self.grabber_cmd = self.GRABBER_STOP
                rospy.logerr("RADIO TRANSMITTER WRONG MODE SELECTED: {}".format(self.RC_MODE))

            # Throttle channel controls lifter
            if rc.data[2] > self.CHANNEL_TRIG:
                self.lifter_cmd = self.LIFTER_RAISE
            elif rc.data[2] < -self.CHANNEL_TRIG:
                self.lifter_cmd = self.LIFTER_LOWER
            else:
                self.lifter_cmd = self.LIFTER_STOP
        else:  # Disarmed or RC is not allowed
            self.grabber_cmd = self.GRABBER_STOP
            self.lifter_cmd = self.LIFTER_STOP

        # ------------------------Publishing control values------------------------
        can_publish = \
            self.allow_rc is True and \
            self.armed is True and \
            self.auton_mode is False

        if can_publish is True:  # Publish arm_cmd message to /rc namespace
            self.last_arm_cmd_stop = False
            self.arm_cmd_msg.grabber_cmd = self.grabber_cmd
            self.arm_cmd_msg.lifter_cmd = self.lifter_cmd
            self.arm_cmd_pub.publish(self.arm_cmd_msg)
        elif self.last_arm_cmd_stop is False:  # Publish stop message once
            self.last_arm_cmd_stop = True
            self.publish_arm_cmd_stop()

    def rc_callback(self, rc):
        '''
        Radio controller data callback function
        Bolow channels in Mode 2:
        CH1: Roll | CH2: Pitch | CH3: Throtlle | CH4: Yaw
        CH5: Switches | CH6: 2-Pos-Switch | CH7: Knob
        CH1 - rc.data[0] etc...
        '''
        # rospy.loginfo(rospy.get_caller_id() + "  RC   %s", rc.data)

        # ------------------------Check if RC allowed------------------------
        self.last_rc_update = rospy.Time.now()
        # if self.last_rc_stop is not False:
        #     self.pub_audio.publish("Hoverboard connected")
        self.last_rc_stop = False

        if self.allow_rc is False:
            return  # Remote control is not allowed return

        self.rc_modes_update(rc)
        self.rc_teleop_update(rc)
        self.rc_arm_update(rc)

    def handle_set_allow_rc(self, req):
        if isinstance(req, SetBoolRequest):
            self.allow_rc = req.data
            if self.allow_rc is True:
                self.pub_audio.publish("Remote enabled")
            else:
                self.pub_audio.publish("Remote disabled")
            return SetBoolResponse(True, '')
        return SetBoolResponse(False, 'Wrong data type')


if __name__ == '__main__':
    rc_main = RCMain()

    try:
        rc_main.init_ros()
    except rospy.ROSInterruptException:
        pass
