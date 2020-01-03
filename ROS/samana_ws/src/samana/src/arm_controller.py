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

# Global variables
grabber_cmd = 0  # 0 - stop; 1 - open; 2 - close
lifter_cmd = 0  # 0 - stop; 1 - raise; 2 - lower
current_grabber = 0
current_lifter = 0
switch_go = -1
switch_gc = -1
switch_lu = -1
switch_ld = -1
rc_data = []


def rc_calback(rc):
    '''
    Radio controller data callback function
    CH1: Roll | CH2: Pitch | CH3: Throtlle | CH4: Yaw
    CH5: Switches | CH6: 2-Pos-Switch | CH7: Knob
    CH1 - rc.data[0] etc...
    '''
    global rc_data

    # Copying data for global scope
    rc_data = []
    for d in rc.data:
        rc_data.append(d)

    # Arming. Switch RC control on or of by 2 pos-switch
    if (rc.data[5] < -50):
        rospy.set_param("/use_rc", False)
    elif (rc.data[5] > 50):
        rospy.set_param("/use_rc", True)

    # Both switches down or both up
    if (rc.data[4] < -950):
        rospy.set_param("/mode1", True)
    else:
        rospy.set_param("/mode1", False)
    if (rc.data[4] > 950):
        rospy.set_param("/mode2", True)
    else:
        rospy.set_param("/mode2", False)


def arm_data_callback(msg):
    '''Arm data callback function'''
    global current_grabber, current_lifter, switch_go, switch_gc, switch_lu, switch_ld
    # rospy.loginfo(rospy.get_caller_id() + "  ARM  %s", msg.data)

    current_grabber = msg.current_grabber
    current_lifter = msg.current_lifter

    # Decode limit switches data
    # Converting int8 to string of len 4
    bin_data = '{data:0{width}b}'.format(
        data=msg.limit_switches, width=4)[::-1]
    switch_go = int(bin_data[0])
    switch_gc = int(bin_data[1])
    switch_lu = int(bin_data[2])
    switch_ld = int(bin_data[3])


def init_subscribers():
    '''Subscribe to all required topics'''
    rospy.Subscriber("arm_data", ArmData, arm_data_callback)
    rospy.Subscriber("rc", Int16Array, rc_calback)


def control_arm():
    '''Publish to /arm_cmd to control arm motors'''
    global current_grabber, current_lifter, switch_go, switch_gc, switch_lu, switch_ld
    global grabber_cmd, lifter_cmd, rc_data

    # NOTE: important constants for arm commands
    GRABBER_STOP = 0
    GRABBER_OPEN = 1
    GRABBER_CLOSE = 2
    LIFTER_STOP = 0
    LIFTER_RAISE = 1
    LIFTER_LOWER = 2

    pub = rospy.Publisher('arm_cmd', ArmCmd, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Setup message for pusblishing
        arm_cmd = ArmCmd()

        if rospy.get_param("/use_rc") is True:  # RC control
            if rospy.get_param("/mode1") is True:  # Force commands
                print('mode1')
                # Yaw channel controls grabber
                if rc_data[3] < -600:
                    grabber_cmd = GRABBER_OPEN
                elif rc_data[3] > 600:
                    grabber_cmd = GRABBER_CLOSE
                else:
                    grabber_cmd = GRABBER_STOP

                # Throttle channel controls lifter
                if rc_data[2] > 600:
                    lifter_cmd = LIFTER_RAISE
                elif rc_data[2] < -600:
                    lifter_cmd = LIFTER_LOWER
                else:
                    lifter_cmd = LIFTER_STOP

            if rospy.get_param("/mode2") is True:  # Drive until limit switch
                print('mode2')
                # Yaw channel controls grabber
                if rc_data[3] < -600 and not switch_go:
                    grabber_cmd = GRABBER_OPEN
                elif rc_data[3] > 600 and not switch_gc:
                    grabber_cmd = GRABBER_CLOSE

                # Throttle channel controls lifter
                if rc_data[2] > 600 and not switch_lu:
                    lifter_cmd = LIFTER_RAISE
                elif rc_data[2] < -600 and not switch_ld:
                    lifter_cmd = LIFTER_LOWER

                # Stop when limit switch is hit
                if grabber_cmd == GRABBER_OPEN and switch_go:
                    grabber_cmd = GRABBER_STOP
                elif grabber_cmd == GRABBER_CLOSE and switch_gc:
                    grabber_cmd = GRABBER_STOP
                if lifter_cmd == LIFTER_RAISE and switch_lu:
                    lifter_cmd = LIFTER_STOP
                elif lifter_cmd == LIFTER_LOWER and switch_ld:
                    lifter_cmd = LIFTER_STOP

        else:  # No RC control
            grabber_cmd = GRABBER_STOP
            lifter_cmd = LIFTER_STOP

        arm_cmd.grabber_cmd = grabber_cmd
        arm_cmd.lifter_cmd = lifter_cmd

        pub.publish(arm_cmd)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('arm_controller', anonymous=True)
    init_subscribers()

    # Different RC control modes
    rospy.set_param("/mode1", False)
    rospy.set_param("/mode2", False)

    try:
        control_arm()  # Infinite loop
    except rospy.ROSInterruptException:
        pass

    rospy.spin()  # spin() simply keeps python from exiting until this node is stopped
