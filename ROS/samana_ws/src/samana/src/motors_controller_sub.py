#!/usr/bin/env python
'''
Gintaras Grebliunas
combinacijus@gmail.com
motors_controller ROS Node
Gathers data from sensors and tells hoverboard motors what to do
'''

import rospy
from samana_msgs.msg import Int16Array
from samana_msgs.msg import Teleop
from helpers import clamp

# Global constants
MAX_SPEED = 1000
MAX_STEER = 1000

# Global variables for motor control
g_speed_rc = 0
g_steer_rc = 0


def rc_calback(rc):
    '''
    Radio controller data callback function
    CH1: Roll | CH2: Pitch | CH3: Throtlle | CH4: Yaw
    CH5: Switches | CH6: 2-Pos-Switch | CH7: Knob
    CH1 - rc.data[0] etc...
    '''
    global g_speed_rc, g_steer_rc
    # rospy.loginfo(rospy.get_caller_id() + "  RC   %s", rc.data)

    # Switch RC control on or of by 2 pos-switch
    if (rc.data[5] < -50):
        rospy.set_param("/use_rc", False)
    elif (rc.data[5] > 50):
        rospy.set_param("/use_rc", True)

    # RC to speed and steer
    power_coef = (rc.data[6] + 1000) / 2000.0  # Knob channel
    g_steer_rc = rc.data[0] * power_coef
    g_speed_rc = rc.data[1] * power_coef

    # Adding throttle channel
    if (rc.data[2] > 0 and power_coef >= 0.2):
        g_speed_rc += rc.data[2] / 4.0

    if (abs(g_steer_rc) < 20):
        g_steer_rc = 0
    if (abs(g_speed_rc) < 20):
        g_speed_rc = 0

    g_steer_rc = clamp(g_steer_rc, -MAX_STEER, MAX_STEER)
    g_speed_rc = clamp(g_speed_rc, -MAX_SPEED, MAX_SPEED)


def hov_calback(hov):
    '''Hoverboard debug data callback function'''
    # rospy.loginfo(rospy.get_caller_id() + "  HOV  %s", hov.data)


def init_subscribers():
    '''Subscribe to all required topics'''
    rospy.Subscriber("hov", Int16Array, hov_calback)
    rospy.Subscriber("rc", Int16Array, rc_calback)


def control_motors():
    '''Publish to /teleop to control hoverboard motors'''
    global g_speed_rc, g_steer_rc

    pub = rospy.Publisher('teleop', Teleop, queue_size=10)

    # In Hz should be atleast 20. Should be as fast as RC input
    # At 50Hz or above sync is lost (softSerial baud 9600)
    # If faster update rate is needed try higher baud rate for hoverboard serial
    rate = rospy.Rate(40) # Recommended 40Hz
    while not rospy.is_shutdown():
        teleop = Teleop()
        # TODO Failsafe
        use_rc = rospy.get_param("/use_rc")
        # rospy.loginfo(use_rc)

        if use_rc is True:
            teleop.speed = g_speed_rc
            teleop.steer = g_steer_rc
        else:
            teleop.speed = 0
            teleop.steer = 0

        pub.publish(teleop)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('motors_controller', anonymous=True)
    init_subscribers()

    rospy.set_param("/use_rc", True)  # Also inits parameter
    try:
        control_motors() # Infinite loop
    except rospy.ROSInterruptException:
        pass

    rospy.spin()  # spin() simply keeps python from exiting until this node is stopped
