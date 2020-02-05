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
from std_msgs.msg import UInt8


class ServoController:
    def __init__(self):
        # Constants
        self.SERVO_POS1 = 10
        self.SERVO_POS2 = 60

        self.last_servo_update = rospy.Time()

    def init_ros(self):
        rospy.init_node('servo_controller', anonymous=True)

        # Message
        self.servo_msg = UInt8()

        # Subscriber
        self.servo_pub = rospy.Publisher("servo", UInt8, queue_size=1, latch=True)

        self.servo_control() # Infinite loop


    def servo_control(self):
        rate = rospy.Rate(2)

        while True:
            self.last_servo_update = rospy.Time()

            if self.servo_msg.data == self.SERVO_POS1:
                self.servo_msg.data = self.SERVO_POS2
            else:
                self.servo_msg.data = self.SERVO_POS1

            self.servo_pub.publish(self.servo_msg)
    

if __name__ == '__main__':
    servo_ctrl = ServoController()

    try:
        servo_ctrl.init_ros()
    except rospy.ROSInterruptException:
        pass
