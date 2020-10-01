#!/usr/bin/env python

"""
    Checks is data is received if not does recovery (rebinds usb)
    NOTE: edit /etc/sudoers by "sudo visudo" like this below to allow to run bash without password
    <USERNAME> ALL=(ALL) NOPASSWD: <PATHTOSCRIPT>/rebind_usb.sh
"""

import rospy
from samana_msgs.msg import Int16Array, ImuCalib
from std_msgs.msg import String
import time
import signal
import sys
import os


def signal_handler(sig, frame):
    sys.exit(0)


class HealthChecker:
    def __init__(self):
        # Variables
        self.last_update = rospy.Time.now()
        self.timeout = rospy.Duration(12.0)

        # Publishers
        self.audio_pub = rospy.Publisher("text_to_speech", String, queue_size=5)

        # Subscribers
        rospy.Subscriber("rc", Int16Array, self.rc_callback)
        rospy.Subscriber("sonar", Int16Array, self.sonar_callback)
        rospy.Subscriber("imu_calib", ImuCalib, self.imu_calib_callback)

    def rc_callback(self, data):
        self.last_update = rospy.Time.now()

    def sonar_callback(self, data):
        self.last_update = rospy.Time.now()
        
    def imu_calib_callback(self, data):
        self.last_update = rospy.Time.now()

    def health_check_loop(self):
        while True:
            time.sleep(3)
            
            # No data is received for some time. Usb might be disconnected
            if rospy.Time.now() - self.last_update > self.timeout:
                self.rebind_usbs()
                self.last_update = rospy.Time.now()

    def rebind_usbs(self):
        self.audio_pub.publish("Rebinding USB")

        path = "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana/src/rebind_usb.sh"
        os.system("sudo {}".format(path))


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)  # Needed for Ctrl+C to work

    rospy.init_node("health_checker")

    health_checker = HealthChecker()
    time.sleep(5.0)  # For first boot to finish
    health_checker.health_check_loop()

    rospy.spin()
