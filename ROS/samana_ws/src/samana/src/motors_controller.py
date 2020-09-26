#!/usr/bin/env python
'''
Gintaras Grebliunas
combinacijus@gmail.com
mototors_controller ROS Node
Sends commands to hoverboard motors. Gathers commands from:
hoverboard pid controller or remote teleop controller from rc_main.py

(Not sure) On failsafe should go to autonomous mode because failsafe values of RX
'''

import math
from copy import deepcopy
from helpers import IsFresh, limit_x, clamp
import rospy
from samana_msgs.msg import Int16Array
from samana_msgs.msg import Teleop
from samana_msgs.msg import RCModes
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from robot_localization.srv import ToggleFilterProcessing, ToggleFilterProcessingRequest


class MotorsController:
    def __init__(self):
        # Constants
        self.MAX_SPEED = clamp(rospy.get_param("hoverboard/max_speed", 200), 0, 1000)
        self.MAX_STEER = clamp(rospy.get_param("hoverboard/max_steer", 170), 0, 1000)
        self.MAX_ACCEL_LIN = max(0, rospy.get_param("hoverboard/max_accel_lin", 1000))
        self.MAX_ACCEL_ANG = max(0, rospy.get_param("hoverboard/max_accel_ang", 1000))

        self.MAX_AUTON_VEL_LIN = max(0, rospy.get_param("hoverboard/max_auton_vel_lin", 0.3))
        self.MAX_AUTON_VEL_ANG = max(0, rospy.get_param("hoverboard/max_auton_vel_ang", 0.8))
        self.MAX_AUTON_ACCEL_LIN = max(0, rospy.get_param("hoverboard/max_auton_accel_lin", 0.3))
        self.MAX_AUTON_ACCEL_ANG = max(0, rospy.get_param("hoverboard/max_auton_accel_ang", 0.8))

        self.MAX_RC_VEL_LIN = max(0, rospy.get_param("hoverboard/max_rc_vel_lin", 1.0))
        self.MAX_RC_VEL_ANG = max(0, rospy.get_param("hoverboard/max_rc_vel_ang", 1.0))
        self.MAX_RC_ACCEL_LIN = max(0, rospy.get_param("hoverboard/max_rc_accel_lin", 5.0))
        self.MAX_RC_ACCEL_ANG = max(0, rospy.get_param("hoverboard/max_rc_accel_ang", 5.0))

        self.SWITCH_TRIG = 950  # Trigger value for a switch
        self.TELEOP_RATE = 40  # Command sending to motor frequency

        # Variables
        self.allow_rc = False    # When false always ignore RC
        self.armed = False       # When false ignore RC
        self.auton_mode = False  # When True read /cmd_vel and publish it to PID controller
        self.auton_mode_prev = False  # To track change od auton_mode

        # Is fresh variables
        self.fresh_cmd_vel = IsFresh(0.4, "Velocity command")
        self.fresh_rc_teleop = IsFresh(0.15, "Remote teleop")
        self.fresh_odom = IsFresh(0.1, "Odometry")

    def init_ros(self):
        rospy.init_node('motors_controller', anonymous=True)

        # Publishers
        self.teleop_pub = rospy.Publisher('teleop', Teleop, queue_size=1)
        self.setpoint_vel_pub = rospy.Publisher('vel_pid/setpoint', Float64, queue_size=1)
        self.state_vel_pub = rospy.Publisher('vel_pid/state', Float64, queue_size=1)
        self.pid_vel_enable_pub = rospy.Publisher('/vel_pid/pid_enable', Bool, queue_size=1, latch=True)
        self.setpoint_yaw_pub = rospy.Publisher('yaw_pid/setpoint', Float64, queue_size=1)
        self.state_yaw_pub = rospy.Publisher('yaw_pid/state', Float64, queue_size=1)
        self.pid_yaw_enable_pub = rospy.Publisher('yaw_pid/pid_enable', Bool, queue_size=1, latch=True)
        self.audio_pub = rospy.Publisher('text_to_speech', String, queue_size=5)

        # Messages
        self.cmd_vel = Twist()
        self.teleop = Teleop()
        self.teleop_prev = Teleop()  # Used for measuring change
        self.rc_teleop = Teleop()
        self.pid_teleop = Teleop()
        self.float64 = Float64()     # For pids
        self.enable_msg = Bool()     # For pids
        self.enable_msg.data = None  # To be sure

        # Subscribers
        rospy.Subscriber("rc/modes", RCModes, self.rc_modes_calback)
        rospy.Subscriber("rc/teleop", Teleop, self.rc_teleop_callback)
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("vel_pid/control_effort", Float64, self.pid_vel_callback)
        rospy.Subscriber("yaw_pid/control_effort", Float64, self.pid_yaw_callback)

        # Services
        self.toggle_ekf_srv = rospy.ServiceProxy("/ekf_odom/toggle", ToggleFilterProcessing)

        # Infinite loop
        self.control_motors()

    def enable_pids(self, enable):
        '''
            Publishes to all pid topic to enable or disable them only on enable change
            And re-enables pids if auton mode has changed
        '''
        def publish_to_pid_enable():
            self.pid_vel_enable_pub.publish(self.enable_msg)
            self.pid_yaw_enable_pub.publish(self.enable_msg)

        if enable is True:  # Enable pids
            if self.enable_msg.data is not True:
                # Disable on auton mode change
                if self.auton_mode != self.auton_mode_prev:
                    self.enable_msg.data = False
                    publish_to_pid_enable()
                # Enable
                self.enable_msg.data = True
                publish_to_pid_enable()
        elif self.enable_msg.data is not False:  # Disable pids
            self.enable_msg.data = False
            publish_to_pid_enable()

    def reset_pids(self):
        '''
            Gets rid of windup
        '''
        self.enable_pids(False)
        self.enable_pids(True)

    def rc_modes_calback(self, modes):
        '''
            Updates RC modes
        '''
        self.allow_rc = modes.allow_rc
        self.armed = modes.armed
        self.auton_mode_prev = self.auton_mode  # Track previous value
        self.auton_mode = modes.auton_mode

    def rc_teleop_callback(self, data):
        '''
            Updates rc_teleop
        '''
        self.rc_teleop = data
        self.fresh_rc_teleop.updated()

    def cmd_vel_callback(self, cmd_vel, fake=False):
        '''
            Accepts cmd_vel either from ROS topic or from pasing cmd_vel to this funtion
            If auton_mode is true publishes limited (velocity and accel) cmd_vel setpoint for a pid controller 
            Save self.cmd_vel to a variable and updates its freshness
        '''
        def limit_cmd_vel(vel, vel_prev):
            '''
                Limits cmd_vel linear and angular speed and acceleration depending on mode
            '''
            TR = self.TELEOP_RATE

            if fake is False:  # Auton mode
                vel.linear.x = limit_x(vel.linear.x, vel_prev.linear.x, x_max_abs=self.MAX_AUTON_VEL_LIN, dx_max=self.MAX_AUTON_ACCEL_LIN / TR)
                vel.angular.z = limit_x(vel.angular.z, vel_prev.angular.z, x_max_abs=self.MAX_AUTON_VEL_ANG, dx_max=self.MAX_AUTON_ACCEL_ANG / TR)
            else:  # RC mode
                vel.linear.x = limit_x(vel.linear.x, vel_prev.linear.x, x_max_abs=self.MAX_RC_VEL_LIN, dx_max=self.MAX_RC_ACCEL_LIN / TR)
                vel.angular.z = limit_x(vel.angular.z, vel_prev.angular.z, x_max_abs=self.MAX_RC_VEL_ANG, dx_max=self.MAX_RC_ACCEL_ANG / TR)

            return vel

        self.cmd_vel = limit_cmd_vel(cmd_vel, self.cmd_vel)
        if fake is False:
            self.fresh_cmd_vel.updated()

        # Velocity setpoint
        self.float64.data = self.cmd_vel.linear.x
        self.setpoint_vel_pub.publish(self.float64)

        # Yaw speed setpoint
        self.float64.data = self.cmd_vel.angular.z
        self.setpoint_yaw_pub.publish(self.float64)

    def odom_callback(self, odom):
        '''
            Publishes state data for PID controller
            Updates that fresh odometry received
        '''
        # State for velocity PID
        self.float64.data = odom.twist.twist.linear.x
        self.state_vel_pub.publish(self.float64)

        # State for yaw speed PID
        self.float64.data = odom.twist.twist.angular.z
        self.state_yaw_pub.publish(self.float64)

        # Fresh odometry received
        self.fresh_odom.updated()

    def pid_vel_callback(self, effort):
        '''
            Updates pid commanded pid_teleop.speed 
        '''
        self.pid_teleop.speed = effort.data * 1000

        if math.isnan(self.pid_teleop.speed) is True:
            self.pid_teleop.speed = 0

    def pid_yaw_callback(self, effort):
        '''
            Updates pid commanded pid_teleop.speed 
        '''
        self.pid_teleop.steer = effort.data * 1000

        if math.isnan(self.pid_teleop.steer) is True:
            self.pid_teleop.steer = 0

    def control_motors(self):
        '''
        Publishes to /teleop speed and steer command for hoverboard motors
        Mode RC: Reads data from /rc
        Mode Autonomous: Reads data from /cmd_vel
        '''
        # In Hz should be atleast 20. Should be as fast as RC input
        # At 50Hz or above sync is lost! (softSerial baud 9600)
        # If faster update rate is needed try higher baud rate for hoverboard serial
        rate = rospy.Rate(self.TELEOP_RATE)  # Recommended 40Hz
        while not rospy.is_shutdown():  # ---------------------------Infinite loop
            # -------------------------------- SAFETY ---------------------------------------
            # Disable EKF filter when no odometry available and enable if it's fresh
            # Disable any control because PID requires odometry and if it fails it will go at full speed
            if self.fresh_odom.changed() is True:  # On odom freshes change
                is_fresh = self.fresh_odom.is_fresh()
                req = ToggleFilterProcessingRequest(is_fresh)
                try:
                    self.toggle_ekf_srv(req)
                except Exception as e:
                    rospy.logwarn("Can't toggle ekf node: {}".format(e))
                    
                self.reset_pids()  # Maybe not needed but to be sure
                self.enable_pids(is_fresh)

                if self.fresh_odom.is_fresh() is False:
                    self.audio_pub.publish("Critical drive disabled")
                else:
                    self.audio_pub.publish("Drive enabled")

            if self.fresh_odom.is_fresh() is False:
                # Stop hoverboard don't allow any movement until odometry data is updated
                self.teleop.speed = 0
                self.teleop.steer = 0
            else:  # Everything ok. Continue with normal control
                # ------------------------------- CONTROL STATES --------------------------------
                if self.allow_rc is True and self.armed is True:  # ---------RC control
                    # Check if rc_teleop is not stale
                    if self.fresh_rc_teleop.is_fresh() is True:
                        self.enable_pids(True)
                    else:  # rc_teleop stale
                        # Zero rc_teleop
                        self.rc_teleop.speed = 0
                        self.rc_teleop.steer = 0
                        self.enable_pids(False)

                    # self.teleop = self.rc_teleop  # Old pure RC control

                    # --------------------------- PID RC CONTROL ---------------------------
                    # Sub and Publish state vel to state_vel topic (in odom_callback())

                    # Velocity setpoint in m/s
                    rc_cmd_vel = Twist()
                    rc_cmd_vel.linear.x = self.rc_teleop.speed / 1000.0 * self.MAX_RC_VEL_LIN
                    rc_cmd_vel.angular.z = self.rc_teleop.steer / 1000.0 * self.MAX_RC_VEL_ANG
                    self.cmd_vel_callback(rc_cmd_vel, fake=True)

                    # These are updated in callback function
                    self.teleop.speed = self.pid_teleop.speed
                    self.teleop.steer = self.pid_teleop.steer
                elif self.auton_mode is True:  # ----------- Autonomous control
                    '''
                        Brief: Executes /cmd_vel commands
                        Reads /cmd_vel in callback
                        Uses PID to find apropriate teleop commands
                        Sets teleop commands
                    '''
                    self.enable_pids(True)

                    # Check if /cmd_vel is stale and should not be used
                    if self.fresh_cmd_vel.is_fresh() is True:
                        '''
                            Do only if auton_mode is True (set inside cmd_vel_callback function)
                            Publish cmd_vel velocity as setpoint for pid controller (cmd_vel_callback())
                            Sub and Publish state vel to state_vel topic (odom_callback())
                            Sub to effort_vel topic (pid_vel_callback())
                            Set effort to teleop
                        '''
                        self.teleop.speed = self.pid_teleop.speed
                        self.teleop.steer = self.pid_teleop.steer
                    else:  # cmd_vel is stale
                        self.reset_pids()
                        self.teleop.speed = 0
                        self.teleop.steer = 0
                    # print('Auton mode: speed: {:2.2f}  steer: {:2.2f}'.format(self.teleop.speed, self.teleop.steer))
                else:  # ----------------------------------- Fully disarmed no control
                    self.teleop.speed = 0
                    self.teleop.steer = 0
                    self.enable_pids(False)

            # Global limit teleop speed and acceleration (in units [-1000..1000])
            TR = self.TELEOP_RATE
            self.teleop.speed = limit_x(self.teleop.speed, self.teleop_prev.speed, x_max_abs=self.MAX_SPEED, dx_max=self.MAX_ACCEL_LIN / TR)
            self.teleop.steer = limit_x(self.teleop.steer, self.teleop_prev.steer, x_max_abs=self.MAX_STEER, dx_max=self.MAX_ACCEL_ANG / TR)
            self.teleop_prev = deepcopy(self.teleop)

            # Publish teleop command to hoverboard
            self.teleop_pub.publish(self.teleop)

            rate.sleep()


if __name__ == '__main__':
    mot_ctrl = MotorsController()

    try:
        mot_ctrl.init_ros()
    except rospy.ROSInterruptException:
        pass
