#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from samana_msgs.msg import Int16Array, Bump
from geometry_msgs.msg import Point, Twist
from tf import TransformListener
import tf
from helpers import clamp
import math
import time
import signal
import sys


def signal_handler(sig, frame):
    sys.exit(0)


class BugAlgorithmServer:
    """
        Not fully implemented. Drives only with vector sum withing one state
    """
    def __init__(self):
        # Constants
        self.base_frame_id = "base_link"

        self.STATE_GOAL = 0
        self.STATE_WALL = 1

        self.DRIVE_FWD = 0
        self.DRIVE_BWD = 1
        self.DRIVE_CCW = 2
        self.DRIVE_CW = 3

        self.SONAR_COUNT = 10
        self.BUMP_COUNT = 15

        # Variables
        self.to_ang = 3.85  # Multiplier from linear to angular velocity
        self.controller_freq = 12  # How often to send cmd_vel commnads
        self.distance_tolerance = 1.5
        self.power_coef = 1.0  # Updated in rc callback
        # self.rc_speed = 0  # Testing only

        self.max_vel_x = 0.35
        self.max_ang_z = self.max_vel_x * self.to_ang
        self.max_heading_fix = 0.4

        self.state = self.STATE_GOAL  # 0. goal - go to goal  1. wall - follow wall
        self.drive = self.DRIVE_FWD  # 0. goal - go to goal  1. wall - follow wall
        self.sonar_hist = [[] for _ in range(self.SONAR_COUNT)]
        self.range_vector_sum = [0, 0]  # x - fwd, y - right
        self.sonar_readings = [[] for _ in range(self.SONAR_COUNT)]
        self.bump_readings = [[] for _ in range(self.BUMP_COUNT)]
        self.bin_data_old = "0" * self.BUMP_COUNT

        self.sonar_angles = [math.radians(x) for x in [0, -20, -45, -90, -155, 155, 90, 45, 20, 0]]
        self.bump_angles = [0, 0, 0, -0.5, -1.57, -2.35, -2.35, 3.14, 2.35, 2.35, 1.57, 0.5, 0, 0, 0]
        self.bump_vel = 0.4  # Bump vector lenght

        self.sonar_max_ranges = [0.15] + [0.5] * 8 + [0.15]  # NOTE: tuning filter
        self.sonars_fwd = [0, 1, 8, 9]
        self.sonars_angle = [2, 7]
        self.sonars_right = [3, ]
        self.sonars_left = [6, ]
        self.sonars_bwd = [4, 5]

        self.bump_fwd = [0, 1, 2, 12, 13, 14]
        self.bump_bwd = [6, 7, 8]
        self.bump_right = [4, 5]
        self.bump_left = [9, 10]

        # Actionlib server
        self.server = actionlib.SimpleActionServer("bug_algorithm", MoveBaseAction, self.execute, auto_start=False)
        self.server.start()

        # Publishers
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber("sonar", Int16Array, self.sonar_callback)
        rospy.Subscriber("bump", Bump, self.bump_callback)
        # rospy.Subscriber("rc", Int16Array, self.rc_callback)

        # Messages
        self.cmd_vel_msg = Twist()

        # Tf listener
        rospy.loginfo("Starting a tf listner.")
        self.listener = tf.TransformListener()
        rospy.loginfo("Started bug_algorithm server.")

        # Manual testing
        # goal = MoveBaseGoal()
        # goal.target_pose.header.frame_id = "map"
        # goal.target_pose.pose.position = Point(10, 10, 0)
        # goal.target_pose.pose.orientation = waypoint.pose.orientation
        # rez = self.execute(goal)
        # print("rez: {}".format(rez))

    def execute(self, goal):
        rospy.loginfo("Bug algorithm goal: x:{} y:{}".format(goal.target_pose.pose.position.x, goal.target_pose.pose.position.x))

        rate = rospy.Rate(self.controller_freq)
        waypoint = goal.target_pose
    
        dist, yaw_error = self.waypoint_info(waypoint)
    
        # Calculate timeout
        start_time = rospy.Time.now().to_sec()
        timeout = rospy.Duration(dist / 0.3 * 1.2 + 3).to_sec()
        rospy.loginfo("dist: {}, now: {}, timeout: {}, dt: {}".format(dist, start_time, timeout, rospy.Duration(dist / 0.3 * 1.1 + 3).to_sec()))

        while dist > self.distance_tolerance:
            # If preemt
            if self.server.is_preempt_requested():
                rospy.loginfo("PREEMPT bug_algorithm")
                self.server.set_preempted()
                return
            
            # If timeout
            if rospy.Time.now().to_sec() - start_time > timeout:
                rospy.loginfo("ABORTED bug algorithm")
                self.server.set_aborted()
                return

            try:
                # Calculate waypoint data
                dist, yaw_error = self.waypoint_info(waypoint)
                # print("dist: {:.3f} yaw_err:{:.2f}".format(dist, yaw_error))
                
                # Switch states
                if dist < self.distance_tolerance:
                    break  # Waypoint reached
                
                # Sum sonar and bump sensor vectors
                vel_x, ang_z = self.get_range_vector_sum()

                # Return to heading P controller
                ang_z -= clamp(0.8 * yaw_error, -self.max_heading_fix, self.max_heading_fix)

                # Publish cmd_vel
                # self.cmd_vel_msg.linear.x = (vel_x + self.rc_speed) * self.power_coef  # Testing
                self.cmd_vel_msg.linear.x = (vel_x + 0.35) * self.power_coef  # Event
                self.cmd_vel_msg.angular.z = ang_z * self.power_coef

                self.cmd_vel_msg.linear.x = clamp(self.cmd_vel_msg.linear.x, -self.max_vel_x, self.max_vel_x)
                self.cmd_vel_msg.angular.z = clamp(self.cmd_vel_msg.angular.z, -self.max_ang_z, self.max_ang_z)

                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                # print("PUB: {:.3f}, {:.3f}".format(self.cmd_vel_msg.linear.x, self.cmd_vel_msg.angular.z))
            except Exception as e:
                rospy.logerr("HOPE NO CRASH: {}".format(e))

            rate.sleep()

        rospy.loginfo("Done bug algorithm")
        self.server.set_succeeded(result=MoveBaseResult())

    def waypoint_info(self, waypoint):
        self.listener.waitForTransform(waypoint.header.frame_id, self.base_frame_id, rospy.Time(0), rospy.Duration(4.0))
        trans, rot = self.listener.lookupTransform(waypoint.header.frame_id, self.base_frame_id, rospy.Time(0))
        x = waypoint.pose.position.x
        y = waypoint.pose.position.y
        distance = math.sqrt(pow(x - trans[0], 2) + pow(y - trans[1], 2))

        yaw_robot = euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])[2]
        yaw_goal = math.atan2(y - trans[1], x - trans[0])
        yaw_error = yaw_robot - yaw_goal

        return distance, yaw_error

    # def state_goal(self):
    #     pass
    
    # def state_wall(self):
    #     pass
    
    def get_range_vector_sum(self):
        """
            Calculate vector to which direction to move
            By combining data from sonar and bump sensors
            :return: command for cmd_vel (vel_x, ang_z)
        """
        vel_x = 0  # x -fwd
        ang_z = 0  # y - right

        for i in range(len(self.sonar_readings)):
            dist = self.sonar_readings[i]
            if dist is not None: 
                vel_x -= math.cos(self.sonar_angles[i]) * (self.sonar_max_ranges[i] - dist)
                ang_z -= math.sin(self.sonar_angles[i]) * (self.sonar_max_ranges[i] - dist) * self.to_ang

            if dist is not None and dist < 0.15 and i in [3, 6]:  # Overwrite side sonar sensors due to strange robot footprint
                ang_z += 1.5 * math.sin(self.sonar_angles[i]) * (self.sonar_max_ranges[i] - dist) * self.to_ang  # So it turn back away from wall when close
            

        for i in range(self.BUMP_COUNT):    
            if self.bump_readings[i] == 1:  # If bumped
                print("{}. ADD x:{}  z: {}".format(i, math.cos(self.bump_angles[i]), math.sin(self.bump_angles[i])))
                vel_x -= math.cos(self.bump_angles[i]) * self.bump_vel
                ang_z -= math.sin(self.bump_angles[i]) * self.bump_vel * self.to_ang

        return vel_x, ang_z

    # def rc_callback(self, rc):
    #     # For limiting max speed testing only
    #     self.power_coef = (rc.data[6] + 1000) / 2000.0  # Knob channel
    #     self.power_coef *= 3
    #     self.power_coef = min(1, max(-1, self.power_coef))

    #     self.rc_speed = rc.data[1] * self.power_coef / 1000.0

    def sonar_callback(self, range_data):
        def is_outlier(reading, i=0):
            """
                Outliers filter for sonars. Considers max speed and recency of readings
                :param reading: new distance reading
                :param i: index of sonar
                :return: True if reading is outlier and should be discarded

                Readings come at 24Hz or 41.7ms
            """
            # NOTE: tuning range filter parameters
            HIST_COUNT = 6  # 6*41.7 = 250.2ms | 7*41.7 = 291.9ms
            MAX_OUTLIERS = int(HIST_COUNT // 3)  # For every 3 readings 1 outlier is allowed
            MAX_DELTA_PER_READING = 0.0292  # 0.5m/s / 24Hz = 0.021m | 0.7m/s - 0.0292cm | 0.03 * 7 = 0.21
            NOISE_LEVEL = 0.0275  # Found by graphing sonar data

            # Store history of readings
            self.sonar_hist[i].append(reading)
            if len(self.sonar_hist[i]) > HIST_COUNT:
                self.sonar_hist[i].pop(0)

            # Find if it's outlier
            outlier = False
            outliers = 0
            for j, dist in enumerate(self.sonar_hist[i]):
                range_thresh = NOISE_LEVEL + MAX_DELTA_PER_READING * j
                if abs(reading - dist) > range_thresh:
                    outliers += 1
                    if outliers > MAX_OUTLIERS:
                        outlier = True
                        break

            return outlier

        for i in range(len(self.sonar_readings)):
            dist = range_data.data[i] / 1000.0
            if dist <= 0 or dist > self.sonar_max_ranges[i] or is_outlier(dist, i) is True:            
                self.sonar_readings[i] = None  # Outlier
            else:
                self.sonar_readings[i] = dist


    def bump_callback(self, bump_data):
        # Converting int16 to string of len 15
        bin_data = '{data:0{width}b}'.format(data=bump_data.bump_bits, width=self.BUMP_COUNT)[::-1]
        for i in range(self.BUMP_COUNT):    
            if bin_data[i] == '1' and self.bin_data_old[i] == '1':  # If bumped
                self.bump_readings[i] = 1
            else:
                self.bump_readings[i] = 0

        # Remember previous bin_data
        self.bin_data_old = bin_data


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)  # Needed for Ctrl+C to work
    rospy.init_node("bug_algorithm_server")
    server = BugAlgorithmServer()
    rospy.spin()
