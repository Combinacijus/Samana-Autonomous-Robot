#!/usr/bin/env python

"""
    Main Samana state machine
"""

import rospkg
import rospy
import tf
import smach
import smach_ros
import actionlib
import csv
import math
import signal
import sys
import numpy as np
import time
from time import sleep
from helpers import IsFresh, clamp, sign
from smach import State, StateMachine
from geometry_msgs.msg import PoseArray, Pose, PointStamped, PoseStamped, Twist
from tf.transformations import quaternion_from_euler
from shapely.geometry import Polygon, Point
from std_srvs.srv import SetBool, SetBoolRequest
from std_msgs.msg import String, Empty, Bool
from samana_msgs.msg import FollowWpAction, FollowWpGoal
from samana_msgs.msg import ArmControlAction, ArmControlGoal, ArmControlResult
from samana_msgs.msg import ArmData

# Global
pub_audio = rospy.Publisher("text_to_speech", String, queue_size=3)


def signal_handler(sig, frame):
    sys.exit(0)


class Init(State):
    def __init__(self):
        State.__init__(self, outcomes=["success"])

    def execute(self, userdata):
        time.sleep(0.5)  # For other subscribers to catch up with publishers

        pub_audio.publish("Samana Initialization start")
        # pub_audio.publish("Samana Initialization Done")

        return "success"


class Check(State):
    def __init__(self):
        State.__init__(self, outcomes=["success", "fail"])

    def execute(self, userdata):
        pub_audio.publish("Samana Check")
        return "success"


class DriveToBag(State):
    def __init__(self):
        State.__init__(self, outcomes=["success", "fail"])

        file_path_default = rospkg.RosPack().get_path("samana") + "/config/waypoints_to_bag.csv"
        file_path = rospy.get_param("~file_wp_to_bag", file_path_default)
        self.drive_to_goal = DriveToGoal(file_path=file_path, dist_tolerance=4.0, goal_frame_id="utm", waypoint_name="goldbag")

    def execute(self, userdata):
        pub_audio.publish("Samana driving to bag area")
        return self.drive_to_goal.execute()


class SearchBag(State):
    def __init__(self):
        State.__init__(self, outcomes=["success", "fail"])
        self.frame_id = rospy.get_param("~wp_frame_id", "utm")
        self.center_x = rospy.get_param("~utm_spiral_x", "689830.2577150462")
        self.center_y = rospy.get_param("~utm_spiral_y", "6087794.247483239")

        self.dist_tolerance = 0.2  # NOTE: tune
        self.delta_wp = 0
        self.last_wp = 0
        self.bag_found = False
        self.spiral_radius = 5.0
        self.spiral_radius_addition = 1.0

        self.GOAL_STATES = ("PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED",
                            "REJECTED", "PREEMPTING", "RECALLING", "RECALLED", "LOST")

        rospy.Subscriber("bag_point", PointStamped, self.bag_point_cb)

        rospy.loginfo("Waiting for enable_detector service")
        rospy.wait_for_service("enable_detector")
        self.enable_detector_proxy = rospy.ServiceProxy("enable_detector", SetBool)

    def execute(self, userdata):
        if self.last_wp == 0:
            pub_audio.publish("Samana starting bag search")
        else:
            pub_audio.publish("Samana continuing bag search")

        # TODO change to normal
        # path_gen = PathGenerator(radius=self.spiral_radius/20.0, pitch=0.1, segm_len_min=0.1, arc_ratio_min=0.05)  # NOTE: inside
        # path_gen = PathGenerator(radius=self.spiral_radius, pitch=2.5, segm_len_min=0.8, arc_ratio_min=0.15)  # NOTE: short
        path_gen = PathGenerator(radius=self.spiral_radius, pitch=0.5, segm_len_min=0.3, arc_ratio_min=0.1)  # NOTE: tuning normal

        # Start object detection
        rospy.loginfo("Object detection STARTED")
        self.enable_detector_proxy(SetBoolRequest(True))

        # Generate path
        r, phi = path_gen.spiral()   # Polar coordinates list to numpy array
        x, y = path_gen.xy_from_polar(r, phi, self.center_x, self.center_y)  # Generate x, y coordinates from polar form
        waypoints = self.xy_phi_to_waypoints(x, y, phi, self.frame_id)
        path_gen.print_path_dist(x, y, self.last_wp)

        # Follow waypoints
        self.drive_to_goal = DriveToGoal(waypoints=waypoints, dist_tolerance=self.dist_tolerance,
                                         goal_frame_id=self.frame_id, short_audio=True, feedback_client=self.feedback_cb,
                                         last_wp=self.last_wp)

        self.drive_to_goal.execute(blocking=False, wait_to_start=True)  # Started following search path

        # Search started, start object detection and stop search if found
        gs = actionlib.GoalStatus
        done_state = (gs.SUCCEEDED, gs.PREEMPTED, gs.ABORTED, gs.REJECTED, gs.RECALLED, gs.LOST)
        while self.bag_found is False:
            # If search path finished it failed to find a bag return fail
            goal_state = self.drive_to_goal.client.get_state()
            if goal_state in done_state:
                rospy.loginfo("follow_waypoints finished without finding a bag. State: {}".format(self.GOAL_STATES[goal_state]))
                self.delta_wp = 0
                self.last_wp = 0
                self.spiral_radius += self.spiral_radius_addition
                # TODO global timeout
                return "fail"
            sleep(0.1)

        # Bag is found. Cancel goals and return success
        self.bag_found = False
        rospy.loginfo("Preempting follow_waypoits")
        self.drive_to_goal.client.cancel_all_goals()
        rospy.loginfo("Waiting for follow_waypoits to preempt")
        self.drive_to_goal.client.wait_for_result()
        self.feedback_cb(0)  # Last self.last_wp update

        # Stop object detection
        self.enable_detector_proxy(SetBoolRequest(False))
        rospy.loginfo("Object detection STOPPED")

        return "success"  # Bag found

    def bag_point_cb(self, data):
        """ Sets bag_found flag to true when topic msg is received """
        self.bag_found = True

    def feedback_cb(self, last_wp):
        """
            Updates last_wp
            If it's second time feedback starts counting from 0 although last_wp might not be 0
        """
        if last_wp == 0:  # If feedback is looping back to 0 add delta to current self.last_wp
            self.last_wp += self.delta_wp
        self.delta_wp = last_wp

    def xy_phi_to_waypoints(self, x, y, phi, frame_id):
        """
            param x: numpy array of x coordinates
            param y: numpy array of y coordinates
            param phi: numpy array of yaw in radians
            param frame_id: frame_id of coordinates frame

            return: waypoints (PoseArray)
        """
        waypoints = []

        waypoints = PoseArray()
        waypoints.header.frame_id = frame_id

        for _x, _y, _phi in zip(x, y, phi):
            pose_tmp = Pose()
            pose_tmp.position.x = _x
            pose_tmp.position.y = _y
            pose_tmp.position.z = 0.0

            quat = quaternion_from_euler(0, 0, _phi - math.pi/2.0)  # -90deg because it's circle's phi not path's
            pose_tmp.orientation.x = quat[0]
            pose_tmp.orientation.y = quat[1]
            pose_tmp.orientation.z = quat[2]
            pose_tmp.orientation.w = quat[3]

            waypoints.poses.append(pose_tmp)

        return waypoints


class AlignWithBag(State):
    def __init__(self):
        State.__init__(self, outcomes=["success", "fail"])

        self.bag_point_odom = None  # PointStamped of bag position in /odom      frame (x - forward, y - right)
        self.bag_point_base = None  # List [x, y]  of bag position in /base_link frame (x - forward, y - right)
        self.base_frame = "base_link"
        self.odom_frame = "odom"
        self.cmd_vel_rate = rospy.Rate(10)  # Freaquency how often to send new cmd_vel command

        self.yaw_tolerance = math.pi / 30.0
        self.yaw_p = 0.4  # NOTE: tune yaw P controller
        self.yaw_min = 0.02
        self.yaw_max = math.pi / 10
        self.yaw_goal = 0

        self.pos_tolerance = 0.03
        self.pos_p = 0.30  # NOTE: tune position P controller
        self.pos_min = 0.02
        self.pos_max = 0.10
        self.pos_goal = 0.25
        self.pos_far = 0.4  # If further than this drive forward while aligning angle

        self.point_lock = PointLock(tolerance=0.2, count_to=6, fresh_time=0.7, fresh_name="Bag detection")

        # Publishers
        self.bag_pose_pub = rospy.Publisher("bag_pose", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # Subscribers
        self.subscriber_enabled = False
        rospy.Subscriber("bag_point", PointStamped, self.bag_point_cb)

        # Service
        rospy.loginfo("Waiting for enable_detector service")
        rospy.wait_for_service("enable_detector")
        self.enable_detector_proxy = rospy.ServiceProxy("enable_detector", SetBool)

        self.tf_listener = tf.TransformListener()

        # Polygon areas of action
        self.area_drive_back1 = Polygon([(0, -0.2), (0, -0.5), (0.35, -0.5), (0.35, -0.2)])
        self.area_drive_back2 = Polygon([(0, 0.2), (0.35, 0.2), (0.35, 0.7), (0, 0.7)])
        # self.area_grab = Polygon([(0.25, -0.03), (0.3, -0.03), (0.3, 0.03), (0.25, 0.03)])

        self.execute_end_sequence()  # NOTE: can change some init variables

    def execute_end_sequence(self):
        """ Called before returning """
        self.subscriber_enabled = False
        self.bag_point_odom = None
        self.bag_point_base = None
        self.point_lock = PointLock(tolerance=0.2, count_to=6, fresh_time=7.0, fresh_name="Bag detection")

    def execute(self, userdata):
        pub_audio.publish("Samana bag found. Aligning with it")

        # Start object detection
        rospy.loginfo("Object detection STARTED")
        self.enable_detector_proxy(SetBoolRequest(True))

        self.subscriber_enabled = True

        counter_grab_area = 0
        self.point_lock.fresh_updated(no_audio=True)
        while True:
            if self.point_lock.is_fresh() is False:
                # Bag detection lost return fail
                self.visualize_bag_point(point=None)  # Clear bag pose visualization
                self.execute_end_sequence()
                return "fail"

            if self.bag_point_odom is None:
                sleep(0.1)
                continue

            # Update bag point in base frame from memory of odom frame point
            # bag_point = self.tf_point(self.bag_point_odom, self.base_frame, self.odom_frame)
            bag_point = self.tf_point(self.point_lock.get_locked_point(), self.base_frame, self.odom_frame)

            if bag_point is None:
                sleep(0.1)
                continue

            self.visualize_bag_point(bag_point, self.base_frame)

            p = Point(bag_point[0], bag_point[1])  # Shapely bag point

            if self.area_drive_back1.contains(p) or self.area_drive_back2.contains(p):
                self.drive(-0.05, 0)  # Drive backwards
            else:
                goal_reached = self.go_to_point(p, self.pos_goal, self.yaw_goal)  # Align

                # Check if goal reach for some time. If so break (return success)
                if goal_reached:
                    counter_grab_area += 1
                    if counter_grab_area > 10:  # TODO: tune
                        break
                else:
                    counter_grab_area = max(0, counter_grab_area - 2)

            self.cmd_vel_rate.sleep()

        # Stopped object detection
        rospy.loginfo("Object detection STOPPED")
        self.enable_detector_proxy(SetBoolRequest(False))

        self.execute_end_sequence()
        return "success"

    def go_to_point(self, point, pos_x_goal, yaw_goal):
        """
            Simple P controller with extra if statements
            :param point: shapely Point
            :param pos_x_goal: goal position x (forward)
            :param yaw_goal: goal yaw angle
            :return: True if goal is withing all tolerances
        """

        pos_error = point.x - pos_x_goal
        yaw_error = math.atan2(point.y, point.x) - yaw_goal
        # print("yaw_err: {:.2f}, pos_err: {:.2f}".format(math.degrees(yaw_error), pos_error))

        # P controller with clampping
        vel = 0
        rot = 0
        is_in_yaw_tol = abs(yaw_error) < self.yaw_tolerance
        is_in_pos_tol = abs(pos_error) < self.pos_tolerance
        is_far = pos_error > self.pos_far

        # Align angle if not in tolerance
        if not is_in_yaw_tol:
            rot = yaw_error * self.yaw_p
            rot = sign(rot) * clamp(abs(rot), self.yaw_min, self.yaw_max)

        # Align position (when yaw in tolerance or really far away)
        if (not is_in_pos_tol and is_in_yaw_tol) or is_far:
            vel = pos_error * self.pos_p
            vel = sign(vel) * clamp(abs(vel), self.pos_min, self.pos_max)

        if is_far:  # Speed up
            vel *= 1.5
            rot *= 1.5

        self.drive(vel, rot)

        if is_in_yaw_tol is True and is_in_pos_tol:
            return True
        return False

    def drive(self, vel, rot):
        """
        Sends velocity command to /cmd_vel topic
        :param vel: velocity forward m/s
        :param rot: yaw angular velocity rad/s
        """
        # print("drive: {:.3f} {:.3f}".format(vel, rot))

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = vel
        cmd_vel_msg.angular.z = rot
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def tf_point(self, point, frame_to, frame_from):
        """
        :param point: list [x, y]
        :return: list(point) [x y] transformed from frame_from to frame_to
        """
        try:
            self.tf_listener.waitForTransform(frame_to, frame_from, rospy.Time(0), rospy.Duration(4.0))

            new_point = PointStamped()
            new_point.header.frame_id = frame_from
            new_point.header.stamp = rospy.Time(0)
            new_point.point.x = point[0]
            new_point.point.y = point[1]
            new_point.point.z = 0.0
            new_point = self.tf_listener.transformPoint(frame_to, new_point)

        except Exception as e:
            rospy.logerr("Can't transform from {} to {} frame".format(frame_from, frame_to))
            rospy.logerr("{}".format(e))
            return None

        return [new_point.point.x, new_point.point.y]

    def visualize_bag_point(self, point, frame_id="odom"):
        # Publishing bag pose for visualization
        bag_pose_msg = PoseStamped()
        bag_pose_msg.header.stamp = rospy.Time.now()
        bag_pose_msg.header.frame_id = frame_id
        bag_pose_msg.pose.orientation.w = 1

        if point is not None:
            bag_pose_msg.pose.position.x = point[0]
            bag_pose_msg.pose.position.y = point[1]
        else:
            bag_pose_msg.pose.position.x = 99999  # Effectively clears pose (draws of screen)
            bag_pose_msg.pose.position.y = 99999
            for i in range(2):
                self.bag_pose_pub.publish(bag_pose_msg)

        self.bag_pose_pub.publish(bag_pose_msg)

    def bag_point_cb(self, data):
        """
            :param data: Bag PointStamped in world coords (x - forward, y - right)
        """
        if self.subscriber_enabled is False:
            return

        # Update bag base and odom points
        self.bag_point_base = [data.point.x, data.point.y]
        self.bag_point_odom = self.tf_point(self.bag_point_base, self.odom_frame, data.header.frame_id)

        self.point_lock.new_point(self.bag_point_odom)


class GrabBag(State):
    def __init__(self):
        State.__init__(self, outcomes=["success", "fail"])

        self.client = None
        self.action_server_started = False
        self.server_name = "arm_control"

        self.current_grabber = 0  # In mA
        self.current_lifter = 0  # In mA
        self.lifter_with_bag_current = 120  # Wihtout bag ~80mA, with bag ~160mA
        self.bag_verified = False
        self.enable_arm_cb = False
        self.new_arm_data = False

        # Publishers  (latch=True because it's params and because subscribers need time to connect to publisher)
        self.enable_front_sensors_pub = rospy.Publisher("params/enable_front_sensors", Bool, queue_size=1, latch=True)

        # Subscriber
        rospy.Subscriber("arm_data", ArmData, self.arm_data_callback)

        self.GOAL_STATES = ("PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED",
                            "REJECTED", "PREEMPTING", "RECALLING", "RECALLED", "LOST")

    def execute(self, userdata):
        pub_audio.publish("Samana grabbing the bag")

        self.bag_verified = False
        self.enable_front_sensors_pub.publish(Bool(False))
        pub_audio.publish("Disabled front sensors")

        self.action_server_started = False
        self.client = actionlib.SimpleActionClient(self.server_name, ArmControlAction)
        rospy.loginfo("Wait for {} server...".format(self.server_name))
        self.client.wait_for_server()

        rospy.loginfo("Sending goal to {} server".format(self.server_name))
        goal = ArmControlGoal()

        # NOTE: tuning
        A = ArmControlGoal
        arm_command_list = []
        arm_command_list += [A.GRABBER_OPEN_FULLY, ] * 5  # Open fully (to unstuck)
        arm_command_list += [A.OPEN_AND_LOWER, ] * 2  # Lower and open to the bag
        arm_command_list += [A.GRABBER_CLOSE_FULLY, ] * 4  # First close
        arm_command_list += ([A.LIFTER_RAISE_SMALL, ] * 2 + [A.GRABBER_CLOSE_FULLY, ] * 3) * 2  # Raise little and close better
        arm_command_list += ([A.LIFTER_RAISE_SMALL, ] + [A.GRABBER_CLOSE_FULLY, ] * 2) * 5  # Raise little and close better
        arm_command_list += [A.LIFTER_RAISE_FULLY, ] * 2  # Raise x2

        for i, cmd in enumerate(arm_command_list):
            goal.arm_command = cmd
            self.client.send_goal(goal, done_cb=self.done_cb, active_cb=self.active_cb)

            # Block until action server starts
            while self.action_server_started is False:
                sleep(0.05)

            # If second to last command (which must be LIFTER_RAISE_FULLY) verify bag by incresed current
            if i == len(arm_command_list) - 2:
                if cmd != A.LIFTER_RAISE_FULLY:
                    rospy.logerr("Raise command is not second to last")
                    time.sleep(7)

                # Enable data gather
                self.enable_arm_cb = True
                self.new_arm_data = False

                # Get average over some time
                lifter_current_list = []
                time.sleep(8.0)
                start_time = rospy.Time.now()
                while True:
                    if self.new_arm_data is True:
                        lifter_current_list.append(abs(self.current_lifter))
                        self.new_arm_data = False

                    if rospy.Time.now() - start_time > rospy.Duration(4.0):
                        break

                # Calculate average current
                avg_current = sum(lifter_current_list) / len(lifter_current_list)
                rospy.loginfo("Average lifter current: {}".format(avg_current))

                # Verify bag grab
                if avg_current > self.lifter_with_bag_current:
                    pub_audio.publish("Bag grabbed verified. {} miliamps".format(avg_current))
                    self.bag_verified = True
                else:
                    pub_audio.publish("Bag is not grabbed {} miliamps".format(avg_current))
                    self.bag_verified = False

                # Disable data gather
                self.enable_arm_cb = False

            # Blocking wait
            self.client.wait_for_result()

            goal_state = self.client.get_state()
            rospy.loginfo("{} finished state: {}".format(self.server_name, self.GOAL_STATES[goal_state]))

            if goal_state != actionlib.GoalStatus.SUCCEEDED:
                sleep(2.0)

        self.enable_front_sensors_pub.publish(Bool(True))
        pub_audio.publish("Enabled front sensors")

        if self.bag_verified is True:
            return "success"
        else:
            return "fail"

    def active_cb(self):
        rospy.loginfo("{} action server is processing the goal".format(self.server_name))
        self.action_server_started = True

    def done_cb(self, state, result):
        rospy.loginfo("{} action server is done. State: {}, Result: {}".format(self.server_name, str(state), str(result)))

    def arm_data_callback(self, msg):
        """Arm data callback function for reading arm currents"""
        if self.enable_arm_cb is True:
            self.new_arm_data = True
            self.current_grabber = msg.current_grabber
            self.current_lifter = msg.current_lifter


class DriveToHome(State):
    def __init__(self):
        State.__init__(self, outcomes=["success", "fail"])

        file_path_default = rospkg.RosPack().get_path("samana") + "/config/waypoints_to_home.csv"
        file_path = rospy.get_param("~file_wp_to_home", file_path_default)
        self.drive_to_goal = DriveToGoal(file_path=file_path, dist_tolerance=4.0, goal_frame_id="utm", waypoint_name="a home")

    def execute(self, userdata):
        pub_audio.publish("Samana driving back to the start area")
        return self.drive_to_goal.execute()


class Finished(State):
    def __init__(self):
        State.__init__(self, outcomes=["success"])

    def execute(self, userdata):
        pub_audio.publish("Samana start area reached or atleast I hope so")
        return "success"


class DriveToGoal():
    """
        Sends specified file with waypoints to follow_waypoints action_server
    """

    def __init__(self, file_path=None, waypoints=None, dist_tolerance=4.0, goal_frame_id="utm",
                 waypoint_name="", short_audio=False, feedback_client=None, last_wp=0):
        self.dist_tolerance = dist_tolerance
        self.goal_frame_id = goal_frame_id
        self.server_name = "follow_waypoints"
        self.last_wp = last_wp  # Counting from 0
        self.last_wp_addition = last_wp  # Constant used for pub_audio correct waypoint
        self.file_path = file_path
        self.waypoints = waypoints
        self.waypoint_name = waypoint_name
        self.short_audio = short_audio
        self.client = None
        self.feedback_client = feedback_client
        self.action_server_started = False

        self.wp_frame_id = rospy.get_param("~wp_frame_id", "utm")
        self.GOAL_STATES = ("PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED",
                            "REJECTED", "PREEMPTING", "RECALLING", "RECALLED", "LOST")

    def execute(self, blocking=True, wait_to_start=False):
        self.action_server_started = False
        self.client = actionlib.SimpleActionClient(self.server_name, FollowWpAction)
        rospy.loginfo("Wait for {} server...".format(self.server_name))
        self.client.wait_for_server()

        rospy.loginfo("Sending goal to follow_waypoints server")
        goal = FollowWpGoal()
        if self.waypoints:
            goal.waypoints = self.waypoints
            goal.waypoints.poses = goal.waypoints.poses[self.last_wp:]  # Continue from last waypoint
        else:
            goal.waypoints = self.read_waypoints_from_file(start_from=self.last_wp)  # Read from file

        goal.dist_tolerance = self.dist_tolerance  # To override distance tolerance

        self.client.send_goal(goal, done_cb=self.done_cb, active_cb=self.active_cb, feedback_cb=self.feedback_cb)

        # Block until action server starts
        if wait_to_start is True:
            while self.action_server_started is False:
                sleep(0.05)

        if blocking is True:
            self.client.wait_for_result()

            goal_state = self.client.get_state()
            rospy.loginfo("follow_waypoints finished state: {}".format(self.GOAL_STATES[goal_state]))

            if goal_state == actionlib.GoalStatus.SUCCEEDED:
                return "success"
            else:
                return "fail"

        return None

    def done_cb(self, state, result):
        rospy.loginfo("Action server is done. State: {}, result: {}".format(str(state), str(result)))

    def active_cb(self):
        rospy.loginfo("Action server is processing the goal")
        self.action_server_started = True

    def feedback_cb(self, feedback):
        if self.short_audio is True:
            pub_audio.publish("{} {}".format(self.waypoint_name, feedback.wp_current + self.last_wp_addition))
        else:
            pub_audio.publish("Samana following {} waypoint: {}".format(self.waypoint_name, feedback.wp_current + self.last_wp_addition))
        self.last_wp = feedback.wp_current

        if self.feedback_client:  # Send this feedback back to caller
            self.feedback_client(self.last_wp)

    def read_waypoints_from_file(self, start_from=0):
        waypoints = []
        with open(self.file_path, "r") as file:
            reader = csv.reader(file, delimiter=",")
            waypoints = PoseArray()
            waypoints.header.frame_id = self.wp_frame_id
            for i, row in enumerate(reader):
                if not row:
                    continue

                if i < start_from:
                    continue

                pose_tmp = Pose()
                pose_tmp.position.x = float(row[0])
                pose_tmp.position.y = float(row[1])
                pose_tmp.position.z = float(row[2])
                pose_tmp.orientation.x = float(row[3])
                pose_tmp.orientation.y = float(row[4])
                pose_tmp.orientation.z = float(row[5])
                pose_tmp.orientation.w = float(row[6])
                waypoints.poses.append(pose_tmp)

        return waypoints


class PathGenerator:
    def __init__(self, radius=5, pitch=0.5, segm_len_min=0.4, arc_ratio_min=0.1):
        # In meters NOTE: tuning
        self.radius = radius
        self.radius_min = 0.1  # Must be > 0
        self.pitch = pitch
        self.segment_len_min = segm_len_min  # Minimum segment length (mainly in the center)
        self.arc_ratio_min = arc_ratio_min  # Minimum arc ratio (mainly in outer spiral)
        self.phi_list = [0, ]
        self.r_list = [0, ]
        self.phi_array = None
        self.r_array = None

    def spiral(self, for_viz=False):
        """
          param for_viz: if True all segment length is fixed else fix by arc length
          return: (r_array, phi_array) in numpy arrays
        """

        # Spiral
        r = self.radius_min
        while r < self.radius:
            n = r / self.pitch  # Spiral leg number
            self.phi = -2 * math.pi * n  # Current angle in polar coordinates

            arc_ratio = self.segment_len_min / (2 * math.pi * r)  # Segment arc ratio with circle circumference
            if not for_viz:
                arc_ratio = max(arc_ratio, self.arc_ratio_min)  # Makes segments longer with bigger r

            delta_phi = arc_ratio * 2 * math.pi  # Angle increase with fixed arc length
            self.phi -= delta_phi

            # Converting angle back to radius
            n_new = -self.phi / (2 * math.pi)
            r = n_new * self.pitch

            self.r_list.append(r)
            self.phi_list.append(self.phi)

        self.phi_array = np.array(self.phi_list)
        self.r_array = np.array(self.r_list)

        return self.r_array, self.phi_array

    def xy_from_polar(self, r_array, phi_array, center_x, center_y):
        x = np.cos(phi_array) * r_array + float(center_x)
        y = np.sin(phi_array) * r_array + float(center_y)
        return x, y

    def print_path_dist(self, x, y, start_from):
        """
            Prints total path length
        """
        dist = 0.0
        for i in range(start_from, len(x) - 1):
            segment_dist = math.sqrt((x[i] - x[i+1])**2 + (y[i] - y[i+1])**2)
            dist += segment_dist

        # Hard codded velocity
        rospy.loginfo("Search path distance: {:.1f}m, at v: {:.2f}m/s t: {:.1f}min".format(dist, 0.3, dist/0.3/60.0))


class PointLock():
    """
    Groups nearby points and if there's enough points in a group locks it as real point
    and won't allow any point to be return from get_locked_point() locked point is still updated
    with nearby new points
    """

    def __init__(self, tolerance, count_to, fresh_time, fresh_name):
        self.point_list = []
        self.point_count = []  # How many point in this tolerance
        self.tolerance = tolerance
        self.count_to = count_to  # If found this amount of points nearby lock that point
        self.locked_point_index = None
        self.latest_point = None
        self.fresh_point = IsFresh(fresh_time, fresh_name)

    def new_point(self, point):
        """
        :param point: list [x, y]
        """
        def dist(p1, p2):
            return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

        def avg(p1, p2):
            return [(p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0]

        self.latest_point = point

        if self.point_list == []:
            self.point_list.append(point)
            self.point_count.append(1)
        else:
            found = False
            if self.locked_point_index is None:  # Still no lock find nearest point
                for i, p in enumerate(self.point_list):  # Find where to add count or append new point
                    print("p1 {}, p2 {}, dist {}, tol: {}".format(p, point, dist(p, point), self.tolerance))
                    if dist(p, point) <= self.tolerance:  # Withing tolerance
                        self.point_list[i] = avg(self.point_list[i], point)  # Average position of points
                        self.point_count[i] += 1  # Add count
                        found = True
                        break

                if found is False:  # Append new point because it's not close to others
                    self.point_list.append(point)
                    self.point_count.append(1)

                for i, c in enumerate(self.point_count):
                    if c >= self.count_to:  # Enough to lock point
                        self.locked_point_index = i  # Locking the point

            else:  # Already locked
                i = self.locked_point_index
                if dist(self.point_list[i], point) <= self.tolerance:  # Withing tolerance
                    self.point_list[i] = avg(self.point_list[i], point)  # Average position of points
                    self.point_count[i] += 1  # Add count
                    found = True

            # If good point found update freshness
            if found:
                self.fresh_point.updated()

            # print("list: {}".format(self.point_list))
            # print("count: {}".format(self.point_count))

    def get_locked_point(self):
        """ Return locked point or latest point """
        if self.locked_point_index is not None:
            return self.point_list[self.locked_point_index]

        return self.latest_point

    def fresh_updated(self, no_audio=False):
        self.fresh_point.updated(no_audio)

    def is_fresh(self):
        return self.fresh_point.is_fresh()


def main():
    signal.signal(signal.SIGINT, signal_handler)  # Needed for Ctrl+C to work
    rospy.init_node("main_state_machine")

    # Top level state machine
    sm_top = StateMachine(outcomes=["success", "fail", "restart"])

    with sm_top:
        # GRAB state
        StateMachine.add("GRAB_BAG", GrabBag(), transitions={"success": "FINISHED",
                                                             "fail": "GRAB_BAG"})
        StateMachine.add("FINISHED", Finished(), transitions={"success": "success"})

        # SEARCH and ALIGN
        # StateMachine.add("INIT", Init(), transitions={"success": "SEARCH_BAG"})
        # StateMachine.add("SEARCH_BAG", SearchBag(), transitions={"success": "ALIGN_WITH_BAG",
        #                                                          "fail": "SEARCH_BAG"})
        # StateMachine.add("ALIGN_WITH_BAG", AlignWithBag(), transitions={"success": "FINISHED",
        #                                                                 "fail": "SEARCH_BAG"})
        # StateMachine.add("FINISHED", Finished(), transitions={"success": "success"})

        # ALL states
        # StateMachine.add("INIT", Init(), transitions={"success": "CHECK"})
        # StateMachine.add("CHECK", Check(), transitions={"success": "DRIVE_TO_BAG",
        #                                                 "fail": "INIT"})
        # StateMachine.add("DRIVE_TO_BAG", DriveToBag(), transitions={"success": "SEARCH_BAG",
        #                                                             "fail": "DRIVE_TO_BAG"})
        # StateMachine.add("SEARCH_BAG", SearchBag(), transitions={"success": "ALIGN_WITH_BAG",
        #                                                          "fail": "DRIVE_TO_HOME"})
        # StateMachine.add("ALIGN_WITH_BAG", AlignWithBag(), transitions={"success": "GRAB_BAG",
        #                                                                 "fail": "SEARCH_BAG"})
        # StateMachine.add("GRAB_BAG", GrabBag(), transitions={"success": "DRIVE_TO_HOME",
        #                                                      "fail": "ALIGN_WITH_BAG"})
        # StateMachine.add("DRIVE_TO_HOME", DriveToHome(), transitions={"success": "FINISHED",
        #                                                               "fail": "DRIVE_TO_HOME"})
        # StateMachine.add("FINISHED", Finished(), transitions={"success": "success"})

    # sis = smach_ros.IntrospectionServer("main_state_machine", sm_top, "/SM_TOP")
    # sis.start()

    outcome = sm_top.execute()  # Starting state machine
    rospy.spin()

    # sis.stop()


if __name__ == "__main__":
    main()
