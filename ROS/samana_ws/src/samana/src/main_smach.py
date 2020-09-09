#!/usr/bin/env python

"""
    Main Samana state machine
"""

from time import sleep
import rospkg
import rospy
import smach
from smach import State, StateMachine
import smach_ros
import actionlib
from samana_msgs.msg import FollowWpAction, FollowWpGoal
from geometry_msgs.msg import PoseArray, Pose
import csv


class Init(State):
    def __init__(self):
        State.__init__(self, outcomes=["success"])

    def execute(self, userdata):
        rospy.loginfo("Initializing Samana")
        sleep(1.0)
        rospy.loginfo("Samana Init Done")

        return "success"


class Check(State):
    def __init__(self):
        State.__init__(self, outcomes=["success", "fail"])

    def execute(self, userdata):
        rospy.loginfo("Samana Check")
        return "success"


class DriveToBag(State):
    def __init__(self):
        State.__init__(self, outcomes=["success", "fail"])

        file_path_default = rospkg.RosPack().get_path("samana") + "/config/waypoints_to_bag.csv"
        file_path = rospy.get_param("~file_wp_to_bag", file_path_default)
        self.drive_to_goal = DriveToGoal(file_path=file_path, dist_tolerance=4.0, goal_frame_id="utm")

    def execute(self, userdata):
        return self.drive_to_goal.execute()


class SearchBag(State):
    def __init__(self):
        State.__init__(self, outcomes=["success", "fail"])

    def execute(self, userdata):
        rospy.loginfo("SEARCHING FOR BAG")
        sleep(2)
        return "success"


class AlignWithBag(State):
    def __init__(self):
        State.__init__(self, outcomes=["success", "fail"])

    def execute(self, userdata):
        rospy.loginfo("ALIGNING WITH BAG")
        sleep(2)
        return "success"


class GrabBag(State):
    def __init__(self):
        State.__init__(self, outcomes=["success", "fail"])

    def execute(self, userdata):
        rospy.loginfo("GRABBING BAG")
        sleep(2)
        return "success"


class DriveToHome(State):
    def __init__(self):
        State.__init__(self, outcomes=["success", "fail"])

        file_path_default = rospkg.RosPack().get_path("samana") + "/config/waypoints_to_home.csv"
        file_path = rospy.get_param("~file_wp_to_home", file_path_default)
        self.drive_to_goal = DriveToGoal(file_path=file_path, dist_tolerance=4.0, goal_frame_id="utm")

    def execute(self, userdata):
        return self.drive_to_goal.execute()


class Finished(State):
    def __init__(self):
        State.__init__(self, outcomes=["success"])

    def execute(self, userdata):
        rospy.loginfo("Samana Finished")

        return "success"


class DriveToGoal():
    """
        Sends specified file with waypoints to follow_waypoints action_server
    """

    def __init__(self, file_path=None, dist_tolerance=4.0, goal_frame_id="utm"):
        self.dist_tolerance = dist_tolerance
        self.goal_frame_id = goal_frame_id
        self.server_name = "follow_waypoints"
        self.last_wp = 0  # Counting from 0
        self.file_path = file_path

        self.GOAL_ID = "utm"
        self.GOAL_STATES = ("PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED",
                            "REJECTED", "PREEMPTING", "RECALLING", "RECALLED", "LOST")

    def execute(self):
        client = actionlib.SimpleActionClient(self.server_name, FollowWpAction)
        rospy.loginfo("Wait for {} server...".format(self.server_name))
        client.wait_for_server()

        rospy.loginfo("Sending goal to follow_waypoints server")
        goal = FollowWpGoal()
        goal.waypoints = self.read_waypoints_from_file(start_from=self.last_wp)
        goal.dist_tolerance = 4.0

        client.send_goal(goal, done_cb=self.done_cb, active_cb=self.active_cb, feedback_cb=self.feedback_cb)
        client.wait_for_result()

        goal_state = client.get_state()
        rospy.loginfo("follow_waypoints finished state: {}".format(self.GOAL_STATES[goal_state]))

        if goal_state == actionlib.GoalStatus.SUCCEEDED:
            return "success"
        else:
            return "fail"

    def done_cb(self, state, result):
        rospy.loginfo("Action server is done. State: {}, result: {}".format(str(state), str(result)))

    def active_cb(self):
        rospy.loginfo("Action server is processing the goal")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback: {}".format(str(feedback)))
        self.last_wp = feedback.wp_current

    def read_waypoints_from_file(self, start_from=0):
        waypoints = []
        with open(self.file_path, "r") as file:
            reader = csv.reader(file, delimiter=",")
            waypoints = PoseArray()
            waypoints.header.frame_id = self.GOAL_ID
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


def main():
    rospy.init_node("main_state_machine")

    # Top level state machine
    sm_top = StateMachine(outcomes=["success", "fail", "restart"])

    with sm_top:
        StateMachine.add("INIT", Init(), transitions={"success": "CHECK"})
        StateMachine.add("CHECK", Check(), transitions={"success": "DRIVE_TO_BAG",
                                                        "fail": "INIT"})
        StateMachine.add("DRIVE_TO_BAG", DriveToBag(), transitions={"success": "SEARCH_BAG",
                                                                     "fail": "DRIVE_TO_BAG"})
        StateMachine.add("SEARCH_BAG", SearchBag(), transitions={"success": "DRIVE_TO_HOME",
                                                                 "fail": "SEARCH_BAG"})
        StateMachine.add("ALIGN_WITH_BAG", AlignWithBag(), transitions={"success": "GRAB_BAG",
                                                                        "fail": "ALIGN_WITH_BAG"})
        StateMachine.add("GRAB_BAG", GrabBag(), transitions={"success": "DRIVE_TO_HOME",
                                                                  "fail": "ALIGN_WITH_BAG"})
        StateMachine.add("DRIVE_TO_HOME", DriveToHome(), transitions={"success": "FINISHED",
                                                                    "fail": "DRIVE_TO_HOME"})
        StateMachine.add("FINISHED", Finished(), transitions={"success": "success"})

    sis = smach_ros.IntrospectionServer("main_state_machine", sm_top, "/SM_TOP")  # For viewing state machine
    sis.start()

    outcome = sm_top.execute()  # Starting state machine

    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
