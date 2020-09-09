#! /usr/bin/env python

import rospy
import actionlib
from samana_msgs.msg import FollowWpAction, FollowWpGoal
from geometry_msgs.msg import PoseArray, Pose
import csv
import rospkg
from time import sleep

def done_cb(state, result):
    rospy.loginfo("Action server is done. State: {}, result: {}".format(str(state), str(result)))

def active_cb():
    rospy.loginfo("Action server is processing the goal")


def feedback_cb(feedback):
    rospy.loginfo("Feedback: {}".format(str(feedback)))

def read_waypoints_from_file():
        waypoints = []
        file_path = rospkg.RosPack().get_path("samana") + "/config/waypoints_to_goal.csv"

        with open(file_path, "r") as file:
            reader = csv.reader(file, delimiter=",")
            waypoints = PoseArray()
            waypoints.header.frame_id = rospy.get_param("follow_waypoints/goal_frame_id", "utm")
            for row in reader:
                if not row:
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


if __name__ == "__main__":
    server_name = "follow_waypoints"
    rospy.init_node("follow_waypoints_client")
    goal_states_text = ("PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED", "REJECTED", "PREEMPTING", "RECALLING", "RECALLED", "LOST")
    client = actionlib.SimpleActionClient(server_name, FollowWpAction)

    rospy.loginfo("Wait for {} server...".format(server_name))
    client.wait_for_server()


    goal = FollowWpGoal()
    goal.waypoints = read_waypoints_from_file()
    goal.dist_tolerance = 4.0
    rospy.loginfo("Sending goal to follow_waypoints server. Count: {}".format(len(goal.waypoints.poses)))
    client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)

    

    # client.wait_for_result()
    state = client.get_state()
    while state == actionlib.GoalStatus.ACTIVE or state == actionlib.GoalStatus.PENDING:
        # rospy.loginfo("State: {}".format(state))
        state = client.get_state()
        sleep(0.5)
        sleep(2)
        client.cancel_goal()

    result = client.get_result()
    rospy.loginfo("Result: {}, State: {}".format(result, goal_states_text[state]))
    
    rospy.loginfo("follow_waypoints finished state: {}".format(goal_states_text[client.get_state()]))
