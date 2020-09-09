#! /usr/bin/env python

import rospy
import actionlib
from time import sleep
from samana_msgs.msg import FollowWpAction, FollowWpFeedback, FollowWpResult
from geometry_msgs.msg import PoseArray


class FollowWaypointServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("follow_waypoints", FollowWpAction, self.execute, auto_start=False)
        self.server.start()
        self.feedback = FollowWpFeedback()

    def execute(self, goal):
        # goal.waypoints, goal.dist_tolerance

        print("Following waypoints with goal\n{}".format(goal.waypoints.poses))

        for i, waypoint in enumerate(goal.waypoints.poses):
            if self.server.is_preempt_requested():
                print("PREEMPT follow_waypoints")
                self.server.set_preempted(self.feedback)
                return

            sleep(1.0)
            rospy.loginfo("Executing move_base goal to position ({:7.4f}, {:7.4f}) with tolerance {:.3}".format(
                           waypoint.position.x, waypoint.position.y, goal.dist_tolerance))

            self.feedback.wp_current = i
            self.server.publish_feedback(self.feedback)

        print("Done waypoints")
        self.server.set_succeeded(result=FollowWpResult(self.feedback))


if __name__ == "__main__":
    rospy.init_node("follow_waypoints_server")
    server = FollowWaypointServer()
    rospy.spin()
