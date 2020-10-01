#!/usr/bin/env python
# Modified code from https://github.com/danielsnider/follow_waypoints/blob/master/src/follow_waypoints/follow_waypoints.py
"""
NOTES:
    - waypoints could be in same format like PoseArray for consistency or list of PoseStamped
    for flexability to specify every waypoint frame_id
    - Actionlib could have better implementation if waypoints data type was changed and overall
    - Need a lot of refactoring
"""

import threading
import rospy
import actionlib
import smach_ros
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Point, PoseWithCovarianceStamped
from std_msgs.msg import Empty
from samana_msgs.msg import FollowWpAction, FollowWpFeedback, FollowWpResult
from std_srvs import srv
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from visualization_msgs.msg import Marker
from tf import TransformListener
from copy import deepcopy
import tf
import math
import rospkg
import csv
import time

waypoints = []  # Global waypoints used for path following (PoseStamped list)
waypoints_action = []  # Action goemal msg later converted to global waypoints
distance_tolerance_action = 0.0  # Updated by action client call


class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])

        self.goal_frame_id = rospy.get_param('~goal_frame_id', 'map')
        self.follow_back = rospy.get_param('~follow_back', 'false')
        self.rotate_going_back = rospy.get_param('~rotate_going_back', 'false')
        self.pose_topic = rospy.get_param('~pose_topic', '/initialpose')
        self.always_from_file = rospy.get_param('~always_from_file', 'false')
        self.use_action_lib = rospy.get_param('~use_action_lib', 'false')

        self.file_path_default = rospkg.RosPack().get_path('samana') + '/config/waypoints.csv'
        self.file_path = rospy.get_param('~file_path', self.file_path_default)

        # Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.poseArray_publisher = rospy.Publisher('/waypoints', PoseArray, queue_size=1)
        self.waypoints_viz = WaypointsViz()  # For visualizing using MarkerArray

        def wait_for_path_reset():
            """thread worker function"""
            global waypoints

            while not rospy.is_shutdown():
                data = rospy.wait_for_message('/path_reset', Empty)
                rospy.loginfo('Received path RESET message')
                self.clear_path_queue()
                rospy.sleep(3)  # Wait 3 seconds because `rostopic echo` latches
                # for three seconds and wait_for_message() in a
                # loop will see it again.

        # Start thread to listen for reset messages to clear the waypoint queue
        reset_thread = threading.Thread(target=wait_for_path_reset)
        reset_thread.start()

    def visualize_path(self, waypoints):
        # Publish empty waypoint queue as pose array so that you can see them the change in rviz, etc.
        self.poseArray_publisher.publish(convert_PoseStamped_to_PoseArray(waypoints))

        # Publish markers array for visualization in rviz and mapviz
        self.waypoints_viz.visualize_waypoints(waypoints)

    def clear_path_queue(self):
        global waypoints

        waypoints = []  # The waypoint queue
        self.visualize_path(waypoints)

    def read_waypoints_from_file(self):
        global waypoints

        rospy.loginfo("Reading waypoints from: {}".format(self.file_path))
        with open(self.file_path, 'r') as file:
            reader = csv.reader(file, delimiter=',')
            for row in reader:
                if not row:
                    continue

                current_pose = PoseStamped()
                current_pose.header.frame_id = self.goal_frame_id
                current_pose.pose.position.x = float(row[0])
                current_pose.pose.position.y = float(row[1])
                current_pose.pose.position.z = float(row[2])
                current_pose.pose.orientation.x = float(row[3])
                current_pose.pose.orientation.y = float(row[4])
                current_pose.pose.orientation.z = float(row[5])
                current_pose.pose.orientation.w = float(row[6])
                waypoints.append(current_pose)

        self.append_follow_back_waypoints()

    def append_follow_back_waypoints(self):
        """
            Appends waypoints in reverse order and also rotates them if specified
        """
        global waypoints

        if self.follow_back is True:
            if self.rotate_going_back is False:
                waypoints += waypoints[::-1]  # Append reversed waypoints
            else:
                waypoints_rev = deepcopy(waypoints)[::-1]
                for i in range(len(waypoints_rev)):
                    # Rotate every quaternion 180deg on yaw axis
                    o = waypoints_rev[i].pose.orientation
                    yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2] + math.pi
                    quat = quaternion_from_euler(0, 0, yaw)
                    waypoints_rev[i].pose.orientation.x = quat[0]
                    waypoints_rev[i].pose.orientation.y = quat[1]
                    waypoints_rev[i].pose.orientation.z = quat[2]
                    waypoints_rev[i].pose.orientation.w = quat[3]
                waypoints += waypoints_rev  # Append waypoint for follow back

    def PoseWithCov_to_PoseStamped(self, waypoint):
        pose = PoseStamped()
        pose.header.frame_id = waypoint.header.frame_id
        pose.pose = waypoint.pose.pose
        return pose

    def execute(self, userdata):
        global waypoints

        self.clear_path_queue()
        self.path_ready = False

        def start_journey():
            self.read_waypoints_from_file()

            print_waypoints_info(waypoints)
            self.visualize_path(waypoints)

            self.start_journey_bool = True

        if self.always_from_file is True or self.use_action_lib is True:  # Read waypoint from the file without user input or by action lib input
            start_journey()
        else:  # Let user choose between new waypoints or reading from a file
            def wait_for_path_ready():
                """
                    Thread worker function
                    Waits for msg. When received outputs waypoints to a file
                """

                data = rospy.wait_for_message('/path_ready', Empty)
                rospy.loginfo('Received path READY message')
                self.path_ready = True

                with open(self.file_path, 'w') as file:
                    for current_pose in waypoints:
                        p = current_pose.pose
                        file.write(str(p.position.x) + ',' + str(p.position.y) + ',' + str(p.position.z) + ',' + str(p.orientation.x) + ',' +
                                   str(p.orientation.y) + ',' + str(p.orientation.z) + ',' + str(p.orientation.w) + '\n')

                rospy.loginfo('poses written to ' + self.file_path)

            # Start thread to listen for when the path is ready (this function will end then)
            # Also will save the clicked path to specified path
            ready_thread = threading.Thread(target=wait_for_path_ready)
            ready_thread.start()

            self.start_journey_bool = False

            def wait_for_start_journey():
                """
                    Thread worker function
                    Waits for msg. When received reads waypoints from a file
                """
                data_from_start_journey = rospy.wait_for_message('start_journey', Empty)
                rospy.loginfo('Received path READY start_journey')

                start_journey()

            # Start thread to listen start_jorney
            # For loading the saved poses from to specified path
            start_journey_thread = threading.Thread(target=wait_for_start_journey)
            start_journey_thread.start()

            rospy.loginfo("Waiting to receive waypoints via Pose msg on topic {}".format(self.pose_topic))
            rospy.loginfo("To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1'")
            rospy.loginfo("OR")
            rospy.loginfo("To start following saved waypoints: 'rostopic pub /start_journey std_msgs/Empty -1'")

            # Wait for published waypoints or saved path loaded
            while (not self.path_ready and not self.start_journey_bool):
                try:
                    pose = rospy.wait_for_message(self.pose_topic, PoseWithCovarianceStamped, timeout=2.5)
                    pose = self.PoseWithCov_to_PoseStamped(pose)
                except rospy.ROSException as e:
                    if 'timeout exceeded' in e.message:
                        continue  # no new waypoint within timeout, looping...
                    else:
                        raise e

                waypoints.append(tf_pose(pose, self.goal_frame_id))
                self.waypoints_viz.visualize_current_waypoint(waypoints[-1])

                # Log info about new waypoint
                p = waypoints[-1].pose
                yaw = euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])[2]
                rospy.loginfo("Recieved new waypoint: frame:{} x:{} y:{} yaw:{}".format(
                    waypoints[-1].header.frame_id, p.position.x, p.position.y, math.degrees(yaw)))

        self.visualize_path(waypoints)

        # Path is ready! return success and move on to the next state (FOLLOW_PATH)
        return 'success'


class FollowPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])

        # Parameters
        self.goal_frame_id = rospy.get_param('~goal_frame_id', 'utm')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        self.duration = rospy.get_param('~wait_duration', 0.0)
        self.distance_tolerance = rospy.get_param('~waypoint_distance_tolerance', 0.0)
        self.check_dist_period = rospy.get_param('~check_dist_period', 0.33)
        self.use_action_lib = rospy.get_param('~use_action_lib', 'false')

        self.use_timeout = rospy.get_param('~use_timeout', 'false')
        self.timeout_a = rospy.get_param('~timeout_a', 2.1)
        self.timeout_b = rospy.get_param('~timeout_b', 4)
        self.timeout_vel = rospy.get_param('~timeout_vel', 0.3)

        self.last_action_move_base = False  # True if last action call was to move_base
        
        self.waypoints_viz = WaypointsViz()

        # Action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.move_base_client.wait_for_server()
        rospy.loginfo('Connected to move_base.')

        self.bug_alg_client = actionlib.SimpleActionClient('bug_algorithm', MoveBaseAction)
        rospy.loginfo('Connecting to bug_algorithm...')
        self.bug_alg_client.wait_for_server()
        rospy.loginfo('Connected to bug_algorithm.')

        # Service
        service_name = "move_base/clear_costmaps"
        rospy.loginfo("Waiting for {} service".format(service_name))
        rospy.wait_for_service(service_name)
        self.clear_costmaps_srv = rospy.ServiceProxy(service_name, srv.Empty)

        # Tf listener
        rospy.loginfo('Starting a tf listner.')
        self.listener = tf.TransformListener()


    def parameters_set_for_actionlib(self):
        # Action parameters override
        if self.use_action_lib is True:
            self.distance_tolerance = distance_tolerance_action

    def clear_costmaps(self, do_clearing=True):
        if do_clearing is True:
            try:
                rospy.logwarn("Clearing all costmaps")
                response = self.clear_costmaps_srv(srv.EmptyRequest())
                rospy.logwarn("Cleared all costmaps: {}".format(response))
            except rospy.ServiceException as e:
                print("Clear costmap service did not process request: " + str(e))

    def execute(self, userdata, feedback=None, action_server=None, clear_costmaps_on_wp=False):
        """
            Used as state machine state or by call from actionlib server
            param feedback: should be callback function cb(waypoint_current) which is called every new goal
                            arg waypoint_current: currently excecuting waypoint
            param action_server: action server object which called this function used to return state
        """
        global waypoints

        self.parameters_set_for_actionlib()

        # Execute waypoints each in sequence
        for i, waypoint in enumerate(waypoints):
            if waypoints == []:
                rospy.loginfo('The waypoint queue has been reset.')
                break

            self.waypoints_viz.visualize_current_waypoint(waypoint)

            # Clear costmap before executing next waypoint 
            self.clear_costmaps(clear_costmaps_on_wp)

            # Publish next waypoint as goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = waypoint.header.frame_id
            goal.target_pose.pose.position = waypoint.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.orientation

            rospy.loginfo("Executing move_base goal #{} to position ({:7.4f}, {:7.4f})".format(
                i, waypoint.pose.position.x, waypoint.pose.position.y))
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")

            # Start with move_base
            self.move_base_client.send_goal(goal)
            self.last_action_move_base = True

            # Start with bug algorithm
            # self.bug_alg_client.send_goal(goal)
            # self.last_action_move_base = False

            # Waypoint execution started call feedback function and pass waypoint number
            if feedback:
                feedback(i)

            if not self.distance_tolerance > 0.0:
                self.move_base_client.wait_for_result()  # Waits for server to finish action
                rospy.loginfo("Waiting for %f sec..." % self.duration)
                time.sleep(self.duration)
            else:  # If distance_tolerance is set
                start_time = rospy.Time.now()  # Used for timeout
                dist_to_wp = self.dist_to_waypoint(waypoint)
                timeout = self.timeout_a * (dist_to_wp / self.timeout_vel) + self.timeout_b
                rospy.loginfo("Waypoint dist: {:.2f}, timeout: {:.2f}".format(self.dist_to_waypoint(waypoint), timeout))

                distance = self.distance_tolerance + 1  # distace > distance_tolerance to always enter the while loop
                while True:  # Loops until waypoint is reached or timed-out
                    # Skip this waypoint because timeout
                    if rospy.Time.now() - start_time > rospy.Duration(timeout):
                        rospy.logwarn("Waypoint TIMEOUT! Couldn't reach goal in {:.2f}sec {:.2f}m away".format(timeout, dist_to_wp))
                        break

                    # Waypoint reached by move_base continue with next waypoint
                    if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                        break
                    
                    if self.bug_alg_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                        break

                    # Follow waypoints tolerance satisfied continue with next waypoint
                    if self.dist_to_waypoint(waypoint) < self.distance_tolerance:
                        break
                    
                    # Check if move_base finished before completing action if so clear costmap and repeat goal
                    gs = actionlib.GoalStatus
                    action_failed = (gs.PREEMPTED, gs.ABORTED, gs.REJECTED, gs.RECALLED, gs.LOST)
                    if (self.last_action_move_base is True and self.move_base_client.get_state() in action_failed) or \
                       (self.last_action_move_base is False and self.bug_alg_client.get_state() in action_failed):
                        # After move_base or bug_algorithm failed. Here are custom recovery behaviours
                        if self.last_action_move_base is True:  # Choose bug algorithm every second time
                            self.clear_costmaps()
                            self.bug_alg_client.send_goal(goal)
                            self.last_action_move_base = False
                        else:
                            self.move_base_client.send_goal(goal)  # Resend same goal
                            self.last_action_move_base = True
                    
                    # Preempt requested (only when using actionlib).
                    # Send current waypoint index as result and stop move_base and return
                    if action_server:
                        if action_server.is_preempt_requested():
                            print("PREEMPTING follow_waypoints")
                            self.move_base_client.cancel_all_goals()
                            rospy.loginfo("CANCELED move_base")
                            self.bug_alg_client.cancel_all_goals()
                            rospy.loginfo("CANCELED bug_algorithm")
                            
                            action_server.set_preempted(result=FollowWpResult(i))

                            self.waypoints_viz.visualize_current_waypoint(waypoint, done=True)

                            return 'preempt'

                    time.sleep(self.check_dist_period)  # Reduces CPU usage

                    # rospy.loginfo_throttle(1, "Distance to goal point: {}".format(distance))  # For debuging
                    # rospy.loginfo("Distance to goal point: {}".format(distance))  # For debuging

            self.waypoints_viz.visualize_current_waypoint(waypoint, done=True)
        
        action_server.set_succeeded(result=FollowWpResult(len(waypoints)))  # All waypoints done
        return 'success'

    def dist_to_waypoint(self, waypoint):
        self.listener.waitForTransform(waypoint.header.frame_id, self.base_frame_id, rospy.Time(0), rospy.Duration(4.0))
        trans, rot = self.listener.lookupTransform(waypoint.header.frame_id, self.base_frame_id, rospy.Time(0))
        distance = math.sqrt(pow(waypoint.pose.position.x-trans[0], 2) + pow(waypoint.pose.position.y-trans[1], 2))

        return distance


class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        # rospy.loginfo('###############################')
        rospy.loginfo('##### FOLLOW WAYPOINTS REACHED FINISH GATE #####')
        # rospy.loginfo('###############################')

        return 'success'


def convert_PoseStamped_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id', 'map')
    poses.poses = [pose.pose for pose in waypoints]
    return poses


def print_waypoints_info(waypoints):
    rospy.loginfo("All waypoints list:")
    for i, waypoint in enumerate(waypoints):
        o = waypoint.pose.orientation
        yaw = math.degrees(euler_from_quaternion([o.x, o.y, o.z, o.w])[2])
        rospy.loginfo("#{}. x:{:.6f} y:{:.6f} | yaw.deg:{:3.5f} | frame_id: {}".format(
                      i, waypoint.pose.position.x, waypoint.pose.position.y, yaw, waypoint.header.frame_id))
    rospy.loginfo("Waypoints count: {}".format(len(waypoints)))


def tf_pose(waypoint, to_frame):
    """Transform PoseStamped to the correct frame"""
    if waypoint.header.frame_id == to_frame:
        return waypoint  # Already in correct frame

    if not hasattr(tf_pose, 'listener'):
        tf_pose.listener = tf.TransformListener()

    tmp = PoseStamped()
    tmp.header.frame_id = waypoint.header.frame_id
    tmp.pose = waypoint.pose
    try:
        tf_pose.listener.waitForTransform(to_frame, tmp.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
        pose = tf_pose.listener.transformPose(to_frame, tmp)
        ret = PoseStamped()
        ret.header.frame_id = to_frame
        ret.pose = pose.pose
        return ret
    except:
        rospy.logwarn("CAN'T TRANSFORM POSE TO {} FRAME".format(to_frame))
        exit()


class WaypointsViz:
    def __init__(self):
        self.init_markers()

        self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=1)

    def init_markers(self):
        # Set up our waypoint markers
        self.MARKER_ID_PATH = 0
        self.MARKER_ID_CURRENT = 1
        self.marker_scale = 2.5
        self.marker_color = {'r': 1.0, 'g': 0., 'b': 0.0, 'a': 0.4}
        marker_lifetime = 0  # 0 is forever
        marker_ns = 'waypoints'

        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=2)

        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = self.MARKER_ID_PATH
        self.markers.type = Marker.LINE_STRIP
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = self.marker_scale
        self.markers.scale.y = self.marker_scale
        self.markers.scale.z = self.marker_scale
        self.markers.color.r = self.marker_color['r']
        self.markers.color.g = self.marker_color['g']
        self.markers.color.b = self.marker_color['b']
        self.markers.color.a = self.marker_color['a']

        # self.markers.header.frame_id = rospy.get_param('~goal_frame_id', 'map')
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = []

    def set_color(self, r, g, b, a):
        self.markers.color.r = r
        self.markers.color.g = g
        self.markers.color.b = b
        self.markers.color.a = a

    def visualize_waypoints(self, waypoints):
        """
            param waypoints: (list of PoseStamped)
        """
        self.markers.points = []
        self.markers.id = self.MARKER_ID_PATH

        if waypoints:
            self.markers.header.frame_id = waypoints[0].header.frame_id
            self.markers.type = Marker.LINE_STRIP
            self.markers.scale.x = self.marker_scale
            self.set_color(self.marker_color['r'], self.marker_color['g'], self.marker_color['b'], self.marker_color['a'])
            for waypoint in waypoints:
                self.markers.points.append(waypoint.pose.position)

        if len(self.markers.points) > 0:
            self.marker_pub.publish(self.markers)
        else:
            rospy.loginfo('No waypoints. Clearing markers')
            self.marker_pub.publish(self.markers)

    def visualize_current_waypoint(self, waypoint, done=False):
        """
            param waypoints: (PoseStamped)
        """

        if waypoint:
            self.markers.header.frame_id = waypoints[0].header.frame_id
            self.markers.type = Marker.POINTS
            self.markers.id = self.MARKER_ID_CURRENT
            self.markers.scale.x = self.marker_scale * 15
            self.markers.scale.y = self.marker_scale * 15
            self.markers.points = []

            if done is True:
                self.set_color(0, 1.0, 0, self.marker_color['a'])
            else:
                self.set_color(self.marker_color['r'], self.marker_color['g'], self.marker_color['b'], self.marker_color['a'])

            self.markers.points.append(waypoint.pose.position)

        if len(self.markers.points) > 0:
            self.marker_pub.publish(self.markers)  # Will override previuos marker because same ID


class FollowWaypointServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("follow_waypoints", FollowWpAction, self.execute, auto_start=False)
        self.server.start()
        self.feedback = FollowWpFeedback()

    def execute(self, goal):
        global waypoints, waypoints_action, distance_tolerance_action

        def feedback_cb(wp_current):
            """Feedback callback from waypoints execution"""
            self.feedback.wp_current = wp_current
            self.server.publish_feedback(self.feedback)

        # rospy.loginfo("Following waypoints\n:{}".format(goal))  # Big output
        rospy.loginfo("Following waypoints count: {}".format(len(waypoints)))

        # NOTE: It's a mess with these global variables definitely needs refactoring
        follow_path = FollowPath()
        distance_tolerance_action = goal.dist_tolerance
        waypoints = PoseArray_to_PoseStamped_list(goal.waypoints)  # Global variable set for further use

        print_waypoints_info(waypoints)
        waypoints_viz = WaypointsViz()  # For visualizing using MarkerArray
        waypoints_viz.visualize_waypoints(waypoints)

        status = follow_path.execute(None, feedback_cb, self.server, goal.clear_costmaps_on_wp)

        waypoints_viz.visualize_waypoints(None)  # Clearing markers

        # rospy.loginfo("Following waypoints done with outcome: {}".format(outcome))
        # self.server.set_succeeded(result=FollowWpResult(self.feedback.wp_current))  # Already set


def PoseArray_to_PoseStamped_list(pose_array):
    pose_stamped_list = []
    for pose in pose_array.poses:
        new_pose = PoseStamped()
        new_pose.header.frame_id = pose_array.header.frame_id
        new_pose.pose = pose
        pose_stamped_list.append(new_pose)

    return pose_stamped_list


def create_state_machine():
    global sm_follow_waypoints
    sm_follow_waypoints = StateMachine(outcomes=['success'])

    with sm_follow_waypoints:
        StateMachine.add('GET_PATH', GetPath(),
                         transitions={'success': 'FOLLOW_PATH'},
                         remapping={'waypoints': 'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                         transitions={'success': 'PATH_COMPLETE'},
                         remapping={'waypoints': 'waypoints'})
        StateMachine.add('PATH_COMPLETE', PathComplete(),
                         transitions={'success': 'success'})


def start_state_machine():
    """
        Runs follow_waypoints state machine called by main() or by action client
    """
    visualize_smach = rospy.get_param('~visualize_smach', 'false')

    sis = None
    if visualize_smach is True:
        sis = smach_ros.IntrospectionServer('server_name', sm_follow_waypoints, 'SM_TOP/SM_FOLLOW')
        sis.start()

    outcome = sm_follow_waypoints.execute()

    if visualize_smach is True:
        # rospy.spin()
        sis.stop()

    return outcome


rospy.init_node('follow_waypointss')
create_state_machine()


def main():
    start_state_machine()


if __name__ == "__main__":
    if rospy.get_param('~use_action_lib', 'false'):
        follow_waypoits_server = FollowWaypointServer()
    else:
        main()
