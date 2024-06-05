import rospy
import actionlib

from ..designator import ObjectDesignatorDescription
from ..pose import Pose
from ..local_transformer import LocalTransformer
from ..bullet_world import BulletWorld
from ..enums import ObjectType
from typing import Any
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal


class PoseNavigator:
    """
    A class to handle navigation to a specified pose using the move_base service in a ROS environment.
    """

    def __init__(self):
        """Initializes the PoseNavigator with an action client for move_base."""
        self.client = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)
        self.query_result = None

    def query_pose_nav(self, navpose):
        """
        Sends a goal to the move_base service, moving the robot to the specified pose.

        :param navpose: PoseStamped, the target pose for the robot to navigate to.
        :return: Result of the navigation query.
        """

        def active_callback():
            rospy.loginfo("Send query to Move base")

        def done_callback(state, result):
            rospy.loginfo("Finished moving")
            self.query_result = result

        def feedback_callback(msg):
            pass

        goal_msg = MoveBaseGoal()
        goal_msg.target_pose = navpose

        rospy.loginfo("Navigating to the target pose")
        rospy.loginfo("Waiting for action server")
        self.client.wait_for_server()
        self.client.send_goal(goal_msg, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
        self.client.wait_for_result()

        return self.query_result

def queryPoseNav(navpose):
    """
    Sends a goal to the move_base Service. Connection to Giskard interface,
    will move robot to the given pose: navpose
    :param navpose: PoseStamped pose robot will navigate to
    """

    global query_result

    def active_callback():
        rospy.loginfo("Send query to Move base")

    def done_callback(state, result):
        rospy.loginfo("Finished moving")
        global query_result
        query_result = result

    def feedback_callback(msg):
        pass

    goal_msg = MoveBaseGoal()

    goal_msg.target_pose = navpose
    rospy.loginfo("navigating")
    client = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)
    rospy.loginfo("Waiting for action server nav")
    client.wait_for_server()
    client.send_goal(goal_msg, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
    client.wait_for_result()

    return query_result