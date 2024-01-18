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


def queryPoseNav(navpose):
    """
    Sends a query to RoboKudo to look for an object that fits the description given by the Object designator description.
    For sending the query to RoboKudo a simple action client will be created and the Object designator description is
    sent as a goal.

    :param object_desc: The object designator description which describes the object that should be perceived
    :return: An object designator for the found object, if there was an object that fitted the description.
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
    print(goal_msg)
    print("navigating")
    client = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)
    print(client)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(goal_msg, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
    client.wait_for_result()

    return query_result
