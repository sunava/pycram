import sys

from geometry_msgs.msg import PoseStamped
from typing_extensions import Callable

import rospy
import actionlib
import rosnode

from ..designator import ObjectDesignatorDescription
from pycram.datastructures.pose import Pose
from pycram.local_transformer import LocalTransformer
from pycram.world import World
from pycram.datastructures.enums import ObjectType
from ..plan_failures import PerceptionLowLevelFailure

try:
    from robokudo_msgs.msg import ObjectDesignator as robokudo_ObjetDesignator
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult
except ModuleNotFoundError as e:
    rospy.logwarn(f"Could not import RoboKudo messages, RoboKudo interface could not be initialized")

robokudo_action_client = None


def init_robokudo_interface(func: Callable) -> Callable:
    """
    Tries to import the RoboKudo messages and with that initialize the RoboKudo interface.
    """
    def wrapper(*args, **kwargs):
        global robokudo_action_client
        topics = list(map(lambda x: x[0], rospy.get_published_topics()))
        if "robokudo_msgs" not in sys.modules:
            rospy.logwarn("Could not initialize the RoboKudo interface since the robokudo_msgs are not imported")
            return

        if "/robokudo" in rosnode.get_node_names():
            robokudo_action_client = create_robokudo_action_client()
            rospy.loginfo("Successfully initialized robokudo interface")
        else:
            rospy.logwarn("RoboKudo is not running, could not initialize RoboKudo interface")
            return

        return func(*args, **kwargs)
    return wrapper


def create_robokudo_action_client() -> Callable:
    """
    Creates a new action client for the RoboKudo query interface and returns a function encapsulating the action
    client. The returned function can be called with a technique and/or type as parameter and returns the result of
    the action client.

    :return: A callable function encapsulating the action client
    """
    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()

    def action_client(object_desc):
        global query_result

        def active_callback():
            rospy.loginfo("Send query to Robokudo")

        def done_callback(state, result):
            rospy.loginfo("Finished perceiving")
            global query_result
            query_result = result

        def feedback_callback(msg):
            pass

        object_goal = make_query_goal_msg(object_desc)
        client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
        wait = client.wait_for_result()
        return query_result

    return action_client


def msg_from_obj_desig(obj_desc: ObjectDesignatorDescription) -> 'robokudo_ObjetDesignator':
    """
    Creates a RoboKudo Object designator from a PyCRAM Object Designator description

    :param obj_desc: The PyCRAM Object designator that should be converted
    :return: The RobotKudo Object Designator for the given PyCRAM designator
    """
    obj_msg = robokudo_ObjetDesignator()
    obj_msg.uid = str(id(obj_desc))
    obj_msg.obj_type = obj_desc.types[0] # For testing purposes

    return obj_msg


def make_query_goal_msg(technique, object_type=None, object_color=None) -> 'QueryGoal':
    """
    Generates a QueryGoal message from a specific technique that describes what needs to be perceived for querying
    RobotKudo. The default message is empty and gets filled with the given parameters. However, if the technique is
    not 'human', 'color', or 'type', it will result in a broader perception where all objects are considered.

    :param technique: The technique that should be used for the query, e.g. type, color, human
    :param object_type: The object type that should be perceived
    :param object_color: The object color that should be perceived
    :return: The RoboKudo QueryGoal message
    """
    goal_msg = QueryGoal()
    if technique == "type":
        if not object_type:
            raise PerceptionLowLevelFailure(
                f"For technique='Type' the object_type must be set, but is {object_type}")

        goal_msg.obj.obj_type = str(object_type)

    elif technique == "color":
        if not object_color:
            raise PerceptionLowLevelFailure(
                f"For technique='color' the object_color must be set, but is {object_color}")
        goal_msg.obj.color.append(str(object_color))

    elif technique == 'human':
        goal_msg.obj.obj_type = "human"

    return goal_msg


@init_robokudo_interface
def query(object_desc: ObjectDesignatorDescription) -> ObjectDesignatorDescription.Object:
    """
    Sends a query to RoboKudo to look for an object that fits the description given by the Object designator description.
    For sending the query to RoboKudo a simple action client will be created and the Object designator description is
    sent as a goal.

    :param object_desc: The object designator description which describes the object that should be perceived
    :return: An object designator for the found object, if there was an object that fitted the description.
    """
    query_result = robokudo_action_client(object_desc)
    pose_candidates = {}
    if query_result.res == []:
        rospy.logwarn("No suitable object could be found")
        return

    for i in range(0, len(query_result.res[0].pose)):
        pose = Pose.from_pose_stamped(query_result.res[0].pose[i])
        pose.frame = World.current_world.robot.get_link_tf_frame(pose.frame)  # TODO: pose.frame is a link name?
        source = query_result.res[0].poseSource[i]

        lt = LocalTransformer()
        pose = lt.transform_pose(pose, "map")

        pose_candidates[source] = pose

    return pose_candidates

@init_robokudo_interface
def query_empty(object_desc: ObjectDesignatorDescription):
    """Sends a query to RoboKudo for an empty query."""
    global robokudo_action_client
    object_goal = make_query_goal_msg(object_desc)
    robokudo_action_client(object_goal)

    return query_result


@init_robokudo_interface
def query_human() -> PoseStamped:
    """Sends a query to RoboKudo to look for a human."""
    global robokudo_action_client
    global human_bool
    global human_pose

    human_bool = False

    def callback(pose):
        global human_bool
        human_bool = True
        human_pose = pose

    # Create client and send goal
    rospy.Subscriber("/human_pose", PoseStamped, callback)
    robokudo_action_client(QueryGoal())

    while not human_bool:
        rospy.sleep(0.5)

    return human_pose


@init_robokudo_interface
def stop_query_human():
    """Sends a query to RoboKudo to stop human detection."""
    global robokudo_action_client
    robokudo_action_client.cancel_all_goals()
    rospy.loginfo("Cancelled current goal")
