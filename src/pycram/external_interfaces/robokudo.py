import sys
from threading import Lock, RLock
from typing import Any

import actionlib
import rosnode
import rospy
from geometry_msgs.msg import PointStamped
from typing_extensions import List, Callable, Optional
from pycram.datastructures.enums import ObjectType

from ..datastructures.pose import Pose
from ..designator import ObjectDesignatorDescription

is_initialized = False

try:
    from robokudo_msgs.msg import ObjectDesignator as robokudo_ObjectDesignator
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult
except ModuleNotFoundError as e:
    rospy.logwarn("Failed to import Robokudo messages, the real robot will not be available")

is_init = False

number_of_par_goals = 0
robokudo_lock = Lock()
robokudo_rlock = RLock()
with robokudo_rlock:
    par_threads = {}
    par_motion_goal = {}


def thread_safe(func: Callable) -> Callable:
    """
    Adds thread safety to a function via a decorator. This uses the robokudo_lock

    :param func: Function that should be thread safe
    :return: A function with thread safety
    """

    def wrapper(*args, **kwargs):
        with robokudo_rlock:
            return func(*args, **kwargs)

    return wrapper


def init_robokudo_interface(func: Callable) -> Callable:
    """
    Checks if the ROS messages are available and if Robokudo is running, if that is the case the interface will be
    initialized.

    :param func: Function this decorator should be wrapping
    :return: A callable function which initializes the interface and then calls the wrapped function
    """

    def wrapper(*args, **kwargs):
        global is_init
        if is_init and "/robokudo" in rosnode.get_node_names():
            return func(*args, **kwargs)
        elif is_init and "/robokudo" not in rosnode.get_node_names():
            rospy.logwarn("Robokudo node is not available anymore, could not initialize robokudo interface")
            is_init = False
            giskard_wrapper = None
            return

        if "robokudo_msgs" not in sys.modules:
            rospy.logwarn("Could not initialize the Robokudo interface since the robokudo_msgs are not imported")
            return

        if "/robokudo" in rosnode.get_node_names():
            rospy.loginfo_once("Successfully initialized Robokudo interface")
            is_init = True
        else:
            rospy.logwarn("Robokudo is not running, could not initialize Robokudo interface")
            return
        return func(*args, **kwargs)

    return wrapper


def make_query_goal_msg(obj_desc: ObjectDesignatorDescription) -> 'QueryGoal':
    """
    Creates a QueryGoal message from a PyCRAM Object designator description for the use of Querying RobotKudo.

    :param obj_desc: The PyCRAM object designator description that should be converted
    :return: The RoboKudo QueryGoal for the given object designator description
    """
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    goal_msg = QueryGoal()
    goal_msg.obj.uid = str(id(obj_desc))
    goal_msg.obj.type = str(obj_desc.types[0].name)  # For testing purposes
    if ObjectType.JEROEN_CUP == obj_desc.types[0]:
        goal_msg.obj.color.append("blue")
    elif ObjectType.BOWL == obj_desc.types[0]:
        goal_msg.obj.color.append("red")
    return goal_msg


def query(object_desc: ObjectDesignatorDescription) -> ObjectDesignatorDescription.Object:
    """
    Sends a query to RoboKudo to look for an object that fits the description given by the Object designator description.
    For sending the query to RoboKudo a simple action client will be created and the Object designator description is
    sent as a goal.
    """
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

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

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
    wait = client.wait_for_result()
    pose_candidates = {}
    # todo check if query is even filled
    for i in range(0, len(query_result.res[0].pose)):
        pose = Pose.from_pose_stamped(query_result.res[0].pose[i])
        # todo check if frame exist and if not in map
        # pose.frame = BulletWorld.current_bullet_world.robot.get_link_tf_frame(pose.frame)
        source = query_result.res[0].pose_source[0]

        pose_candidates[source] = pose

    return pose_candidates


@init_robokudo_interface
def send_query(obj_type: Optional[str] = None, region: Optional[str] = None,
               attributes: Optional[List[str]] = None) -> Any:
    """Generic function to send a query to RoboKudo."""
    goal = QueryGoal()

    if obj_type:
        # detects all obj in sight and returns obj-kind and poses
        # TODO: adjust so that specific things can be detected
        goal.obj.type = "detect"
    if region:
        goal.obj.location = region
    if attributes:
        goal.obj.attribute = attributes

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()

    query_result = None

    def done_callback(state, result):
        nonlocal query_result
        query_result = result
        rospy.loginfo("Query completed")

    client.send_goal(goal, done_cb=done_callback)
    client.wait_for_result()
    return query_result


@init_robokudo_interface
def query_object(obj_desc: ObjectDesignatorDescription) -> dict:
    """Query RoboKudo for an object that fits the description."""
    goal = QueryGoal()
    goal.obj.uid = str(id(obj_desc))
    goal.obj.type = str(obj_desc.types[0].name)

    result = send_query(obj_type=goal.obj.type)

    pose_candidates = {}
    if result and result.res:
        for i in range(len(result.res[0].pose)):
            pose = Pose.from_pose_stamped(result.res[0].pose[i])
            source = result.res[0].pose_source[0]
            pose_candidates[source] = pose
    return pose_candidates


@init_robokudo_interface
def query_human() -> PointStamped:
    """
        Sends a query to RoboKudo to look for a Human
        returns a PointStamped of pose where human is. keeps publishing it onto the
        topic /human_pose
    """
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global human_bool
    global query_result
    global human_pose

    def active_callback():
        rospy.loginfo("Send query to Robokudo to perceive a human")

    def done_callback(state, result):
        rospy.loginfo("Finished perceiving")
        global human_bool
        human_bool = True
        global query_result
        query_result = result

    def feedback_callback(msg):
        # rospy.loginfo("Got feedback")
        global feedback_result
        feedback_result = msg

    def callback(pose):
        global human_bool
        global human_pose
        human_bool = True
        human_pose = pose

    # create client to communicate with perception
    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    object_goal = goal_msg = QueryGoal()
    object_goal.obj.type = 'human'
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)

    # if no human is detected
    human_bool = False
    waiting_human = False
    rospy.Subscriber("/human_pose", PointStamped, callback)

    while not human_bool:
        rospy.sleep(0.5)

    return human_pose


@init_robokudo_interface
def stop_query():
    """Stop any ongoing query to RoboKudo."""
    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    client.wait_for_server()
    client.cancel_all_goals()
    rospy.loginfo("Cancelled current RoboKudo query goal")


@init_robokudo_interface
def query_specific_region(region: str, extra: Optional[Any]) -> Any:
    """Query RoboKudo to scan a specific region."""

    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global query_result

    if region == "sofa":
        def active_callback():
            rospy.loginfo("Send query to Robokudo to scan for seat and human")

        def done_callback(state, result: QueryResult):
            rospy.loginfo("Finished perceiving")
            global query_result
            query_result = result.res

        # fill Query with information so that perception looks for a seat
        object_goal = QueryGoal()

        object_goal.obj.location = "sofa"  # aktivate region filter

        client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
        rospy.loginfo("Waiting for action server")
        client.wait_for_server()
        client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback)
        # TODO: necessary?
        client.wait_for_result()

        return query_result

    if region == "region":

        def active_callback():
            rospy.loginfo("Send query to Robokudo to scan a specific region")

        def done_callback(state, result):
            rospy.loginfo("Finished perceiving")
            global query_result
            # Todo is result.res needed instead?
            query_result = result.res

        def feedback_callback(msg):
            pass

        object_goal = QueryGoal()
        object_goal.obj.location = str(extra)
        client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
        rospy.loginfo("Waiting for action server")
        client.wait_for_server()
        client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
        client.wait_for_result()

        return query_result


@init_robokudo_interface
def query_human_attributes() -> Any:
    """Query RoboKudo for human attributes like brightness of clothes, headgear, and gender."""
    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global query_result

    def active_callback():
        rospy.loginfo("Send query to Robokudo to look for human and attributes")

    def done_callback(state, result: QueryResult):
        rospy.loginfo("Finished perceiving")
        global query_result
        query_result = result

    object_goal = QueryGoal()
    # Perception will detect brightness of clothes, kind of clothes, headgear and gender
    object_goal.obj.attribute = ["attributes"]

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback)
    client.wait_for_result()

    return query_result


def faces_query() -> Any:
    """
    Sends a query to RoboKudo to look for a human. returns four attributes of the perceived human.
    """

    from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult

    global query_result

    def active_callback():
        rospy.loginfo("Send query to Robokudo for face recognition")

    def done_callback(state, result: QueryResult):
        rospy.loginfo("Finished perceiving")
        global query_result
        query_result = result

    object_goal = QueryGoal()

    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    rospy.loginfo("Waiting for action server")
    client.wait_for_server()
    client.send_goal(object_goal, active_cb=active_callback, done_cb=done_callback)
    client.wait_for_result()

    return query_result


@init_robokudo_interface
def query_waving_human() -> Pose:
    """Query RoboKudo for detecting a waving human."""
    result = send_query(obj_type='human')
    if result and result.res:
        try:
            pose = Pose.from_pose_stamped(result.res[0].pose[0])
            return pose
        except IndexError:
            pass
    return None
