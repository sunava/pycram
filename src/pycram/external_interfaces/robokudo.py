import rospy
import actionlib
from tmc_msgs.msg import Voice
from ..designator import ObjectDesignatorDescription
from ..pose import Pose
from ..local_transformer import LocalTransformer
from ..bullet_world import BulletWorld
from ..enums import ObjectType
from geometry_msgs.msg import PoseStamped, PointStamped
from typing import Any, Optional, List

# Global initialization flag
is_initialized = False


def init_robokudo_interface():
    global is_initialized
    if is_initialized:
        return
    try:
        from robokudo_msgs.msg import ObjectDesignator as robokudo_ObjectDesignator
        from robokudo_msgs.msg import QueryAction, QueryGoal, QueryResult
        is_initialized = True
        rospy.loginfo("Successfully initialized robokudo interface")
    except ModuleNotFoundError:
        rospy.logwarn("Could not import RoboKudo messages, RoboKudo interface could not be initialized")


def send_query(obj_type: Optional[str] = None, region: Optional[str] = None,
               attributes: Optional[List[str]] = None) -> Any:
    """Generic function to send a query to RoboKudo."""
    init_robokudo_interface()

    from robokudo_msgs.msg import QueryAction, QueryGoal

    goal = QueryGoal()

    if obj_type:
        goal.obj.type = obj_type
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


def query_human() -> PointStamped:
    """Query RoboKudo for human detection and return the detected human's pose."""
    result = send_query(obj_type='human')
    if result:
        return result  # Assuming result is of type PointStamped or similar.
    return None


def stop_query():
    """Stop any ongoing query to RoboKudo."""
    init_robokudo_interface()
    client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
    client.wait_for_server()
    client.cancel_all_goals()
    rospy.loginfo("Cancelled current RoboKudo query goal")


def query_specific_region(region: str) -> Any:
    """Query RoboKudo to scan a specific region."""
    return send_query(region=region)


def query_human_attributes() -> Any:
    """Query RoboKudo for human attributes like brightness of clothes, headgear, and gender."""
    return send_query(obj_type='human', attributes=["attributes"])


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
