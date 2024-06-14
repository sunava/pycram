import actionlib
import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped, PointStamped
from robokudo_msgs.msg import QueryAction, QueryGoal

from pycram.designators.action_designator import fts, ParkArmsAction, DetectAction
from pycram.designators.motion_designator import MoveGripperMotion
from pycram.enums import Arms, ImageEnum
from pycram.fluent import Fluent
from pycram.language import Monitor, Code
from pycram.plan_failures import SensorMonitoringCondition
from pycram.process_module import real_robot
from utils.startup import startup
import pycram.external_interfaces.giskard_new as giskardpy

# Initialize the necessary components
world, v, text_to_speech_publisher, image_switch_publisher, move, robot = startup()
giskardpy.init_giskard_interface()
client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
rospy.loginfo("Waiting for action server")
client.wait_for_server()
rospy.loginfo("You can start your demo now")

# Initialize global variable
global human_bool
human_bool = False


def monitor_func():
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        return SensorMonitoringCondition
    return False


def callback(point):
    global human_bool
    human_bool = True
    print("Human detected")


class Human:
    """
    Class that represents humans. this class does not spawn a human in a simulation.
    """

    def __init__(self):
        self.human_pose = Fluent()

        self.human_pose_sub = rospy.Subscriber("/cml_human_pose", PointStamped, self.human_pose_cb)

    def human_pose_cb(self, HumanPoseMsg):
        """
        callback function for human_pose Subscriber.
        sets the attribute human_pose when someone (e.g. Perception/Robokudo) publishes on the topic
        :param HumanPoseMsg: received message
        """

        self.human_pose.set_value(True)
        rospy.sleep(1)


def all():
    with real_robot:
        talk = True
        # ParkArmsAction([Arms.LEFT]).resolve().perform()
        try:
            text_to_speech_publisher.pub_now("Starting Carry my Luggage demo.", talk)
            image_switch_publisher.pub_now(ImageEnum.HI.value)  # hi im toya
            text_to_speech_publisher.pub_now("Push down my Hand, when you are Ready.", talk)
            image_switch_publisher.pub_now(ImageEnum.PUSHBUTTONS.value)
            plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
            plan.perform()
        except SensorMonitoringCondition:
            image_switch_publisher.pub_now(ImageEnum.HI.value)  # hi im toya
            text_to_speech_publisher.pub_now("Looking for a human", talk)
            image_switch_publisher.pub_now(ImageEnum.SEARCH.value)  # search
            goal_msg = QueryGoal()
            client.send_goal(goal_msg)

            human = Human()

            rospy.loginfo("Waiting for human to be detected")
            human.human_pose.wait_for()
            text_to_speech_publisher.pub_now("Found a Human", talk)
            image_switch_publisher.pub_now(ImageEnum.HI.value)

            MoveGripperMotion(motion="open", gripper="left").resolve().perform()
            text_to_speech_publisher.pub_now("I am not able to pick up the bag. Please hand it in", talk)
            text_to_speech_publisher.pub_now("Push down my Hand, when you are Ready.", talk)
            image_switch_publisher.pub_now(ImageEnum.PUSHBUTTONS.value)
            try:
                plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
                plan.perform()
            except SensorMonitoringCondition:
                text_to_speech_publisher.pub_now("Closing my Gripper.", talk)
                image_switch_publisher.pub_now(ImageEnum.HI.value)
                MoveGripperMotion(motion="close", gripper="left").resolve().perform()
                text_to_speech_publisher.pub_now("Following you.", talk)
                image_switch_publisher.pub_now(ImageEnum.FOLLOWSTOP.value)
                text_to_speech_publisher.pub_now("Push down my Hand, when we arrived.", talk)
                try:
                    plan = Code(lambda: giskardpy.cml(False)) >> Monitor(monitor_func)
                    plan.perform()
                except SensorMonitoringCondition:

                    text_to_speech_publisher.pub_now("I will open my Gripper, to give you the bag.", talk)
                    text_to_speech_publisher.pub_now("Push down my Hand, when you are Ready.", talk)
                    image_switch_publisher.pub_now(ImageEnum.PUSHBUTTONS.value)
                    try:
                        plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
                        plan.perform()
                    except SensorMonitoringCondition:
                        MoveGripperMotion(motion="open", gripper="left").resolve().perform()

                        text_to_speech_publisher.pub_now("Driving Back.", talk)
                        image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
                        giskardpy.cml(True)
                        text_to_speech_publisher.pub_now("done.", talk)



# goal_msg = QueryGoal()
# print(goal_msg)
# gis()
#all()
# "hi.png" -> 0
# "talk.png" -> 1
# "dish.png" -> 2
# "done.png" -> 3
# "drop.png" -> 4
# "handover.png" -> 5
# "order.png" -> 6
# "picking.png" -> 7
# "placing.png" -> 8
# "repeat.png" -> 9
# "search.png" -> 10
# "waving.mp4" -> 11
# "following" -> 12
# "drivingback" -> 13
# "pushbuttons" -> 14
