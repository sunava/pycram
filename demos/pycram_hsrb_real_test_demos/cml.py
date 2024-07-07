#!/usr/bin/env python3
import time

import actionlib
import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped, PointStamped
from robokudo_msgs.msg import QueryAction, QueryGoal

from pycram.designators.action_designator import fts, ParkArmsAction, DetectAction
from pycram.designators.motion_designator import MoveGripperMotion
from pycram.enums import Arms, ImageEnum
from pycram.fluent import Fluent
from pycram.language import Monitor, Code
from pycram.plan_failures import SensorMonitoringCondition, PlanFailure, HumanNotFoundCondition
from pycram.process_module import real_robot
from utils.startup import startup
from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater

from pycram.enums import ObjectType

from pycram.external_interfaces.navigate import PoseNavigator
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.bullet_world import BulletWorld, Object
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, HSRBMoveGripperReal
import pycram.external_interfaces.giskard_new as giskardpy

# Initialize the necessary components
# world = BulletWorld("DIRECT")
# v = VizMarkerPublisher()

# world = BulletWorld()
# v = VizMarkerPublisher()
# kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")

talk = TextToSpeechPublisher()
img = ImageSwitchPublisher()
gripper = HSRBMoveGripperReal()

# robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
# robot.set_color([0.5, 0.5, 0.9, 1])

# RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
# KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")
rospy.loginfo("Loading done")
giskardpy.init_giskard_interface()
rkclient = actionlib.SimpleActionClient('robokudo/query', QueryAction)
rospy.loginfo("Waiting for action server")
rkclient.wait_for_server()
rospy.loginfo("You can start your demo now")

# Initialize global variable
global human_bool
human_bool = False


class Human:
    """
    Class that represents humans. This class does not spawn a human in a simulation.
    """

    def __init__(self):
        self.human_pose = Fluent()

        self.last_msg_time = time.time()

        self.threshold = 5.0  # seconds

        # Subscriber to the human pose topic
        self.human_pose_sub = rospy.Subscriber("/cml_human_pose", PointStamped, self.human_pose_cb)

        # Timer to check for no message
        self.timer = rospy.Timer(rospy.Duration(1), self.check_for_no_message)

    def check_for_no_message(self, event):
        current_time = time.time()
        if (current_time - self.last_msg_time) > self.threshold:
            # rospy.loginfo("No messages received for %s seconds", self.threshold)
            self.human_pose.set_value(False)

    def human_pose_cb(self, HumanPoseMsg):
        """
        Callback function for human_pose Subscriber.
        Sets the attribute human_pose when someone (e.g. Perception/Robokudo) publishes on the topic.
        :param HumanPoseMsg: received message
        """
        self.last_msg_time = time.time()

        if HumanPoseMsg:
            self.human_pose.set_value(True)
        else:
            self.human_pose.set_value(False)


# one_time_pub = rospy.Publisher("/cml_human_pose", PointStamped, queue_size=10)
# msg = PointStamped()
# one_time_pub.publish(msg)
human = Human()


def monitor_func_human():
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        print("sensor")
        return SensorMonitoringCondition
    if not human.human_pose.get_value():
        print("human")
        return HumanNotFoundCondition
    return False


def monitor_func():
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        return SensorMonitoringCondition
    return False


def callback(point):
    global human_bool
    human_bool = True
    print("Human detected")


def first_part(talk_bool):
    try:
        talk.pub_now("Starting Carry my Luggage demo.", talk_bool)
        img.pub_now(ImageEnum.HI.value)  # hi im toya
        talk.pub_now("Push down my Hand, when you are Ready.", talk_bool)
        img.pub_now(ImageEnum.PUSHBUTTONS.value)
        plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
        plan.perform()
    except SensorMonitoringCondition:
        img.pub_now(ImageEnum.HI.value)  # hi im toya

        talk.pub_now("Looking for a human", talk_bool)
        img.pub_now(ImageEnum.SEARCH.value)  # search
        goal_msg = QueryGoal()
        rkclient.send_goal(goal_msg)
        # worksme this worked before i moved it
        rospy.loginfo("Waiting for human to be detected")
        no_human = True
        while no_human:

            if human.human_pose.wait_for(timeout=5):
                no_human = False
            else:
                talk.pub_now("Looking for a human, please step in front of me", talk_bool)
                rkclient.send_goal(goal_msg)

        talk.pub_now("Found a Human", talk_bool)
        img.pub_now(ImageEnum.HI.value)

        gripper.pub_now("open")
        rospy.sleep(1)
        talk.pub_now("I am not able to pick up the bag. Please hand it in", talk_bool)
        rospy.sleep(1)

        talk.pub_now("Push down my Hand, when you are Ready.", talk_bool)
        img.pub_now(ImageEnum.PUSHBUTTONS.value)
        try:
            plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
            plan.perform()
        except SensorMonitoringCondition:
            talk.pub_now("Closing my Gripper.", talk_bool)
            img.pub_now(ImageEnum.HI.value)
            gripper.pub_now("close")


def pakerino(torso_z=0.15, config=None): #moveme
    if not config:
        config = {'arm_lift_joint': torso_z, 'arm_flex_joint': 0, 'arm_roll_joint': -1.2, 'wrist_flex_joint': -1.5,
                  'wrist_roll_joint': 0}
    giskardpy.avoid_all_collisions()
    giskardpy.achieve_joint_goal(config)
    print("Parking done")


def cml(step="default"): #worksme

    with real_robot:
        talk_bool = True
        if step == "default":
            pakerino()
            first_part(talk_bool)
        if step == "lost_human":
            no_human = True
            while no_human:
                goal_msg = QueryGoal()
                if human.human_pose.get_value():
                    no_human = False
                else:
                    rospy.sleep(5)
                    talk.pub_now("I lost you, please step in front of me", talk_bool)
                    rkclient.send_goal(goal_msg)
        talk.pub_now("Following you.", talk_bool)
        img.pub_now(ImageEnum.FOLLOWSTOP.value)
        talk.pub_now("Push down my Hand, when we arrived.", talk_bool)

        try:
            plan = Code(lambda: giskardpy.cml(False)) >> Monitor(monitor_func_human)
            plan.perform()
        except Exception as e:
            print(f"Exception type: {type(e).__name__}")

            if isinstance(e, SensorMonitoringCondition):

                talk.pub_now("We have arrived.", talk_bool)
                talk.pub_now("I will open my Gripper, to give you the bag.", talk_bool)
                talk.pub_now("Push down my Hand, when you are Ready.", talk_bool)
                img.pub_now(ImageEnum.PUSHBUTTONS.value)
                try:
                    plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func)
                    plan.perform()
                except SensorMonitoringCondition:
                    gripper.pub_now("open")
                    talk.pub_now("Driving Back.", talk_bool)
                    img.pub_now(ImageEnum.DRIVINGBACK.value)
                    giskardpy.cml(True)
                    talk.pub_now("done.", talk_bool)
            elif isinstance(e, HumanNotFoundCondition):
                cml("lost_human")
            #fixme i dont know what to do here
            else:
                print("idk what happned")


        #
        # try:
        #     plan = Code(lambda: giskardpy.cml(False)) >> Monitor(monitor_func_human)
        #     plan.perform()
        # except (SensorMonitoringCondition, HumanNotFoundCondition) as e :
        #     print(e)
        #     print("jo")
        # if e == SensorMonitoringCondition:
        #     rkclient.cancel_all_goals
        #     talk.pub_now("We have arrived.", talk_bool)
        #     talk.pub_now("I will open my Gripper, to give you the bag.", talk_bool)
        #     talk.pub_now("Push down my Hand, when you are Ready.", talk_bool)
        #     img.pub_now(ImageEnum.PUSHBUTTONS.value)
        #     try:
        #         plan = Code(lambda: rospy.sleep(1)) * 99999999 >> Monitor(monitor_func())
        #         plan.perform()
        #     except SensorMonitoringCondition:
        #         gripper.pub_now("open")
        #
        #         talk.pub_now("Driving Back.", talk_bool)
        #         img.pub_now(ImageEnum.DRIVINGBACK.value)
        #         giskardpy.cml(True)
        #         talk.pub_now("done.", talk_bool)
        # else:
        #     print(e)


# goal_msg = QueryGoal()
# print(goal_msg)
# gis()
cml("lost_human")
# "hi.png" -> 0
# "talk_bool.png" -> 1
# "dish.png" -> lost_humanlost_human2
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
