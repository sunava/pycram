
import rospy
from geometry_msgs.msg import WrenchStamped

import pycram.language
from pycram.designators.action_designator import fts, DetectAction, HeadFollowAction
from pycram.designators.motion_designator import MoveGripperMotion
from pycram.language import Monitor
from pycram.plan_failures import SensorMonitoringCondition
from pycram.ros.force_torque_sensor import ForceTorqueSensor
from utils.startup import startup
from pycram.language import *
from pycram.process_module import simulated_robot, real_robot

world, v, text_to_speech_publisher, image_switch_publisher, move = startup()
rospy.loginfo("u can start your demo now")



def monitor_func():
    der: WrenchStamped() = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        return SensorMonitoringCondition
    return False


def printloop():
    print("")


with real_robot:
    talk = True
    try:
        text_to_speech_publisher.publish_text("Starting Carry my Luggage demo.", talk)
        text_to_speech_publisher.publish_text("Push down my Hand, when you are Ready.", talk)
        plan = Code(lambda: printloop()) * 999999 >> Monitor(monitor_func)
        plan.perform()
    except SensorMonitoringCondition:
        print("human interrupted")
        text_to_speech_publisher.publish_text("Looking for a human",talk)
        # look for human
        DetectAction(technique='human').resolve().perform()

        # look at guest and introduce
        HeadFollowAction('start').resolve().perform()
        text_to_speech_publisher.publish_text("I can see you. Please, hand me the bag.", talk)
        text_to_speech_publisher.publish_text("Opening my Gripper.", talk)
        MoveGripperMotion(motion="open", gripper="left").resolve().perform()
        text_to_speech_publisher.publish_text("Push down my Hand, when you are Ready.", talk)
        try:
            plan = Code(lambda: printloop()) * 999999 >> Monitor(monitor_func)
            plan.perform()
        except SensorMonitoringCondition:
            text_to_speech_publisher.publish_text("Closing my Gripper.", talk)
            MoveGripperMotion(motion="close", gripper="left").resolve().perform()