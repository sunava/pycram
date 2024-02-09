import rospy
from pycram.designators.action_designator import NavigateAction
from pycram.designators.motion_designator import TalkingMotion
import pycram.external_interfaces.giskard as giskardpy
from pycram.designators.object_designator import *
from std_msgs.msg import String
from demos.pycram_receptionist_demo.deprecated import talk_actions
from deprecated import deprecated

pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)


def talk_request(name_drink: List):
    """
    callback function that takes the data from nlp (name and drink) and lets the robot talk
    :param name_drink: List ["name", "drink"]
    """
    rospy.loginfo("nlp data:" + name_drink[0] + " " + name_drink[1])
    robot_text = "Hello " + name_drink[0] + " your favorite drink is " + name_drink[1] + " nice to meet you"
    TalkingMotion(robot_text).resolve.perform()


def talk_error(data):
    """
    callback function if no name/drink was heard
    """

    error_msgs = "i could not hear you, please repeat"
    TalkingMotion(error_msgs).resolve().perform()
    pub_nlp.publish("start listening")


def introduce(name1, drink1, name2, drink2):
    """
    Text for robot to introduce two people to each other
    """
    first = "Hey" + name2 + " This is " + name1 + "and the favorite drink of your guest is " + drink1
    second = name1 + "This is " + name2 + "his favorite drink is " + drink2
    TalkingMotion(first).resolve.perform()
    TalkingMotion(second).resolve.perform()



