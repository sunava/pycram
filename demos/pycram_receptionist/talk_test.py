# from pycram.ros.tf_broadcaster import TFBroadcaster
from datetime import time

import actionlib
import rospy
# from tmc_msgs.msg._TalkRequestActionGoal import TalkRequestActionGoal
from std_msgs.msg import String
from tmc_msgs.msg import *
import genpy
import struct
import actionlib_msgs.msg
import genpy
import std_msgs.msg
import tmc_msgs.msg

from pycram.bullet_world import BulletWorld, Object
from pycram.designator import ObjectDesignatorDescription
from pycram.designators.object_designator import BelieveObject
from pycram.enums import ObjectType
from pycram.pose import Pose
from pycram.process_module import real_robot
from pycram.robot_descriptions import robot_description
from pycram.ros.viz_marker_publisher import VizMarkerPublisher

def talker(talk_string):
    print("talker entry")
    pub = rospy.Publisher('/talk_request', Voice, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    hello_str = Voice()
    hello_str.language = 1
    hello_str.sentence = talk_string
    rospy.sleep(2)
    pub.publish(hello_str)
    rate.sleep()
    print('talker done')


def name_drink_talker(string_list):
    pub = rospy.Publisher('/talk_request', Voice, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    toyas_text = "Hey" + string_list[0] + "your favorite drink is " + string_list[
        1] + "nice to meet you. this is the end of the demo"
    voice_msgs = Voice()
    voice_msgs.language = 1
    voice_msgs.sentence = toyas_text
    rospy.sleep(2)
    pub.publish(voice_msgs)


