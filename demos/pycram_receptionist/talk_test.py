# from pycram.ros.tf_broadcaster import TFBroadcaster
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


def talker(talk_string):
    pub = rospy.Publisher('/talk_request', Voice, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    hello_str = Voice()
    hello_str.language = 1
    hello_str.sentence = talk_string

    pub.publish(hello_str)
    rate.sleep()

def name_drink_talker(string_list):
    pub = rospy.Publisher('/talk_request', Voice, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    toyas_text = "Hey" + string_list[0] + "your favorite drink is " + string_list[1] + "nice to meet you. this is the end of the demo"
    voice_msgs = Voice()
    voice_msgs.language = 1
    voice_msgs.sentence = toyas_text
    pub.publish(voice_msgs)


