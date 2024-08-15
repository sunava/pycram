import actionlib
import rospy
from robokudo_msgs.msg import QueryAction

from pycram.designators.action_designator import DetectAction
from pycram.process_module import real_robot
from utils.startup import startup
import pycram.external_interfaces.giskard_new as giskardpy

# Initialize the necessary components
world, v, text_to_speech_publisher, image_switch_publisher, move = startup()
giskardpy.init_giskard_interface()
client = actionlib.SimpleActionClient('robokudo/query', QueryAction)
rospy.loginfo("Waiting for action server")
client.wait_for_server()
rospy.loginfo("You can start your demo now")

with real_robot:

    DetectAction(technique='all').resolve().perform()