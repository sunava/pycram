from pycram.designators.action_designator import fts, ParkArmsAction, DetectAction
from pycram.designators.motion_designator import MoveGripperMotion
from pycram.enums import Arms, ImageEnum
from pycram.fluent import Fluent
from pycram.language import Monitor, Code
from pycram.local_transformer import LocalTransformer
from pycram.plan_failures import SensorMonitoringCondition
from pycram.pose import Pose
from pycram.process_module import real_robot
from demos.pycram_hsrb_real_test_demos.utils.startup import startup
import pycram.external_interfaces.giskard_new as giskardpy
import pycram.external_interfaces.robokudo as robokudo
import pycram.external_interfaces.navigate as navi
import rospy
import tf
from geometry_msgs.msg import PoseStamped

from pycram.ros.viz_marker_publisher import ManualMarkerPublisher

# # Initialize the necessary components
# world, v, text_to_speech_publisher, image_switch_publisher, move, robot = startup()
# giskardpy.init_giskard_interface()
# robokudo.init_robokudo_interface()
# marker = ManualMarkerPublisher()
# listener = tf.TransformListener()


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

move_client = None


def interrupt():
    global move_client
    move_client.cancel_all_goals()

class movebase_client():

    def __init__(self):
        global move_client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        move_client = self.client


    def interrupt(self):
        global move_client
        self.client.cancel_all_goals()

    def pub_now(self, navpose):
        rospy.logerr("Action server not available!")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose = navpose

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()


if __name__ == '__main__':
    navPose = Pose([4,2.19,0])
    move = movebase_client()
    move.pub_now(navPose)


