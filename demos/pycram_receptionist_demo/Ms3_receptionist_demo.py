import rospy

from pycram.designators.action_designator import DetectAction, NavigateAction
from demos.pycram_receptionist_demo.utils.misc import *
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool
import pycram.external_interfaces.navigate as moveBase

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

kitchen = Object("kitchen", "environment", "kitchen.urdf")
giskardpy.init_giskard_interface()
RobotStateUpdater("/tf", "/giskard_joint_states")

pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)

with real_robot:

    # declare variables for humans
    host = HumanDescription("Yannis", fav_drink="Tea")
    guest1 = HumanDescription("guest1")

    # look for human
    DetectAction(technique='human', state='start').resolve().perform()
    rospy.loginfo("human detected")

    # look at guest and introduce
    giskardpy.move_head_to_human()
    TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()
    rospy.sleep(1)

    # signal to start listening
    pub_nlp.publish("start listening")

    # receives name and drink via topic
    x = rospy.Subscriber("nlp_out", String, talk_request_nlp)

    # failure handling
    rospy.Subscriber("nlp_feedback", Bool, talk_error)

    while not understood:
        rospy.sleep(1)



