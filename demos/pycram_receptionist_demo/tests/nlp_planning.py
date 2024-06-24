import rospy

from pycram.designators.action_designator import *
from demos.pycram_receptionist_demo.utils.new_misc import *
from pycram.enums import ObjectType
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool, UInt16

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_8.urdf")
giskardpy.init_giskard_interface()
RobotStateUpdater("/tf", "/giskard_joint_states")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])



# variables for communcation with nlp
pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)
#pub_color = rospy.Publisher('/hsrb/command_status_led', UInt16, queue_size=5, latch=True)
response = ""
callback = False
doorbell = False

# Declare variables for humans
host = HumanDescription("Alina", fav_drink="water")
guest1 = HumanDescription("guest1")
guest2 = HumanDescription("guest2")
seat_number = 2


def data_cb(data):
    global response
    global callback

    response = data.data.split(",")
    callback = True


def doorbell_cb(data):
    global doorbell
    doorbell = True


def misc_fct():
    global response
    global callback

    rospy.Subscriber("/nlp_out", String, data_cb)

    with real_robot:
        # receive data from nlp via topic
        rospy.Subscriber("nlp_out", String, data_cb)
        rospy.Subscriber("nlp_out2", String, doorbell_cb)

        TalkingMotion("waiting for bell").resolve().perform()

        while not doorbell:
            print("no bell")

        TalkingMotion("Bell detected").resolve().perform()



misc_fct()
