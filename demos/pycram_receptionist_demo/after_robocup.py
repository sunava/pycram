import rospy
from geometry_msgs.msg import PoseStamped
from pycram.datastructures.dataclasses import Color
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import TalkingMotion
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.designators.object_designator import *
from std_msgs.msg import String, Bool
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, \
    HSRBMoveGripperReal, StartSignalWaiter

# new imports
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.datastructures.enums import ImageEnum as ImageEnum, Arms
from pycram.world_concepts.world_object import Object
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.worlds.bullet_world import BulletWorld

extension = ObjectDescription.get_file_extension()

# def monitor_func():
#     der = fts.get_last_value()
#     if abs(der.wrench.force.x) > 10.30:
#         return SensorMonitoringCondition
#     return False

world = BulletWorld(WorldMode.DIRECT)
gripper = HSRBMoveGripperReal()
robot = Object("hsrb", ObjectType.ROBOT, f"hsrb{extension}")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot_color = Color(R=0.6, G=0.6, B=0.6, A=1)
robot.set_color(robot_color)

# TODO: change to right map
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_5.urdf")
# apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment{extension}")
# giskardpy.init_giskard_interface()
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

# important Publishers
move = PoseNavigator()
talk = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()

# variables for communcation with nlp
pub_nlp = rospy.Publisher('/startListener', String, queue_size=16)
pub_bell = rospy.Publisher('/startSoundDetection', String, queue_size=16)
response = ""
wait_bool = False
callback = False
doorbell = False
timeout = 15  # 12 seconds timeout
laser = StartSignalWaiter()

# Declare variables for humans
# TODO: change to given name before challenge
host = HumanDescription("John", fav_drink="milk")
host.set_id(1)

guest1 = HumanDescription("Lisa", fav_drink="water")
guest1.set_attributes(['male', 'without a hat', 'wearing a t-shirt', ' a dark top'])
guest1.set_id(0)

guest2 = HumanDescription("Sarah", fav_drink="Juice")
guest2.set_attributes(['female', 'with a hat', 'wearing a t-shirt', ' a bright top'])

# pose variables
door_pose = Pose([3, 1.0, 0], [0, 0, 1, 0])
convo_pose = Pose([4.2, 0.15, 0], [0, 0, 1, 0])
convo_pose_to_couch = Pose([4.2, 0.15, 0], [0, 0, 0, 1])
couch_pose = Pose([8.5, 0, 0], [0, 0, 0, 1])
couch_pose_to_door = Pose([8.6, 0, 0], [0, 0, 1, 0])
start_pose = Pose([3.7, 0.19, 0], [0, 0, 1, 0])

couch_pose_semantik = PoseStamped()
couch_pose_semantik.pose.position.x = 10.8
couch_pose_semantik.pose.position.y = -0.27
couch_pose_semantik.pose.position.z = 0.8



def pakerino(torso_z=0.05, config=None):
    """
    replace function for park arms, robot takes pose of configuration of joint
    """

    if not config:
        config = {'arm_lift_joint': torso_z, 'arm_flex_joint': 0, 'arm_roll_joint': -1.2,
                  'wrist_flex_joint': -1.5, 'wrist_roll_joint': 0, 'head_pan_joint': 0}

    #giskardpy.avoid_all_collisions()
    giskardpy.achieve_joint_goal(config)
    print("[32mParking done")


def demo(step):
    with real_robot:
        global wait_bool
        global callback
        global doorbell
        global guest1
        global guest2


        # DetectAction(technique='human').resolve().perform()
        # HeadFollowAction("start").resolve().perform()
        # rospy.sleep(5)

        # SetGripperAction(grippers=[Arms.LEFT], motions=[GripperState.OPEN]).resolve().perform()

        # pose1 = Pose()
        # pose1.pose.position.x = 2.4
        # pose1.pose.position.y = 1.1
        # pose1.pose.position.z = 1
        # LookAtAction([pose1]).resolve().perform()

        # attr_list = DetectAction(technique='attributes', state='face').resolve().perform()
        # rospy.loginfo(attr_list)

        #PointingAction(0.8,1.8, 0.8).resolve().perform()
        TalkingMotion("hello").perform()

        # ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()
        print("end")
        #rospy.sleep(10)



demo(0)
