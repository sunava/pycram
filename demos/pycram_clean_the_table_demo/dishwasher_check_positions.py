import time
from enum import Enum

import geometry_msgs.msg
import rospy.core
import tf

from demos.pycram_clean_the_table_demo.utils.misc import *
from pycram.process_module import real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.utilities.robocup_utils import StartSignalWaiter
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher

# Create an instance of the StartSignalWaiter
#start_signal_waiter = StartSignalWaiter()
text_to_speech_publisher = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalplate", "Metalbowl", "Metalmug", "Fork", "Spoon"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# x pose of the end of the couch table
table_pose = 4.84

# name of the dishwasher handle and dishwasher door
handle_name = "sink_area_dish_washer_door_handle"
door_name = "sink_area_dish_washer_door"

# Intermediate positions for a safer navigation
move_to_the_middle_table_pose = [2.2, 1.98, 0]
move_to_the_middle_dishwasher_pose = [2.2, -0.1, 0]

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")
giskardpy.init_giskard_interface()

robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
apart_desig = BelieveObject(names=["kitchen"])

giskardpy.initial_adding_objects()
giskardpy.sync_worlds()

# Wait for the start signal
#start_signal_waiter.wait_for_startsignal()

# Once the start signal is received, continue with the rest of the script
rospy.loginfo("Start signal received, now proceeding with tasks.")

class PlacingXPose(Enum):
    """
    Differentiate the x pose for placing
    """
    CUTLERY = 2.376 # dishwasher_main_map.position.x + ??
    SPOON = 2.376
    FORK = 2.376
    PLASTICKNIFE = 2.376
    KNIFE = 2.376
    METALBOWL = 2.83
    METALMUG = 2.79
    METALPLATE = 2.8


class PlacingYPose(Enum):
    """
    Differentiate the y pose for placing
    """
    CUTLERY = -1.59
    SPOON = -1.59
    FORK = -1.59
    PLASTICKNIFE = -1.59
    KNIFE = -1.59
    METALBOWL = -1.73
    METALMUG = -1.75
    METALPLATE = -1.65

dishwasher_main_name = "sink_area_dish_washer_main"

def calculate_placing_pos(obj):
    lt = LocalTransformer()

    link = apartment.get_link_tf_frame(dishwasher_main_name)

    world.current_bullet_world.add_vis_axis(apartment.get_link_pose(dishwasher_main_name))

    #dishwasher = Pose([2.376 ,-1.59,0.4],[0,0,0,1]) # dies ist 1), gebe deine Map pose an
    dishwasher = Pose([0.6050592811085371, 0.26473268332425715, -0.05399999618530266],
                      [0, 0, -0.7073882378922517, 0.7068252124052276]) # Dies ist 2), gib die aus 1) berechnete dishwasher_main pose an


    dishwasher.header.frame_id = link # auskommentieren, wenn 1) verwendet
    newp = lt.transform_pose(dishwasher, "map") # link statt map wenn 1) verwendet. map wenn 2) verwendet
    print(newp)
    world.current_bullet_world.add_vis_axis(newp)

    # schreibe funktion, welche auf diswasher in link die Werte aus 2) addiert/subtrahiert und transformiere zurück in map


with real_robot:
    rospy.loginfo("Starting demo")
    #TalkingMotion("Starting demo").resolve().perform()

    dishwasher_main_pose = apartment.get_link_pose(dishwasher_main_name)
    print(dishwasher_main_pose)
    calculate_placing_pos("test")