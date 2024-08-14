import numpy as np
import rospy
import tf
from geometry_msgs.msg import PointStamped
from gtts import gTTS
import playsound
import io
import os
from tempfile import NamedTemporaryFile
from demos.pycram_storing_groceries_demo.utils.misc import *
from pycram.designators.location_designator import find_placeable_pose
from pycram.language import Code

from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.designators.action_designator import *
from pycram.enums import ObjectType
from pycram.process_module import real_robot, _real_robot
import pycram.external_interfaces.giskard_new as giskardpy
# import pycram.external_interfaces.giskard as giskardpy_old

import pycram.external_interfaces.robokudo as robokudo
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, StartSignalWaiter, \
    HSRBMoveGripperReal, pakerino, GraspListener
from pycram.designator import LocationDesignatorDescription
import random

world = BulletWorld()
v = VizMarkerPublisher()
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "robocup_vanessa.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"])
table_pose = Pose([6.6, 4.9, 0.0], [0.0, 0.0, 0, 1])
long_table_1 = Pose([6.65, 4.6, 0],[0, 0, 0, 1])
long_table_pick = Pose([6.60, 4.6, 0],[0, 0, 0, 1])

long_table_1_rotated = Pose([6.65, 4.6, 0],[0, 0, 1, 0])
shelf_1 = Pose([6.2, 5.6, 0],[0, 0, 1, 0])
shelf_1_rotated1 = Pose([6.2, 5.6, 0],[0, 0, -0.7, 0.7])
shelf_1_rotated = Pose([6.2, 5.6, 0],[0, 0, 0, 1])
lt = LocalTransformer()

milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])
cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([2.5, 2.3, 1.05]),
                color=[0, 1, 0, 1])
spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([2.4, 2.2, 0.85]), color=[0, 0, 1, 1])
bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.5, 2.2, 1.02]), color=[1, 1, 0, 1])
human_female = Object("human_female", ObjectType.HUMAN, "female_standing.stl", pose=Pose([3, 3, 0]), color=[1, 1, 0, 1])

giskardpy.init_giskard_interface()
world.simulate(seconds=1, real_time=True)


def test_move():
    pose1 = Pose([1, 1, 0])
    giskardpy.teleport_robot(pose1)
    with _real_robot:
        NavigateAction([Pose([2, 2, 0])]).resolve().perform()
        giskardpy.teleport_robot(pose1)

def test_all_perception():
    with _real_robot:
        table_obj = DetectAction(technique='all').resolve().perform()
        first, *remaining = table_obj
        for dictionary in remaining:
            for value in dictionary.values():
                print(value)

def test_human_perception():
    with _real_robot:
        table_obj = DetectAction(technique='types', object_designator=BelieveObject(types=[ObjectType.MILK])).resolve().perform()
        first, *remaining = table_obj
        for dictionary in remaining:
            for value in dictionary.values():
                print(value)


def test_talk():
    with _real_robot:
        TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()