from itertools import islice

import numpy as np
import rospy
import tf
from dynamic_reconfigure.srv import ReconfigureRequest
from geometry_msgs.msg import PointStamped, Twist

from demos.pycram_storing_groceries_demo.utils.misc import *
from pycram.designators.location_designator import find_placeable_pose
from pycram.language import Code

from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.datastructures.enums import ObjectType, ImageEnum
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard_new as giskardpy
# import pycram.external_interfaces.giskard as giskardpy_old
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, HSRBMoveGripperReal, pakerino
from dynamic_reconfigure.msg import Config, BoolParameter, IntParameter, StrParameter, DoubleParameter, GroupState
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest


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
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "robocup123.urdf")
print(kitchen.links)