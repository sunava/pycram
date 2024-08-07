import pycram.external_interfaces.giskard_new as giskardpy
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater

from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, StartSignalWaiter

world = BulletWorld()
v = VizMarkerPublisher()
robot = Object(name="hsrb", type=ObjectType.ROBOT, path="hsrb.urdf", pose=Pose([1, 2, 0]))

kitchen = Object(name="kitchen", type=ObjectType.ENVIRONMENT, path="isr-testbed.urdf")

