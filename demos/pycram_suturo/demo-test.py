from pycram.bullet_world import BulletWorld, Object
from pycram.enums import ObjectType
from pycram.external_interfaces import giskard
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper

world = BulletWorld()
robot = Object("pr2", "robot", "../../resources/" + robot_description.name + ".urdf")
giskard.achieve_joint_goal({"torso_lift_joint": 0.28})