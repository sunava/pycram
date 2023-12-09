from pycram.process_module import simulated_robot, with_simulated_robot, real_robot, with_real_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper
import pycram.external_interfaces.giskard as giskardpy
from pycram.fluent_language_misc import failure_handling
import threading
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.plan_failures import TorsoFailure
from pycram.language import macros, par
import sys
#from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.enums import ObjectType

from pycram.ros.robot_state_updater import RobotStateUpdater
#from pycram.ros.joint_state_publisher import JointStatePublisher

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

world.set_gravity([0, 0, -9.8])
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])
kitchen = Object("kitchen", "environment", "kitchen.urdf")
robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])


giskardpy.init_giskard_interface()
giskardpy.sync_worlds()
RobotStateUpdater("/tf", "/giskard_joint_states")
#/pycram/viz_marker topic bei Marker Array


class HumanDescription:
    def __init__(self, name, fav_drink, shirt_color, gender):
        self.name = name
        self.fav_drink = fav_drink
        self.shirt_color = shirt_color
        self.gender = gender
        # characteristics to consider: height, hair color, and age. TODO: coordinate with Perception on what is easy to implement


with real_robot:
    handle_desig = ObjectPart(names=["kitchen_2/fridge_area_lower_drawer_handle"], part_of=kitchen_desig.resolve())

    drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(), robot_desig=robot_desig.resolve()).resolve()
    host = HumanDescription("vanessa", "Aperol", "black", "female") #TODO: 2h before Challenge change to real name of host
    doorbell = False
    #human1 = HumanDescription()
    #human2 = HumanDescription()

    while not doorbell: #TODO: replace with query to Knowledge
        dummy_input = input("waiting for bell: ")
        if dummy_input.lower() == "kling":
            print("i heard the doorbell")
            doorbell = True
            break
        else:
            print("no bellsound heard")

    if doorbell:
        #MoveTorsoAction([0.15]).resolve().perform()
        NavigateAction(target_locations=[Pose([-2, 0, 0])]).resolve().perform()
        #OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()





