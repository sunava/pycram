import numpy as np
import rospy
from tmc_control_msgs.msg import GripperApplyEffortActionGoal

from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.designators.motion_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot, semi_real_robot, real_robot
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
import pycram.external_interfaces.giskard as giskardpy
import rospy

from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.designators.motion_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot, semi_real_robot, real_robot
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
import pycram.external_interfaces.giskard as giskardpy

from geometry_msgs.msg import Point
# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

import pybullet as p
#robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([1, 2, 0]))
#robot.set_color([0.5, 0.5, 0.9, 1])

id = BulletWorld.current_bullet_world.add_rigid_box(Pose(), (0.5,0.5,0,5), [1,0,0,1])
box_object = Object("box" + "_" + str(rospy.get_time()), "box", pose=Pose(), color=[1,0,0,1], id=id,
                    customGeom={"size": [0.5, 0.5, 0.5]})
#box_object.set_pose(Pose())
#box_desig = ObjectDesignatorDescription.Object(box_object.name, box_object.type, box_object)

#milk = Object("milk", "milk", "milk.stl", pose=Pose([4.8, 2.6, 0.87]),color=[0, 1, 0, 1])
#print (milk)
#print(box_object)