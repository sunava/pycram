import time
from math import sqrt

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
world = BulletWorld("DIRECT")

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Initialize Giskard interface for motion planning



#giskardpy.spawn_kitchen()
# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
giskardpy.init_giskard_interface()
# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

#giskardpy.removing_of_objects()
#giskardpy.sync_worlds()
robot.set_color([0.5, 0.5, 0.9, 1])


# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "couch-kitchen.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)

# Define a breakfast cereal object
#breakfast_cereal = Object("breakfast_cereal", "breakfast_cereal", "breakfast_cereal.stl", pose=Pose([4.8, 2.6, 0.87]),color=[0, 1, 0, 1])
# fork = Object("Fork", "fork", "spoon.stl", pose=Pose([-2.8, 2.3, 0.368], object_orientation), color=[1, 0, 0, 1])
# spoon = Object("Spoon", "spoon", "spoon.stl", pose=Pose([-2.5, 2.3, 0.368], object_orientation), color=[0, 1, 0, 1])
# metalmug = Object("Metalmug", "metalmug", "bowl.stl", pose=Pose([-3.1, 2.3, 0.39]), color=[0, 1, 0, 1])
# plate = Object("Plate", "plate", "board.stl", pose=Pose([-2.2, 2.3, 0.368], object_orientation), color=[0, 1, 0, 1])
#bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([4.8, 2.6, 0.87]), color=[0, 1, 0, 1])

# Set the world's gravity
#world.set_gravity([0.0, 0.0, 9.81])

with ((real_robot)):
    rospy.loginfo("Starting demo")
    TalkingMotion("Starting demo").resolve().perform()
    print(f"{robot.get_joint_limits('arm_lift_joint')}")
    MoveGripperMotion(motion="open", gripper="left").resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()
   # ParkArmsAction([Arms.LEFT]).resolve().perform()
   # MoveTorsoAction([0.2]).resolve().perform()
    print(f" arm_flex: {robot.get_joint_state('arm_flex_joint')}")
    print(f" arm_roll: {robot.get_joint_state('arm_roll_joint')}")
    print(f" wrist_flex: {robot.get_joint_state('wrist_flex_joint')}")
    print(f" wrist_roll: {robot.get_joint_state('wrist_roll_joint')}")
   # print(f" hand_distal: {robot.get_joint_state('hand_l_distal_link')}")
   # print(f" hand_spring: {robot.get_joint_state('hand_l_spring_proximal_link')}")
   # print(f" hand_palm: {robot.get_joint_state('hand_palm_link')}")
   # print(f" gripper_tool_frame: {robot.get_joint_state('gripper_tool_frame')}")
    print(f" hand_proximal: {robot.get_joint_state('hand_l_proximal_joint')}")
    print(f" hand_motor: {robot.get_joint_state('hand_motor_joint')}")
    print(f" arm_lift: {robot.get_joint_state('arm_lift_joint')}")
    print(f"print all: {robot.get_complete_joint_state()}")

    #MoveGripperMotion(motion="open", gripper="left").resolve().perform()
    TalkingMotion("Moving now").resolve().perform()
    PlaceGivenObjAction([Arms.LEFT]).resolve().perform()

    TalkingMotion("Grab object now!").resolve().perform()
    time.sleep(5)
    MoveGripperMotion(motion="close", gripper="left").resolve().perform()
    print(f" arm_lift2: {robot.get_joint_state('arm_lift_joint')}")
    print("finished")

    # arm_joints = ["arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    # arm_links = ["arm_flex_link", "arm_roll_link", "wrist_flex_link", "wrist_roll_link"]
    # arm_carry = {"park": [0, 1.5, -1.85, 0]}
    # gripper_links = ["hand_l_distal_link", "hand_l_spring_proximal_link", "hand_palm_link",
    #                  "hand_r_distal_link", "hand_r_spring_proximal_link", "hand_gripper_tool_frame"]
    # gripper_joints = ["hand_l_proximal_joint", "hand_r_proximal_joint", "hand_motor_joint"]