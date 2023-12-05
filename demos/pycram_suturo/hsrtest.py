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

from pycram.ros.robot_state_updater import RobotStateUpdater
#from pycram.ros.joint_state_publisher import JointStatePublisher

print(sys.meta_path)
world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

#broadcaster = TFBroadcaster()
#joint_publisher = JointStatePublisher("joint_states", 0.1)

world.set_gravity([0, 0, -9.8])
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])
kitchen = Object("kitchen", "environment", "kitchen.urdf")
robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
spawning_poses = {
    # 'bigknife': Pose([-0.95, 1.2, 1.3], [1, -1, 1, -1]),
    'bigknife': Pose([0.9, 0.6, 0.5], [0, 0, 0, -1]),
    # 'bread': Pose([-0.85, 0.9, 0.90], [0, 0, -1, 1])
    'bread': Pose([-0.85, 0.9, 0.90], [0, 0, -1, -1]),
    'board': Pose([-0.85, 0.9, 0.85], [0, 0, -1, -1]),
    'cocumber': Pose([-0.85, 0.9, 0.87], [0, 0, -1, -1])
}
bigknife = Object("bigknife", "bigknife", "big-knife.stl", spawning_poses["bigknife"])
cocumber = Object("cocumber", "cocumber", "cocumber.stl", spawning_poses["cocumber"])
board = Object("board", "board", "board.stl", spawning_poses["board"])
cocumber.set_color([0, 1, 0.04, 1])
board.set_color([0.4, 0.2, 0.06, 1])
bigknife_BO = BelieveObject(names=["bigknife"])
bread_BO = BelieveObject(names=["bread"])
cocumber_BO = BelieveObject(names=["cocumber"])
giskardpy.init_giskard_interface()
giskardpy.sync_worlds()
RobotStateUpdater("/tf", "/joint_states")

# giskardpy.achieve_joint_goal({"torso_lift_joint": 0.28})
import random

# Example usage:
# @failure_handling(5)
# def some_function():
#     numerator = 10
#     denominator = random.choice([0])  # Randomly chooses 0, 1, or 2
#     result = numerator / denominator  # This might raise a ZeroDivisionError
#     print(f"Result is {result}")
#     return result


def move_object():
    # Move to sink
    with par as s:
        # MoveTorsoAction([0.33]).resolve().perform()
        print("test")
        print("test1")


with real_robot:
    #some_function()
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    MoveTorsoAction([0.33]).resolve().perform()
    giskardpy.sync_worlds()
    grasp = robot_description.grasps.get_orientation_for_grasp("top")
    arm = "left"
    # print("pickuppose")
    # pickup_pose_knife = CostmapLocation(target=bigknife_BO.resolve(), reachable_for=robot_desig).resolve()
    # print("nav")
    # pickup_arm = pickup_pose_knife.reachable_arms[0]
    pipose = Pose([0.4, 0.6, 0], [0, 0, 0, -1])
    NavigateAction(target_locations=[pipose]).resolve().perform()
    print("pickup")
    PickUpAction(object_designator_description=bigknife_BO,
                 arms=["left"],
                 grasps=["top"]).resolve().perform()
    print("park")
    #
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    original_quaternion = (0, 0, 0, 1)
    rotation_axis = (0, 0, 1)
    rotation_quaternion = helper.axis_angle_to_quaternion(rotation_axis, 180)
    resulting_quaternion = helper.multiply_quaternions(original_quaternion, rotation_quaternion)
    nav_pose = Pose([-0.3, 0.9, 0.0], resulting_quaternion)
    NavigateAction(target_locations=[nav_pose]).resolve().perform()
    LookAtAction(targets=[cocumber_BO.resolve().pose]).resolve().perform()
