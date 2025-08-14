from dataclasses import dataclass
from typing import Optional, Dict, List

from .base import BaseMotion
from ...datastructures.enums import Arms, GripperState, MovementType, WaypointsMovementType, Frame
from ...datastructures.grasp import GraspDescription
from ...datastructures.pose import PoseStamped
from ...datastructures.world import World
from ...external_interfaces.ik import request_ik
from ...failure_handling import try_motion
from ...failures import ToolPoseNotReachedError
from ...local_transformer import LocalTransformer
from ...plan import with_plan
from ...process_module import ProcessModuleManager
from ...robot_description import RobotDescription
from ...utils import _apply_ik
from ...world_concepts.world_object import Object

# class ReachMotion(BaseMotion):
#     """
#     """
#     object_designator: Object
#     """
#     Object designator_description describing the object that should be picked up
#     """
#     arm: Arms
#     """
#     The arm that should be used for pick up
#     """
#     grasp_description: GraspDescription
#     """
#     The grasp description that should be used for picking up the object
#     """
#     movement_type: MovementType = MovementType.CARTESIAN
#     """
#     The type of movement that should be performed.
#     """
#
#     def perform(self):
#         target_pose = self.object_designator.get_grasp_pose(self.end_effector, self.grasp_description)
#         target_pose.rotate_by_quaternion(self.end_effector.grasps[self.grasp_description])
#         #todo: Symantic World- LocalTransformer
#         target_pre_pose = LocalTransformer().translate_pose_along_local_axis(target_pose,
#                                                                              self.end_effector.get_approach_axis(),
#                                                                              -self.object_designator.get_approach_offset())
#
#         pose = self.local_transformer.transform_pose(target_pre_pose, Frame.Map.value)
#
#         MoveTCPMotion(pose, self.arm, allow_gripper_collision=False, movement_type=self.movement_type).perform()


@with_plan
@dataclass
class MoveArmJointsMotion(BaseMotion):
    """
    Moves the joints of each arm into the given position
    """

    left_arm_poses: Optional[Dict[str, float]] = None
    """
    Target positions for the left arm joints
    """
    right_arm_poses: Optional[Dict[str, float]] = None
    """
    Target positions for the right arm joints
    """

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager()
        return pm_manager.move_arm_joints().execute(self)


@with_plan
@dataclass
class MoveGripperMotion(BaseMotion):
    """
    Opens or closes the gripper
    """

    motion: GripperState
    """
    Motion that should be performed, either 'open' or 'close'
    """
    gripper: Arms
    """
    Name of the gripper that should be moved
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper is allowed to collide with something
    """

    def perform(self):
        robot_description = RobotDescription.current_robot_description
        gripper = self.gripper
        arm_chain = robot_description.get_arm_chain(gripper)
        if arm_chain.end_effector.gripper_object_name is not None:
            robot = World.current_world.get_object_by_name(arm_chain.end_effector.gripper_object_name)
        else:
            robot = World.robot
        motion = self.motion
        robot.set_multiple_joint_positions(arm_chain.get_static_gripper_state(motion))


@with_plan
@dataclass
class MoveTCPMotion(BaseMotion):
    """
    Moves the Tool center point (TCP) of the robot
    """

    target: PoseStamped
    """
    Target pose to which the TCP should be moved
    """
    arm: Arms
    """
    Arm with the TCP that should be moved to the target
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper can collide with something
    """
    movement_type: Optional[MovementType] = MovementType.CARTESIAN
    """
    The type of movement that should be performed.
    """

    def perform(self):
        target = self.target
        robot = World.robot
        arm = self.arm
        self.move_arm_tcp(target=target, robot=robot, arm=arm)

    def move_arm_tcp(self, target: PoseStamped, robot: Object, arm: Arms, tip_link: str = None) -> None:
        """
        Calls the ik solver to calculate the inverse kinematics of the arm and then sets the joint states accordingly.

        :param target: Target pose to which the end-effector should move.
        :param robot: Robot object representing the robot.
        :param arm: Which arm to move
        """
        if tip_link is None:
            tip_link = RobotDescription.current_robot_description.get_arm_chain(arm).get_tool_frame()

        joints = RobotDescription.current_robot_description.get_arm_chain(arm).joints

        inv = request_ik(target, robot, joints, tip_link)
        _apply_ik(robot, inv)

@with_plan
@dataclass
class MoveTCPWaypointsMotion(BaseMotion):
    """
    Moves the Tool center point (TCP) of the robot
    """

    waypoints: List[PoseStamped]
    """
    Waypoints the TCP should move along 
    """
    arm: Arms
    """
    Arm with the TCP that should be moved to the target
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper can collide with something
    """
    movement_type: WaypointsMovementType = WaypointsMovementType.ENFORCE_ORIENTATION_FINAL_POINT
    """
    The type of movement that should be performed.
    """

    def perform(self):
        waypoints = self.waypoints
        robot = World.robot
        arm = self.arm
        for waypoint in waypoints:
            self.move_arm_tcp(target=waypoint, robot=robot, arm=arm)
    def move_arm_tcp(self, target: PoseStamped, robot: Object, arm: Arms, tip_link: str = None) -> None:
        """
        Calls the ik solver to calculate the inverse kinematics of the arm and then sets the joint states accordingly.

        :param target: Target pose to which the end-effector should move.
        :param robot: Robot object representing the robot.
        :param arm: Which arm to move
        """
        if tip_link is None:
            tip_link = RobotDescription.current_robot_description.get_arm_chain(arm).get_tool_frame()

        joints = RobotDescription.current_robot_description.get_arm_chain(arm).joints

        inv = request_ik(target, robot, joints, tip_link)
        _apply_ik(robot, inv)
