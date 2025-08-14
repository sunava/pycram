from dataclasses import dataclass

from .base import BaseMotion
from ...datastructures.enums import Arms
from ...datastructures.pose import PoseStamped
from ...datastructures.world import World
from ...description import ObjectDescription
from ...external_interfaces.ik import request_ik
from ...plan import with_plan
from ...process_module import ProcessModuleManager
from ...robot_description import RobotDescription
from ...utils import _apply_ik
from ...world_concepts.world_object import Object
from ...world_reasoning import link_pose_for_joint_config


@with_plan
@dataclass
class OpeningMotion(BaseMotion):
    """
    Designator for opening container
    """

    object_part: ObjectDescription.Link
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    def perform(self):
        part_of_object = self.object_part.parent_entity

        container_joint_name = part_of_object.find_joint_above_link(self.object_part.name)
        lower_limit, upper_limit = part_of_object.get_joint_limits(container_joint_name)

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint_name: max(lower_limit, upper_limit - 0.05)}, self.object_part.name)

        self._move_arm_tcp(goal_pose, World.robot, self.arm)

        part_of_object.set_joint_position(container_joint_name, upper_limit)

    def _move_arm_tcp(target: PoseStamped, robot: Object, arm: Arms, tip_link: str = None) -> None:
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
class ClosingMotion(BaseMotion):
    """
    Designator for closing a container
    """

    object_part: ObjectDescription.Link
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    def perform(self):
        part_of_object = self.object_part.parent_entity

        container_joint_name = part_of_object.find_joint_above_link(self.object_part.name)
        lower_joint_limit = part_of_object.get_joint_limits(container_joint_name)[0]

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint_name: lower_joint_limit}, self.object_part.name)

        self._move_arm_tcp(goal_pose, World.robot, self.arm)

        part_of_object.set_joint_position(container_joint_name, lower_joint_limit)

    def _move_arm_tcp(target: PoseStamped, robot: Object, arm: Arms, tip_link: str = None) -> None:
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
