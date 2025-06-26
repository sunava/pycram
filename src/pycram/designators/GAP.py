from __future__ import annotations

import math
import numpy as np
from abc import abstractmethod
from dataclasses import dataclass
from datetime import timedelta
from time import sleep
from typing import Optional, Union, Iterable, Tuple

from typing_extensions import Any

from .. import utils
from .motion_designator import MoveTCPMotion, MoveToolMotion, MoveTCPWaypointsMotion
from ..datastructures.enums import Arms, Grasp, AxisIdentifier, MovementType
from ..datastructures.partial_designator import PartialDesignator
from ..datastructures.pose import PoseStamped
from ..datastructures.world import World
from ..designator import ActionDescription
from ..has_parameters import has_parameters
from ..local_transformer import LocalTransformer
from ..plan import with_plan
from ..robot_description import RobotDescription
from ..world_concepts.world_object import Object

from pycram.external_interfaces import giskard


@dataclass
class GAP(ActionDescription):
    object_: Object
    tool: Object
    arm: Arms
    technique: Optional[str] = None

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        pass

    @abstractmethod
    def plan(self) -> None:
        ...


@has_parameters
@dataclass
class MixingAction(GAP):
    def plan(self) -> None:
        lt = LocalTransformer()
        obj = self.object_
        pose = lt.transform_to_object_frame(obj.pose, obj)
        height_offset = obj.size[2] + 0.05

        for t in range(20):  # 2 * steps, with steps=10
            p = pose.copy()
            r, a, h = 0.0035 * t, math.radians(30) * t, 0.001 * t
            p.pose.position.x += r * math.cos(a)
            p.pose.position.y += r * math.sin(a)
            p.pose.position.z += h

            spiral = lt.transform_pose(p, "map")
            spiral.pose.position.z += height_offset
            World.current_world.add_vis_axis(spiral)
            # MoveTCPMotion(spiral, self.arm).perform()

        World.current_world.remove_vis_axis()

    @classmethod
    @with_plan
    def description(cls, object_: Union[Iterable[Object], Object], tool: Union[Iterable[Object], Object],
                    arm: Optional[Union[Iterable[Arms], Arms]] = None,
                    technique: Optional[Union[Iterable[str], str]] = None):
        return PartialDesignator(cls, object_=object_, tool=tool, arm=arm, technique=technique)


@dataclass
class CuttingAction(GAP):
    slice_thickness: Optional[float] = 0.005

    def plan(self) -> None:
        if self.technique is None:
            self.technique = "Slicing"
        lt = LocalTransformer()
        obj = self.object_
        tool = self.tool
        pose = lt.transform_to_object_frame(obj.pose, obj)

        height = obj.size[2]
        length = max(obj.size[0], obj.size[1])
        length_tool = max(tool.size[0], tool.size[1])
        print(length)
        num_slices, start_offset = self.calculate_slices(length)
        print(num_slices, start_offset)
        slice_coordinates = [start_offset + i * self.slice_thickness for i in range(num_slices)]
        slice_poses = []
        for i, x in enumerate(slice_coordinates):
            tmp_pose = pose.copy()
            if i == len(slice_coordinates) - 1:
                tmp_pose.pose.position.x = x
            else:
                tmp_pose.pose.position.x = x
            slice_poses.append(tmp_pose)

        cutting_poses = []
        for slice_pose in slice_poses:
            pose_a = obj.pose
            pose_b = World.robot.pose
            angle, angle_y = self.get_rotation_offset_from_axis_preference(pose_a, pose_b)
            direction = -1 if angle_y >= 0 else 1
            tool_halve = length_tool / 2

            slice_pose.pose.position.y += direction * (tool_halve + (tool_halve / 2))

            new_pose = self.perpendicular_pose(slice_pose=slice_pose, angle=angle)

            q1 = utils.axis_angle_to_quaternion([0, 1, 0], 15)
            new_pose.rotate_by_quaternion(q1)



            supreme_cut = new_pose.copy()
            # new_pose.pose.position.x += 0.01
            supreme_cut.pose.position.x -= 0.02
            supreme_cut.pose.position.y += 0.06
            supreme_cut_rotate = supreme_cut.copy()

            #

            q2 = utils.axis_angle_to_quaternion([1, 0, 0], 47)

            supreme_cut_rotate.rotate_by_quaternion(q2)




            supreme_cut_rotate.rotate_by_quaternion(q1)

            final_new_pose = lt.transform_pose(new_pose, "map")
            m_supreme_cut_r = lt.transform_pose(supreme_cut_rotate, "map")
            m_supreme_cut = lt.transform_pose(supreme_cut, "map")
            # World.current_world.add_vis_axis(final_pose)

            lift_pose = final_new_pose.copy()
            lift_pose.pose.position.z += 2 * height

            adjusted_lift_pose = m_supreme_cut.copy()
            adjusted_lift_pose.pose.position.z += 2 * height + 0.02
            # World.current_world.add_vis_axis(lift_pose)
            final_new_pose.pose.position.z -= 0.1

            final_saw_pose = final_new_pose.copy()
            final_saw_pose.pose.position.x += 0.08

            final_saw_rotate = final_saw_pose.copy()

            final_saw_rotate.pose.position.y -= 0.005
            q2 = utils.axis_angle_to_quaternion([1, 0, 0], 90)
            final_saw_rotate.rotate_by_quaternion(q2)

            final_saw_rotate_back = final_saw_rotate.copy()
            final_saw_rotate_back.pose.position.x -= 0.08
            final_saw_rotate_back.pose.position.y -= 0.02

            final_saw_rotate_back_lift = final_saw_rotate_back.copy()
            final_saw_rotate_back_lift.pose.position.z = lift_pose.pose.position.z

            m_supreme_cut.pose.position.z -= 0.1
            m_supreme_cut_r.pose.position.z -= 0.1
            cutting_poses_tmp = []
            cutting_poses_tmp.append(lift_pose)
            cutting_poses_tmp.append(final_new_pose)
            cutting_poses_tmp.append(final_saw_pose)
            cutting_poses_tmp.append(final_saw_rotate)
            cutting_poses_tmp.append(final_saw_rotate_back)
            cutting_poses_tmp.append(final_saw_rotate_back_lift)
            cutting_poses_tmp.append(lift_pose)
            cutting_poses.append(cutting_poses_tmp)
            # cutting_poses.append(m_supreme_cut)
            # cutting_poses.append(m_supreme_cut)
            # cutting_poses.append(m_supreme_cut_r)
            # # cutting_poses.append(m_supreme_cut)
            # cutting_poses.append(adjusted_lift_pose)
            # MoveToolMotion(self.tool, True,lift_pose, self.arm, allow_gripper_collision=True,
            #                movement_type=MovementType.CARTESIAN).perform()
            # MoveToolMotion(self.tool, True,final_pose, self.arm, allow_gripper_collision=True,
            #                movement_type=MovementType.CARTESIAN).perform()
            # MoveToolMotion(self.tool, True, lift_pose, self.arm, allow_gripper_collision=False,
            #                movement_type=MovementType.CARTESIAN).perform()
        # print("moveTCPway")
        # cutting_poses[:] = cutting_poses[len(cutting_poses) // 3: 2 * len(cutting_poses) // 3]

        # cutting_poses = cutting_poses[1:][: len(cutting_poses[1:]) // 3]
        cutting_poses = cutting_poses[: len(cutting_poses) // 2]
        cutting_poses = [pose for sublist in cutting_poses for pose in sublist]


        World.current_world.add_vis_axis(cutting_poses[0])
        tip_link = self.tool.name
        root_link = "torso_lift_link"

        giskard.avoid_all_collisions()
        giskard.allow_gripper_collision(Arms.RIGHT)
        print(tip_link)
        print(root_link)

        giskard.achieve_cartesian_goal_sequence(cutting_poses, tip_link, root_link, Arms.RIGHT)
        # MoveTCPWaypointsMotion(cutting_poses, self.arm, allow_gripper_collision=True, tip_link=tool.tip_link)
        # print("moveTCPwa11y")

    @classmethod
    @with_plan
    def description(cls, object_: Union[Iterable[Object], Object], tool: Union[Iterable[Object], Object],
                    arm: Optional[Union[Iterable[Arms], Arms]] = None,
                    technique: Optional[Union[Iterable[str], str]] = None, slice_thickness: Optional[float] = 0.03):
        return PartialDesignator(cls, object_=object_, tool=tool, arm=arm, technique=technique,
                                 slice_thickness=slice_thickness)

    def calculate_slices(self, obj_length):
        if self.technique == 'Halving':
            return 1, 0
        if self.technique in ['Cutting Action', 'Sawing', 'Paring', 'Cutting', 'Carving', 'Slicing']:
            num_slices = int(obj_length // self.slice_thickness)
            print(num_slices)
            start_offset = (-obj_length / 2) + (self.slice_thickness / 2)
            return num_slices, start_offset
        return 0, 0

    @staticmethod
    def perpendicular_pose(slice_pose, angle) -> PoseStamped:
        pose_rotated = slice_pose
        rotation_quaternion = utils.axis_angle_to_quaternion([0, 0, 1], angle)
        pose_rotated.rotate_by_quaternion(rotation_quaternion)
        return pose_rotated

    @staticmethod
    def get_rotation_offset_from_axis_preference(pose_a, pose_b: PoseStamped) -> Tuple[int, float]:
        """
        Compute a discrete rotation offset (-90 or 90 degrees) to align this pose's local axes with the direction
        toward a target pose, based on which axis (X or Y) is more aligned.

        :param pose_a: The source pose.
        :param pose_b: The target pose to align with.
        :return: Tuple of (rotation offset in degrees, signed angle difference in radians for Y axis).
        """
        fx, ax = pose_a.is_facing_2d_axis(pose_b, axis=AxisIdentifier.X)
        fy, ay = pose_a.is_facing_2d_axis(pose_b, axis=AxisIdentifier.Y)

        return (-90 if abs(ax) > abs(ay) else 90), ay


@dataclass
class PouringAction(GAP):
    angle: Optional[float] = 90

    def plan(self) -> None:
        lt = LocalTransformer()
        gripper_frame = World.robot.get_link_tf_frame("base_link")
        grasp_rot = RobotDescription.current_robot_description.get_arm_chain(self.arm).end_effector.get_grasp(
            Grasp.FRONT, None, False)

        pose = lt.transform_pose(self.object_.pose, gripper_frame)
        pose.pose.position.x += 0.009
        pose.pose.position.y -= 0.125
        pose.pose.position.z += 0.17

        pose = lt.transform_pose(lt.transform_pose(pose, "map"), gripper_frame)
        pose.orientation = grasp_rot
        pose = lt.transform_pose(pose, "map")

        World.current_world.add_vis_axis(pose)
        # MoveTCPMotion(pose, self.arm, allow_gripper_collision=False, movement_type=MovementType.CARTESIAN).perform()

        pour_pose = pose.copy()
        pour_pose.rotate_by_quaternion(utils.axis_angle_to_quaternion([1, 0, 0], -self.angle))
        World.current_world.add_vis_axis(pour_pose)

        # MoveTCPMotion(pour_pose, self.arm, allow_gripper_collision=False,movement_type=MovementType.CARTESIAN).perform()
        sleep(3)
        # MoveTCPMotion(pose, self.arm, allow_gripper_collision=False, movement_type=MovementType.CARTESIAN).perform()

        World.current_world.remove_vis_axis()

    @classmethod
    @with_plan
    def description(cls, object_: Union[Iterable[Object], Object], tool: Union[Iterable[Object], Object],
                    arm: Optional[Union[Iterable[Arms], Arms]] = None,
                    technique: Optional[Union[Iterable[str], str]] = None,
                    angle: Optional[Union[Iterable[float], float]] = 90):
        return PartialDesignator(cls, object_=object_, tool=tool, arm=arm, technique=technique, angle=angle)


PouringActionDescription = PouringAction.description
MixingActionDescription = MixingAction.description
CuttingActionDescription = CuttingAction.description
