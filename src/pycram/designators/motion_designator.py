from abc import ABC, abstractmethod
from dataclasses import dataclass

from sqlalchemy.orm import Session
from .object_designator import ObjectDesignatorDescription, ObjectPart, RealObject
from ..designator import ResolutionError
from ..orm.base import ProcessMetaData
from ..plan_failures import PerceptionObjectNotFound
from ..process_module import ProcessModuleManager
from ..orm.motion_designator import (MoveMotion as ORMMoveMotion,
                                     MoveTCPMotion as ORMMoveTCPMotion, LookingMotion as ORMLookingMotion,
                                     MoveGripperMotion as ORMMoveGripperMotion, DetectingMotion as ORMDetectingMotion,
                                     OpeningMotion as ORMOpeningMotion, ClosingMotion as ORMClosingMotion,
                                     Motion as ORMMotionDesignator)
from ..datastructures.enums import ObjectType, Arms, GripperState

from typing_extensions import Dict, Optional, get_type_hints
from ..datastructures.pose import Pose
from ..tasktree import with_tree
from ..designator import BaseMotion

@dataclass
class MoveMotion(BaseMotion):
    """
    Moves the robot to a designated location
    """

    target: Pose
    """
    Location to which the robot should be moved
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.navigate().execute(self)
        # return ProcessModule.perform(self)

    def to_sql(self) -> ORMMoveMotion:
        return ORMMoveMotion()

    def insert(self, session, *args, **kwargs) -> ORMMoveMotion:
        motion = super().insert(session)
        pose = self.target.insert(session)
        motion.pose = pose
        session.add(motion)

        return motion


@dataclass
class MoveTCPMotion(BaseMotion):
    """
    Moves the Tool center point (TCP) of the robot
    """

    target: Pose
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

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_tcp().execute(self)

    def to_sql(self) -> ORMMoveTCPMotion:
        return ORMMoveTCPMotion(self.arm, self.allow_gripper_collision)

    def insert(self, session: Session, *args, **kwargs) -> ORMMoveTCPMotion:
        motion = super().insert(session)
        pose = self.target.insert(session)
        motion.pose = pose
        session.add(motion)

        return motion


@dataclass
class LookingMotion(BaseMotion):
    """
    Lets the robot look at a point
    """
    target: Pose

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.looking().execute(self)

    def to_sql(self) -> ORMLookingMotion:
        return ORMLookingMotion()

    def insert(self, session: Session, *args, **kwargs) -> ORMLookingMotion:
        motion = super().insert(session)
        pose = self.target.insert(session)
        motion.pose = pose
        session.add(motion)

        return motion


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

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_gripper().execute(self)

    def to_sql(self) -> ORMMoveGripperMotion:
        return ORMMoveGripperMotion(self.motion, self.gripper, self.allow_gripper_collision)

    def insert(self, session: Session, *args, **kwargs) -> ORMMoveGripperMotion:
        motion = super().insert(session)
        session.add(motion)

        return motion


@dataclass
class DetectingMotion(BaseMotion):
    """
    Tries to detect an object in the FOV of the robot
    """

    object_type: ObjectType
    """
    Type of the object that should be detected
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        world_object = pm_manager.detecting().execute(self)
        if not world_object:
            raise PerceptionObjectNotFound(
                f"Could not find an object with the type {self.object_type} in the FOV of the robot")
        if ProcessModuleManager.execution_type == "real":
            return RealObject.Object(world_object.name, world_object.obj_type,
                                                  world_object, world_object.get_pose())

        return ObjectDesignatorDescription.Object(world_object.name, world_object.obj_type,
                                                  world_object)

    def to_sql(self) -> ORMDetectingMotion:
        return ORMDetectingMotion(self.object_type)

    def insert(self, session: Session, *args, **kwargs) -> ORMDetectingMotion:
        motion = super().insert(session)
        session.add(motion)

        return motion


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
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_arm_joints().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class WorldStateDetectingMotion(BaseMotion):
    """
    Detects an object based on the world state.
    """

    object_type: ObjectType
    """
    Object type that should be detected
    """

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.world_state_detecting().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class MoveJointsMotion(BaseMotion):
    """
    Moves any joint on the robot
    """

    names: list
    """
    List of joint names that should be moved 
    """
    positions: list
    """
    Target positions of joints, should correspond to the list of names
    """

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_joints().execute(self)

    def to_sql(self) -> ORMMotionDesignator:
        pass

    def insert(self, session: Session, *args, **kwargs) -> ORMMotionDesignator:
        pass


@dataclass
class OpeningMotion(BaseMotion):
    """
    Designator for opening container
    """

    object_part: ObjectPart.Object
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.open().execute(self)

    def to_sql(self) -> ORMOpeningMotion:
        return ORMOpeningMotion(self.arm)

    def insert(self, session: Session, *args, **kwargs) -> ORMOpeningMotion:
        motion = super().insert(session)
        op = self.object_part.insert(session)
        motion.object = op
        session.add(motion)

        return motion


@dataclass
class ClosingMotion(BaseMotion):
    """
    Designator for closing a container
    """

    object_part: ObjectPart.Object
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.close().execute(self)

    def to_sql(self) -> ORMClosingMotion:
        return ORMClosingMotion(self.arm)

    def insert(self, session: Session, *args, **kwargs) -> ORMClosingMotion:
        motion = super().insert(session)
        op = self.object_part.insert(session)
        motion.object = op
        session.add(motion)

        return motion

#
# class GraspingDishwasherHandleMotion(MotionDesignatorDescription):
#     """
#     Designator for grasping the dishwasher handle
#     """
#
#     @dataclasses.dataclass
#     class Motion(MotionDesignatorDescription.Motion):
#
#         handle_name: str
#         """
#         Name of the handle to grasp
#         """
#         arm: str
#         """
#         Arm that should be used
#         """
#
#         @with_tree
#         def perform(self):
#             pm_manager = ProcessModuleManager.get_manager()
#             return pm_manager.grasp_dishwasher_handle().execute(self)
#
#     def __init__(self, handle_name: str, arm: str, resolver: Optional[Callable] = None):
#         """
#         Lets the robot grasp the dishwasher handle specified by the given parameter.
#         :param handle_name: name of the handle that should be grasped
#         :param arm: Arm that should be used
#         :param resolver: An alternative resolver
#         """
#         super().__init__(resolver)
#         self.cmd: str = 'open'
#         self.handle_name = handle_name
#         self.arm: str = arm
#
#     def ground(self) -> Motion:
#         """
#         Default resolver for grasping motion designator, returns a resolved motion designator for the input parameters.
#         :return: A resolved motion designator
#         """
#         return self.Motion(self.cmd, self.handle_name, self.arm)
#
#
# class HalfOpeningDishwasherMotion(MotionDesignatorDescription):
#     """
#     Designator for half opening the dishwasher door to a given degree.
#     """
#
#     @dataclasses.dataclass
#     class Motion(MotionDesignatorDescription.Motion):
#         handle_name: str
#         """
#         Name of the dishwasher handle which is grasped
#         """
#         goal_state_half_open: float
#         """
#         Goal state of the door, defining the degree to open the door
#         """
#         arm: str
#         """
#         Arm that should be used
#         """
#
#         @with_tree
#         def perform(self):
#             pm_manager = ProcessModuleManager.get_manager()
#             return pm_manager.half_open_dishwasher().execute(self)
#
#     def __init__(self, handle_name: str, goal_state_half_open: float, arm: str, resolver: Optional[Callable] = None):
#         """
#         Lets the robot open the dishwasher to a given degree. This motion designator assumes that the handle
#         is already grasped.
#         :param handle_name: Name of the handle which is grasped
#         :param goal_state_half_open: degree to which the dishwasher door should be opened
#         :param arm: Arm that should be used
#         :param resolver: An alternative resolver
#         """
#         super().__init__(resolver)
#         self.cmd: str = 'open'
#         self.handle_name = handle_name
#         self.goal_state_half_open = goal_state_half_open
#         self.arm: str = arm
#
#     def ground(self) -> Motion:
#         """
#         Default resolver for opening motion designator, returns a resolved motion designator for the input parameters.
#         :return: A resolved motion designator
#         """
#         return self.Motion(self.cmd, self.handle_name, self.goal_state_half_open, self.arm)
#
# class MoveArmAroundMotion(MotionDesignatorDescription):
#     """
#     Designator for moving the arm around the dishwasher to further open the door.
#     """
#
#     @dataclasses.dataclass
#     class Motion(MotionDesignatorDescription.Motion):
#         handle_name: str
#         """
#         Name of the dishwasher handle which was grasped
#         """
#         arm: str
#         """
#         Arm that should be used
#         """
#
#         @with_tree
#         def perform(self):
#             pm_manager = ProcessModuleManager.get_manager()
#             return pm_manager.move_arm_around_dishwasher().execute(self)
#
#     def __init__(self, handle_name: str, arm: str, resolver: Optional[Callable] = None):
#         """
#         Lets the robot move its arm around the dishwasher door for the next opening action. This motion designator assumes that the handle
#         is not grasped anymore.
#         :param handle_name: Name of the handle which was grasped
#         :param arm: Arm that should be used
#         :param resolver: An alternative resolver
#         """
#         super().__init__(resolver)
#         self.cmd: str = 'open'
#         self.handle_name = handle_name
#         self.arm: str = arm
#
#     def ground(self) -> Motion:
#         """
#         Default resolver for moving arm around motion designator, returns a resolved motion designator for the input parameters.
#         :return: A resolved motion designator
#         """
#         return self.Motion(self.cmd, self.handle_name, self.arm)
#
# class FullOpeningDishwasherMotion(MotionDesignatorDescription):
#     """
#     Designator for fully opening the dishwasher. Assumes that the door is already half opened and the arm is in the right position.
#     """
#
#     @dataclasses.dataclass
#     class Motion(MotionDesignatorDescription.Motion):
#         handle_name: str
#         """
#         Name of the dishwasher handle which was grasped
#         """
#         door_name: str
#         """
#         Name of the dishwasher door which should be opened
#         """
#         goal_state_full_open: float
#         """
#         Goal state of the door, defining the degree to open the door
#         """
#         arm: str
#         """
#         Arm that should be used
#         """
#
#         @with_tree
#         def perform(self):
#             pm_manager = ProcessModuleManager.get_manager()
#             return pm_manager.full_open_dishwasher().execute(self)
#
#     def __init__(self, handle_name: str, door_name: str, goal_state_full_open: float, arm: str, resolver: Optional[Callable] = None):
#         """
#         Lets the robot fully open the dishwasher door. This motion designator assumes that the dishwasher is already half open and the arm is in the right position.
#         :param handle_name: Name of the handle, which was grasped
#         :param door_name: Name of the door, which should be opened
#         :param goal_state_full_open: degree to which the dishwasher door should be opened
#         :param arm: Arm that should be used
#         :param resolver: An alternative resolver
#         """
#         super().__init__(resolver)
#         self.cmd: str = 'open'
#         self.handle_name = handle_name
#         self.door_name = door_name
#         self.goal_state_full_open = goal_state_full_open
#         self.arm: str = arm
#
#     def ground(self) -> Motion:
#         """
#         Default resolver for opening motion designator, returns a resolved motion designator for the input parameters.
#         :return: A resolved motion designator
#         """
#         return self.Motion(self.cmd, self.handle_name, self.door_name, self.goal_state_full_open, self.arm)