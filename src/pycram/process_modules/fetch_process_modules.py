from .default_process_modules import *
from ..utils import _apply_ik
from .default_process_modules import _move_arm_tcp


class FetchMoveHead(DefaultMoveHead):
    """
    Moves the head of the iai_Fetch robot to look at a specified point in the world coordinate frame.
    This point can be a position or an object, and the orientation is calculated based on the
    robot's base and camera alignment.
    """
    def _execute(self, desig):
        # Fetch-specific neck configuration
        target = desig.target
        robot = World.robot
        local_transformer = LocalTransformer()

        # Fetch robot uses head_pan_joint and head_tilt_joint
        neck = RobotDescription.current_robot_description.get_neck()
        pan_joint = neck["pan_joint"]
        tilt_joint = neck["tilt_joint"]

        # Transform the target pose to the head pan link frame
        pose_in_pan = local_transformer.transform_pose(target,
                                                       robot.get_link_tf_frame("head_pan_link")).position_as_list()
        pose_in_tilt = local_transformer.transform_pose(target,
                                                        robot.get_link_tf_frame("head_tilt_link")).position_as_list()

        # Calculate the new pan angle
        new_pan = np.arctan2(pose_in_pan[1], pose_in_pan[0])

        # Handle tilt offset if it exists
        tilt_offset = RobotDescription.current_robot_description.get_offset(tilt_joint)
        tilt_offset_rotation = tilt_offset.rotation if tilt_offset else [0, 0, 0]
        rotation_tilt_offset = R.from_euler('xyz', tilt_offset_rotation).apply(pose_in_tilt)

        # Calculate the new tilt angle
        new_tilt = -np.arctan2(rotation_tilt_offset[2],
                               np.sqrt(rotation_tilt_offset[0] ** 2 + rotation_tilt_offset[1] ** 2))

        # Get current joint positions
        current_pan = robot.get_joint_position(pan_joint)
        current_tilt = robot.get_joint_position(tilt_joint)

        # Set the new joint positions
        robot.set_joint_position(pan_joint, new_pan)
        robot.set_joint_position(tilt_joint, new_tilt)


class FetchManager(DefaultManager):

    def __init__(self):
        super().__init__()
        self.robot_name = "fetch"

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return FetchMoveHead(self._looking_lock)
