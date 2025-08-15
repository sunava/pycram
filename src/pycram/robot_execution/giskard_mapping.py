from pycram.datastructures.world import World
from pycram.external_interfaces import giskard
from pycram.failures import NavigationGoalNotReachedError
from pycram.robot_description import RobotDescription

class GiskardMappings:
    def map_MoveMotion(self, motion):
        giskard.avoid_all_collisions()
        giskard.achieve_cartesian_goal(
            motion.target,
            RobotDescription.current_robot_description.base_link,
            "map"
        )
        if not World.current_world.robot.pose.almost_equal(motion.target, 0.05, 3):
            raise NavigationGoalNotReachedError(World.current_world.robot.pose, motion.target)

    def map_LookingMotion(self, motion):
        # dein Code hier …
        pass

    def map_MoveTCPMotion(self, motion):
        # dein Code hier …
        pass

def map_LookingMotion(motion):
    target = motion.target
    robot = World.robot

    local_transformer = LocalTransformer()
    neck = RobotDescription.current_robot_description.get_neck()
    pan_link = neck["yaw"][0]
    tilt_link = neck["pitch"][0]
    pan_joint = neck["yaw"][1]
    tilt_joint = neck["pitch"][1]

    pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame(pan_link)).position.to_list()
    pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame(tilt_link)).position.to_list()
    new_pan = np.arctan2(pose_in_pan[1], pose_in_pan[0])

    tilt_offset = RobotDescription.current_robot_description.get_offset(tilt_joint)
    quaternion_list = [0, 0, 0, 1]
    if tilt_offset:
        q = tilt_offset.pose.orientation
        quaternion_list = [q.x, q.y, q.z, q.w]

    tilt_offset_rotation = euler_from_quaternion(quaternion_list, axes='sxyz')
    adjusted_pose_in_tilt = R.from_euler('xyz', tilt_offset_rotation).apply(pose_in_tilt)
    new_tilt = -np.arctan2(adjusted_pose_in_tilt[2], np.sqrt(adjusted_pose_in_tilt[0] ** 2 + adjusted_pose_in_tilt[1] ** 2))

    if RobotDescription.current_robot_description.name in {"iCub", "tiago_dual"}:
        new_tilt = -new_tilt

    current_pan = robot.get_joint_position(pan_joint)
    current_tilt = robot.get_joint_position(tilt_joint)

    giskard.avoid_all_collisions()
    giskard.achieve_joint_goal({
        pan_joint: new_pan + current_pan,
        tilt_joint: new_tilt + current_tilt
    })

def map_MoveTCPMotion(motion):
    lt = LocalTransformer()
    pose_in_map = lt.transform_pose(motion.target, "map")
    tip_link = RobotDescription.current_robot_description.get_arm_chain(motion.arm).get_tool_frame()
    root_link = "map"

    gripper_that_can_collide = motion.arm if motion.allow_gripper_collision else None
    if motion.allow_gripper_collision:
        giskard.allow_gripper_collision(motion.arm)

    if motion.movement_type == MovementType.STRAIGHT_TRANSLATION:
        giskard.achieve_straight_translation_goal(pose_in_map.position.to_list(), tip_link, root_link)
    elif motion.movement_type == MovementType.STRAIGHT_CARTESIAN:
        giskard.achieve_straight_cartesian_goal(pose_in_map, tip_link, root_link)
    elif motion.movement_type == MovementType.TRANSLATION:
        giskard.achieve_translation_goal(pose_in_map.position.to_list(), tip_link, root_link)
    elif motion.movement_type == MovementType.CARTESIAN:
        giskard.achieve_cartesian_goal(pose_in_map, tip_link, root_link,
                                       grippers_that_can_collide=gripper_that_can_collide)

    if not World.current_world.robot.get_link_pose(tip_link).almost_equal(motion.target, 0.3, 3):
        raise ToolPoseNotReachedError(World.current_world.robot.get_link_pose(tip_link), motion.target)

def map_MoveArmJointsMotion(motion):
    joint_goals = {}
    if motion.left_arm_poses:
        joint_goals.update(motion.left_arm_poses)
    if motion.right_arm_poses:
        joint_goals.update(motion.right_arm_poses)
    giskard.avoid_all_collisions()
    giskard.achieve_joint_goal(joint_goals)

def map_MoveJointsMotion(motion):
    name_to_position = dict(zip(motion.names, motion.positions))
    giskard.avoid_all_collisions()
    giskard.achieve_joint_goal(
        name_to_position,
        align=motion.align,
        tip_link=motion.tip_link,
        tip_normal=motion.tip_normal,
        root_link=motion.root_link,
        root_normal=motion.root_normal
    )

def map_OpeningMotion(motion):
    giskard.achieve_open_container_goal(
        RobotDescription.current_robot_description.get_arm_chain(motion.arm).get_tool_frame(),
        motion.object_part.name
    )

def map_ClosingMotion(motion):
    giskard.achieve_close_container_goal(
        RobotDescription.current_robot_description.get_arm_chain(motion.arm).get_tool_frame(),
        motion.object_part.name
    )

def map_MoveTCPWaypointsMotion(motion):
    lt = LocalTransformer()
    waypoints = [lt.transform_pose(x, "map") for x in motion.waypoints]
    tip_link = RobotDescription.current_robot_description.get_arm_chain(motion.arm).get_tool_frame()
    root_link = "map"

    giskard.avoid_all_collisions()
    if motion.allow_gripper_collision:
        giskard.allow_gripper_collision(motion.arm)

    giskard.achieve_cartesian_waypoints_goal(
        waypoints=waypoints,
        tip_link=tip_link,
        root_link=root_link,
        enforce_final_orientation=True if motion.movement_type == WaypointsMovementType.ENFORCE_ORIENTATION_FINAL_POINT else False
    )
