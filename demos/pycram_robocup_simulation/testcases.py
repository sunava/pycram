# from pycram.designators.motion_designator import TalkingMotion
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA
from typing_extensions import Tuple

from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot, semi_real_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color
import pycram.external_interfaces.giskard as giskardpy

extension = ObjectDescription.get_file_extension()
world = BulletWorld(WorldMode.DIRECT)

v = VizMarkerPublisher()
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "../../resources/after_robocup.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
extension = ObjectDescription.get_file_extension()

robot = Object("hsrb", ObjectType.ROBOT, f"hsrb{extension}", pose=Pose([1, 2, 0]))
color = Color().from_list([0.9, 0.5, 0.9, 1])
robot.set_color(rgba_color=color)

# robot.set_color(0.5, 0.5, 0.9, 1)
robot_desig = ObjectDesignatorDescription(names=["hsrb"])
table_pose = Pose([6.6, 4.9, 0.0], [0.0, 0.0, 0, 1])
long_table_1 = Pose([6.65, 4.6, 0], [0, 0, 0, 1])
long_table_pick = Pose([6.60, 4.6, 0], [0, 0, 0, 1])

long_table_1_rotated = Pose([6.65, 4.6, 0], [0, 0, 1, 0])
shelf_1 = Pose([6.2, 5.6, 0], [0, 0, 1, 0])
shelf_1_rotated1 = Pose([6.2, 5.6, 0], [0, 0, -0.7, 0.7])
shelf_1_rotated = Pose([6.2, 5.6, 0], [0, 0, 0, 1])

milk = Object("milk", ObjectType.MILK, "../../resources/milk.stl", pose=Pose([7.6, 5.2, 0.87]), color=[1, 0, 0, 0.95])
cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "../../resources/breakfast_cereal.stl",
                pose=Pose([2.5, 2.3, 0.95]), color=[0, 1, 0, 1])
spoon = Object("spoon", ObjectType.SPOON, "../../resources/spoon.stl", pose=Pose([2.4, 2.2, 0.85]),
               color=[0, 0, 1, 0.95])
bowl = Object("bowl", ObjectType.BOWL, "../../resources/bowl.stl", pose=Pose([2.5, 2.2, 1.02]), color=[1, 1, 0, 0.95])
human_female = Object("human_female", ObjectType.HUMAN, "../../resources/female_standing.stl", pose=Pose([3, 3.8, 0]),
                      color=[1, 1, 0, 1.05])
world.simulate(seconds=1)
RobotStateUpdater("/tf", "/giskard_joint_states")
KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")
fts_previous_value = None
lt = LocalTransformer()


# world.simulate()

@giskardpy.init_giskard_interface
def move_to_table(pose1):
    with semi_real_robot:
        NavigateAction([pose1]).resolve().perform()


def test_move():
    pose1 = Pose([6.7, 5, 0])
    move_to_table(pose1)


def test_all_perception():
    with semi_real_robot:
        perceived_obj = DetectAction(technique=PerceptionTechniques.ALL).resolve().perform()
        for obj_desig in perceived_obj.values():
            print(obj_desig)


def test_types_perception():
    with semi_real_robot:
        perceived_obj = DetectAction(technique=PerceptionTechniques.TYPES,
                                     object_designator=BelieveObject(types=[ObjectType.MILK])).resolve().perform()
        for obj_desig in perceived_obj.values():
            print(obj_desig)


def test_talk():
    with semi_real_robot:
        TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").perform()


def test_costmap():
    print(kitchen.links)
    sem = SemanticCostmap(kitchen_desig.resolve().world_object, "dinner_table:dinner_table:table_center")
    sem.visualize()


def test_pick_up():
    pose1 = Pose([6.7, 5, 0])
    with semi_real_robot:
        move_to_table(pose1)
    giskardpy.clear()
    milk_desig = ObjectDesignatorDescription(names=["milk"])
    tf_link = kitchen.get_link_tf_frame("dinner_table:dinner_table:table_center")
    main_pick_up_procedure(milk_desig.resolve(), tf_link)
    TalkingMotion("I have picked up the milk").perform()
    # previous_value = fts.get_last_value()


def determine_grasp_set(oTb):
    if oTb.pose.position.x >= 0.20:
        return Grasp.TOP
    return None


def calculate_object_grasp(obj_desig, oTb, grasp_set):
    angle = Pose.quaternion_to_angle(oTb.pose.orientation)
    object_dim = obj_desig.world_object.get_object_dimensions()
    print(f"Object dimensions of {obj_desig.name}: {object_dim}")

    if grasp_set:
        return Grasp.TOP, object_dim, angle

    if object_dim[2] < 0.055:
        rospy.logwarn(f"{obj_desig.name} grasp is set to top, angle: {angle}")
        return Grasp.TOP, object_dim, angle

    if object_dim[2] < 0.065 or (angle > 40 and object_dim[0] > 0.075 and object_dim[1] > 0.075):
        rospy.logwarn(f"{obj_desig.name} grasp is set to top, angle: {angle}")
        return Grasp.FRONT, object_dim, angle

    rospy.logwarn(f"{obj_desig.name} grasp is set to front, angle: {angle}")
    return Grasp.FRONT, object_dim, angle


def adjust_pose_for_grasp(grasp, oTb, object_dim):
    if grasp == Grasp.TOP:
        print("Pose adjusted with z")
        oTb.pose.position.z += (object_dim[2] / 10)
        if object_dim[2] < 0.02:
            rospy.logwarn(f"I am not able to grasp the object: please help me!")
            oTb.pose.position.z = 0.011
    else:
        oTb.pose.position.x += 0.03


def set_grasp_rotation_and_transform(grasp, oTb):
    grasp_rotation = RobotDescription.current_robot_description.grasps[grasp]
    if grasp == Grasp.TOP:
        grasp_q = Quaternion(grasp_rotation[0], grasp_rotation[1], grasp_rotation[2], grasp_rotation[3])
        oTb.multiply_quaternions(grasp_rotation)
    else:
        oTb.orientation = grasp_rotation

    lt = LocalTransformer()
    return lt.transform_pose(oTb, "map")


def execute_motion_sequence(oTmG, config_for_placing, object_name, grasp):
    World.current_world.add_vis_axis(oTmG)

    MoveArmJointsMotion(left_arm_poses=config_for_placing)
    MoveGripperMotion(gripper=Arms.LEFT, motion=GripperState.OPEN, allow_gripper_collision=False).perform()
    TalkingMotion(f"Pick Up now! {object_name.split('_')[0]} from: {grasp}").perform()
    MoveTCPMotion(arm=Arms.LEFT, target=oTmG, allow_gripper_collision=False).perform()


def attach_object_and_finalize(obj_desig):
    giskardpy.achieve_attached(obj_desig)
    tip_link = 'gripper_tool_frame'
    world.robot.attach(obj_desig.world_object, tip_link)

    MoveGripperMotion(gripper=Arms.LEFT, motion=GripperState.CLOSE, allow_gripper_collision=False).perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()


def main_pick_up_procedure(obj_desig, tf_link):
    obj_pose = obj_desig.pose
    oTb = lt.transform_pose(obj_pose, tf_link)

    grasp_set = determine_grasp_set(oTb)
    grasp, object_dim, angle = calculate_object_grasp(obj_desig, oTb, grasp_set)
    adjust_pose_for_grasp(grasp, oTb, object_dim)

    oTmG = set_grasp_rotation_and_transform(grasp, oTb)
    after_pose = oTmG.copy()
    after_pose.pose.position.z += 0.02

    config_for_placing = {
        'arm_flex_joint': -1.1, 'arm_lift_joint': 1.15, 'arm_roll_joint': 0,
        'wrist_flex_joint': -1.6, 'wrist_roll_joint': 0,
    } if grasp == Grasp.TOP else {
        'arm_lift_joint': -1, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
        'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0,
    }

    execute_motion_sequence(oTmG, config_for_placing, obj_desig.name, grasp)
    attach_object_and_finalize(obj_desig)


def find_placeable_pose(environment_link, enviroment_desig, object_desig, margin=0.2):
    location_desig = SemanticCostmapLocation(urdf_link_name=environment_link,
                                             part_of=enviroment_desig,
                                             for_object=object_desig, margin=margin)
    empty_loc = []
    for location in location_desig:

        # Check if the location is clear of objects
        if not is_location_clear(location.pose):
            continue  # Skip this location if it's not clear

        empty_loc.append(location.pose)

    return empty_loc


def is_location_clear(location_pose, clearance_radius=0.25):
    """
    Check if the specified location is clear of objects within the given clearance radius.
    Implement the logic to check for nearby objects in the environment.
    """
    for obj in world.current_world.objects:
        if obj.obj_type != ObjectType.ENVIRONMENT and obj.obj_type != ObjectType.ROBOT:
            # Calculate the Euclidean distance between the object and the location
            obj_position = obj.pose.position  # Assuming 'pose' attribute with 'position'
            distance = ((obj_position.x - location_pose.position.x) ** 2 +
                        (obj_position.y - location_pose.position.y) ** 2 +
                        (obj_position.z - location_pose.position.z) ** 2) ** 0.6
            if distance < clearance_radius:
                return False  # An object is within the clearance radius
    return True  # No objects are within the clearance radius


def main_place_procedure(obj_desig, environment_link):
    # find_pose_in_shelf("biggest_group", obj_desig)
    print(find_placeable_pose(environment_link, kitchen_desig.resolve(), robot_desig.resolve(), obj_desig))


def test_place():
    main_place_procedure(milk, 'shelf_hohc:kitchen_cabinet:shelf_floor_0')


def find_pose_in_context(group: str, obj_desig: object, groups_at_location: Dict[str, Tuple[Pose, str]]):
    """
    Find a pose in the context of the given group and object.
    :param group: the group the objects should be placed to
    :param obj_desig: the object that should be placed
    :param groups_at_location: the groups that where already perceived at the location where to place
    :return:
    """
    link = None
    nearest_pose_to_group = None
    try:
        link = groups_at_location[group][1]
        group_pose = groups_at_location[group][0]
        place_poses = find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve())
        nearest_pose_to_group = get_closest_pose(group_pose, place_poses)

    except (TypeError, KeyError):
        place_poses = []
        for link in links_from_shelf:
            place_poses.append(
                (find_placeable_pose(link, kitchen_desig.resolve(), robot_desig.resolve(), "left", world, 0.1,
                                     object_desig=object), link)
            )

        # in this case it's the biggest group since why not? i assume most poses are then free
        longest_group = max(place_poses, key=lambda x: len(x[0]))
        nearest_pose_to_group = longest_group[0][0]
        link = longest_group[1]
        if group not in groups_in_shelf:
            groups_in_shelf[group] = [nearest_pose_to_group, longest_group[1]]

    if nearest_pose_to_group:
        z_offsets = calculate_z_offsets(links_from_shelf)
        z_height_to_next = get_z_height_to_next_link(link, z_offsets)

        pose_in_shelf = lt.transform_pose(nearest_pose_to_group, kitchen.get_link_tf_frame(link))
        # pose_in_shelf.pose.position.x = -0.005
        if z_height_to_next:
            pose_in_shelf.pose.position.z = (z_height_to_next / 2) - 0.01
        else:
            pose_in_shelf.pose.position.z += 0.03
        adjusted_pose_in_map = lt.transform_pose(pose_in_shelf, "map")
        world.current_bullet_world.add_vis_axis(adjusted_pose_in_map)

        return adjusted_pose_in_map, link
