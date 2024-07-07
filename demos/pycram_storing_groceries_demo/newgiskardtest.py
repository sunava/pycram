import numpy as np
import rospy

from demos.pycram_storing_groceries_demo.utils.misc import *
from pycram.designators.location_designator import find_placeable_pose
from pycram.language import Code

from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.designators.action_designator import *
from pycram.enums import ObjectType
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard_new as giskardpy
# import pycram.external_interfaces.giskard as giskardpy_old

import pycram.external_interfaces.robokudo as robokudo
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, StartSignalWaiter, \
    HSRBMoveGripperReal
from pycram.designator import LocationDesignatorDescription
import random

world = BulletWorld()
v = VizMarkerPublisher()
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

talk = TextToSpeechPublisher()
img_swap = ImageSwitchPublisher()
start_signal_waiter = StartSignalWaiter()
move = PoseNavigator()
gripper = HSRBMoveGripperReal()
robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"])
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

giskardpy.init_giskard_interface()
giskardpy.clear()
# #robokudo.init_robokudo_interface()
# #rospy.sleep(2)
# giskardpy.spawn_kitchen()
#
# # Shelf Variables
# shelf_pose = Pose([4.417880842356951,4.913135736923778, 0] ,[0, 0, 0, 1])
# lower_compartment = ShelfCompartmentDescription(height=0.09, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
# middle_compartment = ShelfCompartmentDescription(height=0.37, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
# upper_compartment = ShelfCompartmentDescription(height=0.71, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
# shelves = [lower_compartment, middle_compartment, upper_compartment]


shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0], [0.0, 0.0, 0, 1])
rotated_shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0],
                          [0.0, 0.0, 0.7220721042045632, 0.6918178057332686])
table_pose = Pose([2.862644998141083, 5.046512935221523, 0.0], [0.0, 0.0, 0.7769090622619312, 0.6296128246591604])

# List of objects
objects = [
    "Fork", "Pitcher", "Bleachcleanserbottle", "Crackerbox", "Minisoccerball",
    "Baseball", "Mustardbottle", "Jellochocolatepuddingbox", "Wineglass", "Orange",
    "Coffeepack", "Softball", "Metalplate", "Pringleschipscan", "Strawberry",
    "Glasscleanerspraybottle", "Tennisball", "Spoon", "Metalmug", "Abrasivesponge",
    "Jellobox", "Dishwashertab", "Knife", "Cerealbox", "Metalbowl", "Sugarbox",
    "Coffeecan", "Milkpackja", "Apple", "Tomatosoupcan", "Tunafishcan",
    "Gelatinebox", "Pear", "Lemon", "Banana", "Pottedmeatcan", "Peach",
    "Plum", "Rubikscube", "Mueslibox", "Cupblue", "Cupgreen", "Largemarker",
    "Masterchefcan", "Scissors", "Scrubcleaner", "Grapes", "Cup_small",
    "screwdriver", "clamp", "hammer", "wooden_block", "Cornybox*"
]

popcorn_frame = "popcorn_table:p_table:table_center"
# Group objects by similarity
groups = {
    "Kitchen Utensils and Tools": ["Fork", "Spoon", "Knife"],
    "Containers and Drinkware": ["Pitcher", "Metalplate", "Metalbowl", "Metalmug", "Wineglass", "Cupblue", "Cupgreen",
                                 "Cup_small"],
    "Cleaning Supplies": ["Bleachcleanserbottle", "Glasscleanerspraybottle", "Abrasivesponge", "Scrubcleaner"],
    "Packaged Food and Beverages": ["Crackerbox", "Mustardbottle", "Jellochocolatepuddingbox", "Coffeepack",
                                    "Pringleschipscan", "Jellobox", "Dishwashertab", "Cerealbox", "Sugarbox",
                                    "Coffeecan", "Milkpackja", "Tomatosoupcan", "Tunafishcan", "Gelatinebox",
                                    "Mueslibox", "Cornybox*", "Masterchefcan", "Pottedmeatcan"],
    "Fruits": ["Orange", "Strawberry", "Apple", "Pear", "Lemon", "Banana", "Peach", "Plum", "Grapes"],
    "Sports Equipment": ["Minisoccerball", "Baseball", "Softball", "Tennisball"],
    "Miscellaneous": ["Rubikscube", "Largemarker", "Scissors", "screwdriver", "clamp", "hammer", "wooden_block"]
}

links_from_shelf = [
    'shelf:shelf:shelf_floor_0',
    'shelf:shelf:shelf_floor_1',
    'shelf:shelf:shelf_floor_2',
]
giskardpy.sync_worlds()


def find_group(obj):
    obj_lower = obj.lower()
    for group, items in groups.items():
        for item in items:
            if obj_lower in item.lower() or item.lower() in obj_lower:
                return group
    return None


def get_closet_link_to_pose(obj_pose):
    position_distance = 30
    nearest_link = None
    for link in links_from_shelf:
        link_pose = kitchen.get_link_pose(link)
        posez = obj_pose.position.z

        dis = abs(link_pose.pose.position.z - posez)
        if dis <= position_distance:
            position_distance = dis
            nearest_link = link
    return nearest_link


def get_closest_pose(obj_pose, pose_list):
    position_distance = float('inf')
    nearest_pose = None

    for pose in pose_list:
        dis = ((obj_pose.pose.position.x - pose.position.x) ** 2 +
               (obj_pose.pose.position.y - pose.position.y) ** 2 +
               (obj_pose.pose.position.z - pose.position.z) ** 2) ** 0.5
        if dis < position_distance:
            position_distance = dis
            nearest_pose = pose

    return nearest_pose


def pickerino(object_desig, grasp, arm, talk, frame):
    lt = LocalTransformer()
    robot = BulletWorld.robot
    # Retrieve object and robot from designators
    object = object_desig.bullet_world_object
    # Calculate the object's pose in the map frame
    oTm = object.get_pose()
    execute = True

    if grasp == "top":
        oTm.pose.position.z += 0.035

    grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
    oTb = lt.transform_pose(oTm, kitchen.get_link_tf_frame(frame))
    oTb.orientation = grasp_rotation
    oTmG = lt.transform_pose(oTb, "map")

    rospy.logwarn("Opening Gripper")
    gripper.pub_now("open")

    rospy.logwarn("Picking up now")

    tool_frame = robot_description.get_tool_frame(arm)
    special_knowledge_offset = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))
    y = 0.04
    special_knowledge_offset.pose.position.y -= y
    if grasp == "top":
        z = 0.025
        special_knowledge_offset.pose.position.z -= z

    push_baseTm = lt.transform_pose(special_knowledge_offset, "map")
    special_knowledge_offsetTm = lt.transform_pose(special_knowledge_offset, "map")
    liftingTm = push_baseTm
    liftingTm.pose.position.z += 0.03
    talk.pub_now("Pick Up now!" + object.type)
    giskard_return = giskardpy.achieve_sequence_pick_up(oTmG, special_knowledge_offsetTm)
    giskardpy.achieve_attached(object_desig)
    tip_link = 'hand_gripper_tool_frame'
    BulletWorld.robot.attach(object=object_desig.bullet_world_object, link=tip_link)
    gripper.pub_now("close")


def pakerino(torso_z=0.15, config=None):
    if not config:
        config = {'arm_lift_joint': torso_z, 'arm_flex_joint': 0, 'arm_roll_joint': -1.2, 'wrist_flex_joint': -1.5,
                  'wrist_roll_joint': 0}
    giskardpy.avoid_all_collisions()
    giskardpy.achieve_joint_goal(config)
    print("Parking done")


def monitor_func_place():
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        return SensorMonitoringCondition
    return False


def placerino(object_desig, grasp, arm, talk, target_location, frame):
    lt = LocalTransformer()
    robot = BulletWorld.robot
    oTm = target_location
    oTm.pose.position.z += 0.015
    # if grasp == "top":
    #     oTm.pose.position.z += 0.035

    grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
    oTb = lt.transform_pose(oTm, kitchen.get_link_tf_frame(frame))
    oTb.orientation = grasp_rotation
    special_knowledge_offset = oTb
    # x = 0.04
    # special_knowledge_offset.pose.position.x -= x
    # if grasp == "top":
    #     y = 0.025
    #     special_knowledge_offset.pose.position.y -= y
    oTmG = lt.transform_pose(oTb, "map")
    # push_baseTm = lt.transform_pose(special_knowledge_offset, "map")

    # tool_frame = robot_description.get_tool_frame(arm)
    # special_knowledge_offset = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))

    # push_baseTm = lt.transform_pose(special_knowledge_offset, "map")
    # special_knowledge_offsetTm = lt.transform_pose(special_knowledge_offset, "map")
    # push_baseTm.pose.position.z -= 1
    # world.current_bullet_world.add_vis_axis(oTmG)
    # world.current_bullet_world.add_vis_axis(push_baseTm)
    talk.pub_now("Placing now!")
    # giskardpy.achieve_placing_without_prepose(oTmG, object_desig, kitchen)
    giskardpy.achieve_sequence_te(oTmG, object_desig)
    gripper.pub_now("open")
    # try:
    #     plan = Code(lambda: giskardpy.achieve_sequence_te(push_baseTm)) >> Monitor(monitor_func_place)
    #     plan.perform()
    # except SensorMonitoringCondition:
    #     rospy.logwarn("interrupted")
    # talk.pub_now("opening my gripper")
    # MoveGripperMotion(motion="open", gripper="left").resolve().perform()
    # robot.detach_all()


def multiply_quaternions(q1, q2):
    """
    Multiply two quaternions.

    Parameters:
    q1 (tuple): First quaternion (x1, y1, z1, w1).
    q2 (tuple): Second quaternion (x2, y2, z2, w2).

    Returns:
    tuple: The product of the two quaternions.
    """

    x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
    x2, y2, z2, w2 = q2.x, q2.y, q2.z, q2.w

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


def sort_object_data_by_angle_and_height_with_threshold(object_data, angle_threshold):
    # Sort the object_data by height in descending order
    sorted_by_height = sorted(object_data.items(), key=lambda item: -item[1]['height'])

    # Adjust the order based on the angle difference with the threshold
    adjusted_list = []
    while sorted_by_height:
        current_item = sorted_by_height.pop(0)
        adjusted_list.append(current_item)

        for i, next_item in enumerate(sorted_by_height):
            if abs(current_item[1]['angle'] - next_item[1]['angle']) > angle_threshold:
                continue
            else:
                # Move the item with a smaller angle difference to the front
                sorted_by_height.pop(i)
                adjusted_list.append(next_item)
                current_item = next_item
                break

    return dict(adjusted_list)


# Now `sorted_object_data` is sorted by the lowest angle and highest height


def demo(step):
    with real_robot:
        # Wait for the start signal
        gripper.pub_now("close")
        config = {'arm_lift_joint': 0.64, 'arm_flex_joint': 0, 'arm_roll_joint': -1.2, 'wrist_flex_joint': -1.5,
                  'wrist_roll_joint': 0}
        # pakerino()

        locationtoplace = Pose([2.6868790796016738, 5.717920690528091, 0.715])

        # #giskardpy.spawn_kitchen()
        groups_in_shelf = {
            "Kitchen Utensils and Tools": None,
            "Containers and Drinkware": None,
            "Cleaning Supplies": None,
            "Packaged Food and Beverages": None,
            "Fruits": None,
            "Sports Equipment": None,
            "Miscellaneous": None
        }
        groups_on_table = {}

        if step <= 3:
            rospy.logerr("driving")
            talk.pub_now("driving", True)
            move.pub_now(table_pose)
            pose1 = Pose([1.35, 4.4, 0], [0, 0, 1, 0])
            move.pub_now(pose1)
        #     look_pose = kitchen.get_link_pose("popcorn_table:p_table:table_center")
        #     # look_pose.pose.position.x += 0.5
        #     giskardpy.move_head_to_pose(locationtoplace)
        #     # giskardpy.move_head_to_pose(look_pose)
        #     # LookAtAction(targets=[look_pose]).resolve().perform()  # 0.18
        #     # LookAtAction(targets=[look_pose]).resolve().perform()
        #     table_obj = DetectAction(technique='all').resolve().perform()
        #     first, *remaining = table_obj
        #     for dictionary in remaining:
        #         for value in dictionary.values():
        #             try:
        #                 group = find_group(value.type)
        #                 groups_on_table[value.name] = [value, group]
        #
        #             except AttributeError:
        #                 pass
        # obj_to_place = None
        #
        #
        # if step <= 4:
        #     giskardpy.sync_worlds()
        #     # print(groups_on_table.keys())
        #     # Initialize an empty dictionary to store the data
        #     object_data = {}
        #
        #     for obj in groups_on_table.values():
        #         lt = LocalTransformer()
        #         robot = BulletWorld.robot
        #
        #         # Retrieve object and robot from designators
        #         object = obj[0].bullet_world_object
        #
        #         # Calculate the object's pose in the map frame
        #         oTm = object.get_pose()
        #         execute = True
        #
        #         oTb = lt.transform_pose(oTm, kitchen.get_link_tf_frame(popcorn_frame))
        #         angle = helper.quaternion_to_angle((oTb.pose.orientation.x, oTb.pose.orientation.y,
        #                                             oTb.pose.orientation.z, oTb.pose.orientation.w))
        #         object_dim = object.get_object_dimensions()
        #         print("obj deim von " + str(object.name) + str(object.get_object_dimensions()))
        #
        #         #height
        #         if object_dim[2] < 0.11:
        #             rospy.logwarn(object.name + "grasp is set to top")
        #             grasp = "top"
        #             #<oTb.pose.position.z += (object_dim[2]/10)
        #         else:
        #             rospy.logwarn(object.name + "grasp is set to front")
        #             grasp = "front"
        #
        #         if object_dim[2]< 0.02:
        #             oTb.pose.position.z = 0.011
        #         grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
        #         if object_dim[0] > 0.099 or grasp == "top":
        #             rospy.logwarn(object.name + "width is bigger then 0.099 or graso is set to top")
        #             grasp_q = Quaternion(grasp_rotation[0], grasp_rotation[1], grasp_rotation[2], grasp_rotation[3])
        #             oTb.orientation = multiply_quaternions(oTb.pose.orientation, grasp_q)
        #
        #         else:
        #             rospy.logwarn(object.name + "object is small enough")
        #             print(grasp_rotation)
        #             oTb.orientation = grasp_rotation
        #
        #         oTmG = lt.transform_pose(oTb, "map")
        #         print(obj[0].name)
        #
        #         offsetTbase = lt.transform_pose(oTb, obj[0].name)
        #
        #         if grasp == "top":
        #             print("pose adjusted with z")
        #             offsetTbase.pose.position.z += 0.0
        #         else:
        #             print("pose adjusted with y")
        #             offsetTbase.pose.position.x -= 0.0
        #
        #
        #         pre_pose = lt.transform_pose(offsetTbase, "map")
        #         BulletWorld.current_bullet_world.add_vis_axis(oTmG)
        #         # Store the data in the dictionary
        #         object_data[object.name] = {
        #             "pick_pose": oTmG,
        #             "pre_pose": pre_pose,
        #             "angle": angle,
        #             "height": object_dim[2],
        #             "width": object_dim[0],
        #             "grasp": grasp,
        #             "obj": obj[0]
        #         }
        #
        #     # Example usage:
        #     angle_threshold = 15  # Define your angle threshold here
        #     sorted_and_adjusted_object_data = sort_object_data_by_angle_and_height_with_threshold(object_data,
        #                                                                                           angle_threshold)
        #
        #     first_key = next(iter(sorted_and_adjusted_object_data))
        #
        #
        #
        #     for obj in sorted_and_adjusted_object_data:
        #         object_info = object_data[obj]
        #
        #         # Access individual attributes
        #         pick_pose = object_info['pick_pose']
        #         pre_pose = object_info['pre_pose']
        #         angle = object_info['angle']
        #         height = object_info['height']
        #         width = object_info['width']
        #         grasp = object_info['grasp']
        #         obj = object_info['obj']
        #
        #
        #         gripper.pub_now("open")
        #         talk.pub_now("Pick Up now!" + obj.bullet_world_object.name.split('_')[0])
        #         #giskard_return = giskardpy.achieve_sequence_pick_up(pre_pose, pick_pose)
        #
        #         giskard_success = True
        #         max_attempts = 3  # Limit the number of attempts to avoid infinite loops
        #         attempts = 0
        #
        #         while giskard_success and attempts < max_attempts:
        #             giskard_return = giskardpy.achieve_sequence_pick_up(pre_pose, pick_pose)
        #             attempts += 1
        #             if giskard_return.error and giskard_return.error.msg:
        #                 giskard_success = False
        #             else:
        #                 # If the action was successful, break the loop
        #                 # Alternatively, update the logic to retry if certain conditions are met
        #                 break
        #         #giskard_return = giskardpy.achieve_placing_without_prepose(oTmG, obj, kitchen)
        #         # oTg = lt.transform_pose(oTmG, robot.get_link_tf_frame("hand_gripper_tool_frame"))
        #         # oTg.pose.position.z += 0.03
        #         # otm_special = lt.transform_pose(oTmG, "map")
        #         # giskardpy.achieve_sequence_pick_up(otm_special)
        #         print(giskard_return.error.msg)
        #         giskardpy.achieve_attached(obj)
        #         tip_link = 'hand_gripper_tool_frame'
        #         BulletWorld.robot.attach(object=obj.bullet_world_object, link=tip_link)
        #         gripper.pub_now("close")
        #         giskardpy.avoid_all_collisions()
        #         pakerino()
        #         gripper.pub_now("open")
        #
        #
        #

        # locationtoplace = Pose([2.7868790796016738, 5.617920690528091, 0.815])
        # placerino(obj_to_place, "front", "left", talk, locationtoplace, popcorn_frame)

        #
        #
        # obj_to_place = obj[0]
        # # PickUpAction(obj[0], ["left"], ["front"]).resolve().perform()

        # pakerino()
        # gripper.pub_now("close")


demo(0)
