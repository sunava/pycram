import numpy as np

from demos.pycram_storing_groceries_demo.utils.misc import *

from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.designators.action_designator import *
from pycram.enums import ObjectType
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
import pycram.external_interfaces.robokudo as robokudo
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, StartSignalWaiter

import random

world = BulletWorld()
v = VizMarkerPublisher()
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

text_to_speech_publisher = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()
start_signal_waiter = StartSignalWaiter()
move = PoseNavigator()

robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])

RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

giskardpy.init_giskard_interface()
# #robokudo.init_robokudo_interface()
# #rospy.sleep(2)
# # giskardpy.spawn_kitchen()
#
# # Shelf Variables
# shelf_pose = Pose([4.417880842356951,4.913135736923778, 0] ,[0, 0, 0, 1])
# lower_compartment = ShelfCompartmentDescription(height=0.09, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
# middle_compartment = ShelfCompartmentDescription(height=0.37, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
# upper_compartment = ShelfCompartmentDescription(height=0.71, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
# shelves = [lower_compartment, middle_compartment, upper_compartment]


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
    'kitchen/shelf:shelf:shelf_floor_0',
    'kitchen/shelf:shelf:shelf_floor_1',
    'kitchen/shelf:shelf:shelf_floor_2',
]


def find_group(obj):
    obj_lower = obj.lower()
    for group, items in groups.items():
        for item in items:
            if obj_lower in item.lower() or item.lower() in obj_lower:
                return group
    return None


# def find_reachable_location_and_nav_pose(enviroment_link, enviroment_desig, robot_desig, arm, world,
#                                          margin_cm=0.2, object_desig=None):
#     # rospy.loginfo("Create a SemanticCostmapLocation instance")
#     location_desig = SemanticCostmapLocation(urdf_link_name=enviroment_link,
#                                              part_of=enviroment_desig,
#                                              for_object=object_desig, margin_cm=margin_cm)
#
#     # rospy.loginfo("Iterate through the locations in the location designator")
#     for location in location_desig:
#         world.current_bullet_world.add_vis_axis(location.pose)
#
#         # Check if the location is clear of objects
#         if not is_location_clear(location.pose, world):
#             continue  # Skip this location if it's not clear
#
#         try:
#             # rospy.loginfo("Create a CostmapLocation instance to check if the location is reachable")
#             reachable_location = CostmapLocation(
#                 target=location.pose,
#                 reachable_for=robot_desig,
#                 reachable_arm=arm
#             )
#             resolved_location = reachable_location.resolve()
#             world.current_bullet_world.add_vis_axis(resolved_location.pose)
#             nav_pose = resolved_location.pose
#             world.current_bullet_world.remove_vis_axis()
#             return location.pose, nav_pose
#         except StopIteration:
#             pass
#     rospy.loginfo("No costmap solution found for the object in the environment")
#     return None, None


def is_location_clear(location_pose, world, clearance_radius=0.20):
    """
    Check if the specified location is clear of objects within the given clearance radius.
    Implement the logic to check for nearby objects in the environment.
    """
    for obj in world.current_bullet_world.objects:
        if obj.type != ObjectType.ENVIRONMENT and obj.type != ObjectType.ROBOT:
            # Calculate the Euclidean distance between the object and the location
            obj_position = obj.pose.position  # Assuming 'pose' attribute with 'position'
            distance = ((obj_position.x - location_pose.position.x) ** 2 +
                        (obj_position.y - location_pose.position.y) ** 2 +
                        (obj_position.z - location_pose.position.z) ** 2) ** 0.5
            if distance < clearance_radius:
                return False  # An object is within the clearance radius
    return True


def demo(step):
    with real_robot:
        # Wait for the start signal
        if step <= 0:
            start_signal_waiter.wait_for_startsignal()

            # continue with the rest of the script
            rospy.loginfo("Start signal received, now proceeding with tasks.")

        # # move to shelf
        # if step <= 1:
        #     move.query_pose_nav(shelf_pose)

        # perceive shelf
        groups_in_shelf = {
            "Kitchen Utensils and Tools": None,
            "Containers and Drinkware": None,
            "Cleaning Supplies": None,
            "Packaged Food and Beverages": None,
            "Fruits": None,
            "Sports Equipment": None,
            "Miscellaneous": None
        }

        if step <= 2:
            shelf_obj = DetectAction(technique='all').resolve().perform()
            first, *remaining = shelf_obj
            for dictionary in remaining:
                for value in dictionary.values():
                    print(value)
                    try:
                        group = find_group(value.type)
                        distance
                        groups_in_shelf[group] = [value.pose, link]
                    except AttributeError:
                        pass
            print(groups_in_shelf)
        if step <= 3:
            table_obj = DetectAction(technique='all').resolve().perform()
            first, *remaining = table_obj
            for dictionary in remaining:
                for value in dictionary.values():
                    try:
                        group = find_group(value.type)
                        if groups_in_shelf[group] is not None:
                            print("searching for free space in shelf depending on group")
                            # navigate to shelf
                            # get free space in shelf
                            # place object
                            # navigate to table

                    except AttributeError:
                        pass

        # res = sort_new(shelf_obj, robot)
        #
        #
        #
        # # object sorted by distance to robot
        # sorted_objects = res[0]
        # print("sorted obj: " + str(sorted_objects))
        #
        # # dict with object poses
        # object_list_poses = res[1]
        # print("pose list: " + str(object_list_poses))
        #
        # # update perceived info to shelves
        # order_items_to_shelves(object_list_poses, shelves)

        # TODO: next steps

        # navigate to table and perceive

        # while objects on table:

        # pick up nearest object

        # navigate to shelf

        # place object depending on shelf areas

        # end?

def get_closet_link_to_pose():
    lf = LocalTransformer()
    position_distance = 30
    nearest_link = None
    for link in links_from_shelf:
      transform = lf.lookupTransform(target_frame="map", source_frame=link, time=rospy.Time(0))
      pose = Pose([0, 0, 0])
      posez = pose.position.z

      dis = abs(transform[0][2] - posez)
      print(dis)
      if dis <= position_distance:
          position_distance = dis
          nearest_link = link
    print(nearest_link)


get_closet_link_to_pose()
#demo(2)
