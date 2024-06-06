from collections import Counter

from demos.pycram_storing_groceries_demo.utils.misc import *
from docutils.nodes import math
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.designators.object_designator import *

# poses declared before the challenge
pose_test_area = Pose([2.25, 2.4, 0], [0, 0, 0, 1])
pose_cabinet = Pose([2.25, 2.4, 0], [0, 0, 0, 1])
pose_table = Pose([1.5, 2.0, 0], [0, 0, 1, 0])

categories = {
    "Sports Equipment": ["Minisoccerball", "Baseball", "Softball", "Tennisball"],
    "Cutlery": ["Fork", "Spoon", "Knife", "Metalplate", "Metalbowl"],
    "Drinkware": ["Pitcher", "Wineglass", "Metalmug", "Cupblue", "Cupgreen", "Cup_small"],
    "Cleaning Supplies": ["Bleachcleanserbottle", "Glasscleanerspraybottle", "Abrasivesponge", "Scrubcleaner", "Dishwashertab"],
    "Tools": ["Scissors", "Screwdriver", "Clamp", "Hammer", "Wooden_block"],
    "Toys/Games": ["Rubikscube"],
    "Other": ["Largemarker"],
    "Snacks": ["Crackerbox", "Pringleschipscan"],
    "Breakfast Foods": ["Cerealbox", "Mueslibox", "Cornybox"],
    "Canned Goods": ["Tomatosoupcan", "Tunafishcan", "Pottedmeatcan", "Masterchefcan"],
    "Beverage Ingredients": ["Coffeecan", "Coffeepack", "Sugarbox", "Milkpackja"],
    "Condiments": ["Mustardbottle"],
    "Fruits": ["Orange", "Strawberry", "Apple", "Pear", "Lemon", "Banana", "Peach", "Plum", "Grapes"]
}

item_to_category = {item: category for category, items in categories.items() for item in items}


def navigate_and_detect(location: str):
    """
    Navigates to given location and perceives.

    :return: tupel of State and dictionary of found objects in the FOV
    """
    TalkingMotion("Navigating").resolve().perform()
    if location == "cabinet":
        NavigateAction([pose_cabinet]).resolve().perform()
    elif location == "table":
        TalkingMotion("done").resolve().perform()
        #NavigateAction([pose_table]).resolve().perform()

    TalkingMotion("Perceiving").resolve().perform()

    # von Celina und mohammed
    try:
        object_desig = DetectAction(technique='all').resolve().perform()
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        object_desig = {}
    return object_desig


def sort_new(perceived: dict, robot: BulletWorldObject):
    obj_list = {}
    sorted_obj_list = []

    obj_dict = perceived[1]
    robot_pose = robot.get_pose()
    for value in obj_dict:
        # calculate distance
        distance = math.sqrt(pow((obj_dict[value].pose.position.x - robot_pose.pose.position.x), 2) +
                             pow((obj_dict[value].pose.position.y - robot_pose.pose.position.y), 2) +
                             pow((obj_dict[value].pose.position.z - robot_pose.pose.position.z), 2))

        # get position of object
        obj_pose = Pose()
        obj_pose.position = [obj_dict[value].pose.position.x, obj_dict[value].pose.position.y,
                             obj_dict[value].pose.position.z]
        obj_pose.orientation = [obj_dict[value].pose.orientation.x, obj_dict[value].pose.orientation.y,
                                obj_dict[value].pose.orientation.z, obj_dict[value].pose.orientation.w]


        # put everything in a dict
        obj_list[value] = obj_pose
        sorted_obj_list.append((obj_dict[value].type, distance))

    sorted_obj_list = sorted(sorted_obj_list, key=lambda distance: distance[1])

    return [sorted_obj_list, obj_list]


def sort_and_remove_duplicates(unsorted_list):
    #letters to lowercase
    unsorted_list = [s.lower() for s in unsorted_list]

    #count frequency of each string
    frequency = Counter(unsorted_list)

    #remove duplicates (transform to set and back to list)
    unique_strings = list(set(unsorted_list))

    #sort list according to frequency
    sorted_unique_strings = sorted(unique_strings, key=lambda x: (-frequency[x], x))

    return sorted_unique_strings


def order_categorys(objects:List):
    categorys = []
    for obj in objects:
        try:
            categorys.append(item_to_category[obj])
        except KeyError:
            categorys.append("unknown")

    ordered_categorys = sort_and_remove_duplicates(categorys)
    return ordered_categorys

def get_category(obj: str):
    try:
        return item_to_category[obj]
    except KeyError:
        return "unknown"



