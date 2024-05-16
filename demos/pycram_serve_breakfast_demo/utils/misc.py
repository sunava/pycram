import rospy
from docutils.nodes import math

from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.designators.object_designator import *
from std_msgs.msg import String
from deprecated import deprecated


def sort_objects(obj_dict: dict, wished_sorted_obj_list: list):
    """
    Transforms the given object dictionary to a distance sorted list.
    The Metalplate, if seen, is arranged as the first object in the list.

    :param obj_dict: tupel of State and dictionary of founded objects in the FOV
    :param wished_sorted_obj_list: list of object types we like to keep
    :return: distance sorted list of seen and wished to keep objects
    """
    sorted_objects = []

    if len(obj_dict) == 0:
        return sorted_objects

    # cut of the given State and keep the dictionary
    first, *remaining = obj_dict
    # calculate euclidian distance for all found object in a list of tupels
    for dictionary in remaining:
        sorted_objects = sorted(filter(lambda x: x in wished_sorted_obj_list, dictionary.values()),
                                key=lambda x: wished_sorted_obj_list.index(x))

    # print which objects are in the final list
    test_list = []
    for test_object in sorted_objects:
        test_list.append(test_object.type)
    print(test_list)

    return sorted_objects


def get_bowl_list(obj_dict: dict):
    objects_list = []

    if len(obj_dict) == 0:
        return objects_list

    first, *remaining = obj_dict
    for dictionary in remaining:
        for value in dictionary.values():
            if value.type == "Metalbowl":
                objects_list.append(value)
    return objects_list


def get_bowl(obj_dict: dict):
    if len(obj_dict) == 0:
        return None

    first, *remaining = obj_dict
    for dictionary in remaining:
        for value in dictionary.values():
            if value.type == "Metalbowl":
                return value
    return None


def get_free_spaces(obj_dict: dict):
    free_places_tuples = []
    sorted_places = []

    if len(obj_dict) == 0:
        return sorted_places

    first, *remaining = obj_dict
    for dictionary in remaining:
        for value in dictionary.values():
            locations_list = value.attribute
            for location in locations_list:
                seperated_location = location.split(",")
                occupied = eval(seperated_location[1])
                if occupied:
                    x = float(seperated_location[2])
                    y = float(seperated_location[3])
                    z = float(seperated_location[4])
                    free_places_tuples.append((Pose([x, y, z]), y))

    sorted_list = sorted(free_places_tuples, key=lambda y_pose: y_pose[1])
    sorted_places = [tup[0] for tup in sorted_list]

    # print which objects are in the final list
    test_list = []
    for test_object in sorted_places:
        test_list.append(test_object.pose)
    print(test_list)

    return sorted_places
