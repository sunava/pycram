import json
import os

from pycram.designators.object_designator import *
from pycram.pose import Pose


def color_map(color):
    color_switch = {
        "red": [1, 0, 0, 1],
        "green": [0, 1, 0, 1],
        "blue": [0, 0, 1, 1],
        "black": [0, 0, 0, 1],
        "white": [1, 1, 1, 1],
        # add more colors if needed
    }
    color = color_switch.get(color)
    if color is None:
        return None  # color = [0, 0, 0, 1]
    return color


def get_place_pose(object_type, location):
    poses = {
        ('bowl', 'table'): Pose([4.8, 3.8, 0.8]),
        ('bowl', 'countertop'): Pose([3, 3.8, 1.02], [0, 0, 1, 0]),
        ('cereal', 'table'): Pose([4.8, 3.6, 0.8]),
        ('cereal', 'countertop'): Pose([3, 3.6, 1.02], [0, 0, 1, 0]),
        ('milk', 'table'): Pose([4.8, 4, 0.8]),
        ('milk', 'countertop'): Pose([3, 4, 1.02], [0, 0, 1, 0]),
        ('spoon', 'table'): Pose([4.8, 3.7, 0.8], [0, 0, 1, 0]),
        ('spoon', 'countertop'): Pose([3, 3.7, 1.02]),
        ('cup', 'table'): Pose([4.9, 3.9, 0.72]),
        ('cup', 'countertop'): Pose([2.9, 3.9, 0.95], [0, 0, 1, 0]),
    }
    pose = poses.get((object_type, location))
    return pose


def save_statistics_to_file(statistics, short_str):
    directory = rospy.get_param('/interrupt_demo_node/workdir') + '/robot_logs'
    filename = directory + '/statistics_' + short_str + '.json'
    if not os.path.exists(directory):
        os.makedirs(directory)
    with open(filename, 'w') as file:
        json.dump(statistics, file, indent=4)
    rospy.loginfo(f"Statistics saved to {filename}")


def calculate_statistics(minor_interrupt_count, major_interrupt_count, object_states, ignored_commands, short_str):
    total_commands = minor_interrupt_count + major_interrupt_count
    objects_replaced = sum(1 for state in object_states.values() if state == "new_location")
    objects_not_correct = sum(1 for state in object_states.values() if state not in ["new_location", "old_location"])

    failure_success_rate = (objects_not_correct / total_commands) * 100 if total_commands > 0 else 0
    ignored_commands_rate = (ignored_commands / total_commands) * 100 if total_commands > 0 else 0

    statistics = {
        "total_commands": total_commands,
        "objects_replaced": objects_replaced,
        "objects_not_correct": objects_not_correct,
        "ignored_commands": ignored_commands,
        "failure_success_rate": failure_success_rate,
        "ignored_commands_rate": ignored_commands_rate
    }
    rospy.loginfo(f"Statistics: {statistics}")
    save_statistics_to_file(statistics, short_str)

    return statistics
