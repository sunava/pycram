import json
import os
import shutil

import pybullet as p
import pybullet_data
import numpy as np
from PIL import Image

from pycram.designators.object_designator import *
from pycram.pose import Pose

import json
import os
from collections import OrderedDict
from datetime import datetime

now = datetime.now()


def read_json_file(file_path):
    with open(file_path, 'r') as file:
        return json.load(file)


def combine_statistics_from_directory(directory_path):
    combined_statistics = {}

    for filename in os.listdir(directory_path):
        if filename.endswith(".json"):
            file_path = os.path.join(directory_path, filename)
            statistics = read_json_file(file_path)
            for key, value in statistics.items():
                if key in combined_statistics:
                    combined_statistics[key] += value
                else:
                    combined_statistics[key] = value

    # Calculate additional statistics
    total_commands = combined_statistics.get('total_commands', 0)
    objects_replaced = combined_statistics.get('objects_replaced', 0)
    objects_not_correct = combined_statistics.get('objects_not_correct', 0)
    ignored_commands = combined_statistics.get('ignored_commands', 0)
    changed_locations = combined_statistics.get('changed_locations', 0)

    # Calculate ignored commands rate
    if total_commands > 0:
        combined_statistics['ignored_commands_rate'] = (ignored_commands / total_commands) * 100
    else:
        combined_statistics['ignored_commands_rate'] = 0.0

    # Calculate failed/lost commands
    # failed_lost_commands = total_commands - (
    #         objects_replaced + objects_not_correct + ignored_commands + changed_locations)
    # combined_statistics['failed/lost_commands'] = failed_lost_commands

    return combined_statistics


def write_json_file(data, file_path):
    # Specify the desired order of keys
    ordered_data = OrderedDict()
    ordered_data['total_commands'] = data.get('total_commands', 0)
    ordered_data['objects_replaced'] = data.get('objects_replaced', 0)
    ordered_data['changed_locations'] = data.get('changed_locations', 0)
    ordered_data['objects_not_correct'] = data.get('objects_not_correct', 0)
    ordered_data['ignored_commands'] = data.get('ignored_commands', 0)
    # ordered_data['lost_commands'] = data.get('failed/lost_commands', 0)
    ordered_data['failure_success_rate'] = data.get('failure_success_rate', 0.0)
    ordered_data['ignored_commands_rate'] = data.get('ignored_commands_rate', 0.0)

    with open(file_path, 'w') as file:
        json.dump(ordered_data, file, indent=4)


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


once = False


def save_statistics_to_file(statistics, short_str):
    global once
    directory = rospy.get_param('/interrupt_demo_node/workdir') + '/robot_logs/tmp'

    filename = directory + '/statistics_' + short_str + '.json'

    if not os.path.exists(directory):
        os.makedirs(directory)

    if not once:
        short_str = now.strftime("-%d-%m-%y-%H:%M")
        output_file = directory + '/../robot_combined_statistics' + short_str + '.json'
        combined_statistics = combine_statistics_from_directory(directory)
        write_json_file(combined_statistics, output_file)
        print(f"Combined statistics have been written to {output_file}")
        print(f"Cleard robots log tmp folder")
        # Clear the directory before saving new statistics
        clear_directory(directory)
        once = True

    with open(filename, 'w') as file:
        json.dump(statistics, file, indent=4)
    # rospy.loginfo(f"Statistics saved to {filename}")


def clear_directory(directory):
    """
    Clear all files in the given directory.
    """
    if os.path.exists(directory):
        for filename in os.listdir(directory):
            file_path = os.path.join(directory, filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
            except Exception as e:
                rospy.logerror(f'Failed to delete {file_path}. Reason: {e}')


def calculate_statistics(minor_interrupt_count, major_interrupt_count, object_states, ignored_commands, short_str,
                         changed_locations):
    total_commands = minor_interrupt_count + major_interrupt_count
    objects_replaced = sum(1 for state in object_states.values() if state == "new_location")
    objects_not_correct = sum(1 for state in object_states.values() if state not in ["new_location", "old_location"])

    failure_success_rate = (objects_not_correct / total_commands) * 100 if total_commands > 0 else 0
    ignored_commands_rate = (ignored_commands / total_commands) * 100 if total_commands > 0 else 0

    statistics = {
        "total_commands": total_commands,
        "objects_replaced": objects_replaced,
        "changed_locations": changed_locations,
        "ignored_commands": ignored_commands,
        "failure_success_rate": failure_success_rate,
        "ignored_commands_rate": ignored_commands_rate,
        "objects_not_correct": objects_not_correct
    }
    # rospy.loginfo(f"Statistics: {statistics}")
    save_statistics_to_file(statistics, short_str)

    return statistics


def update_object_state(obj_name, state, current_location, globaldict):
    if obj_name in globaldict["object_states"]:
        if state == "new_location" and obj_name in globaldict["original_locations"]:
            if globaldict["original_locations"][obj_name] != current_location:
                globaldict["object_states"][obj_name] = state
        elif state == "old_location" and obj_name in globaldict["original_locations"]:
            if globaldict["original_locations"][obj_name] == current_location:
                globaldict["object_states"][obj_name] = state


def current_snapshot(obj_type):
    view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[4, 3, 1],
                                                      distance=3,
                                                      yaw=45,
                                                      pitch=-22.5,
                                                      roll=0,
                                                      upAxisIndex=2)
    aspect_ratio = 16.0 / 9.0
    projection_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=aspect_ratio,
                                                     nearVal=0.1,
                                                     farVal=100.0)

    width, height, rgbImg, depthImg, segImg = p.getCameraImage(width=1920,
                                                               height=1080,
                                                               viewMatrix=view_matrix,
                                                               projectionMatrix=projection_matrix)
    rgb_array = np.array(rgbImg)
    rgb_array = rgb_array[:, :, :3]

    image = Image.fromarray(rgb_array)
    directory = rospy.get_param('/interrupt_demo_node/workdir') + '/robot_logs/tmp'

    if not os.path.exists(directory):
        os.makedirs(directory)
    now_time = datetime.now()

    short_str = now_time.strftime("-%d-%m-%y-%H:%M:%S")
    output_file = directory + f'/../snapshot_' + short_str + f'_{obj_type}.png'
    image.save(output_file)


def aggregate_robot_statistics(directory=rospy.get_param('/interrupt_demo_node/workdir') + '/robot_logs'):
    summary = {
        "total_commands": 0,
        "objects_replaced": 0,
        "changed_locations": 0,
        "objects_not_correct": 0,
        "ignored_commands": 0,
        # "lost_commands": 0,
        "failure_success_rate": 0.0,
        "ignored_commands_rate": 0.0
    }

    files = os.listdir(directory)

    json_files = [f for f in files if f.startswith("robot_combined_statistics") and f.endswith(".json")]

    for json_file in json_files:
        with open(os.path.join(directory, json_file), 'r') as f:
            data = json.load(f)
            summary["total_commands"] += data.get("total_commands", 0)
            summary["objects_replaced"] += data.get("objects_replaced", 0)
            summary["changed_locations"] += data.get("changed_locations", 0)
            summary["objects_not_correct"] += data.get("objects_not_correct", 0)
            summary["ignored_commands"] += data.get("ignored_commands", 0)
            # summary["lost_commands"] += data.get("lost_commands", 0)
            summary["failure_success_rate"] += data.get("failure_success_rate", 0.0)
            summary["ignored_commands_rate"] += data.get("ignored_commands_rate", 0.0)

    num_files = len(json_files)
    if num_files > 0:
        summary["failure_success_rate"] /= num_files
        summary["ignored_commands_rate"] /= num_files

    summary_file = os.path.join(directory, "statistics_summary.json")
    with open(summary_file, 'w') as f:
        json.dump(summary, f, indent=4)

    print(f"Summary written to {summary_file}")
