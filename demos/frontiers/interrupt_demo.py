#!/usr/bin/env python3
import logging
from datetime import datetime

import rospy
from pycram.bullet_world import BulletWorld, Object
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.external_interfaces.interrupt_actionclient import InterruptClient
from pycram.failure_handling import RetryMonitor
from pycram.language import Monitor, Code
from pycram.plan_failures import MajorInterrupt, PlanFailure, ChangeLocationException
from pycram.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from utils.utils import *

sleep = True
ignored_commands = 0
changed_locations = 0
minor_interrupt_count = 0
major_interrupt_count = 0
try:
    from speech_processing.msg import dict_object, message_objects_in_use
except ModuleNotFoundError as e:
    rospy.logwarn("Failed to import speech_processing messages, frontiers can not be used")
now = datetime.now()
short_str = now.strftime("%Y-%m-%d_%H:%M:%S")
world = BulletWorld("DIRECT")
viz = VizMarkerPublisher()
robot = Object("pr2", "robot", "pr2.urdf", pose=Pose([1, 2, 0]))
apartment = Object("apartment", "environment", "apartment-small.urdf")

milk1 = Object("milk1", "milk", "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[0, 0, 1, 1])
milk2 = Object("milk2", "milk", "milk.stl", pose=Pose([2.5, 1.7, 1.02]), color=[1, 0, 0, 1], size="big")
bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([2.5, 2.2, 1.02]), color=[1, 1, 1, 1])
spoon = Object("spoon", "spoon", "spoon.stl", pose=Pose([2.4, 2.2, 0.85]), color=[0, 0, 1, 1])
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([2.5, 2.4, 1.05]), color=[0, 1, 0, 1])
cup = Object("cup", "cup", "jeroen_cup.stl", pose=Pose([0.5, 1.6, 1.38], [0, 0, 1, 0]))
human = Object("humanf", "human", "female_sitting.stl", pose=Pose([4.9, 5.0, 0], [0, 0, 0, 1]))
human = Object("humans", "human", "female_standing.stl", pose=Pose([3.0, 5.0, 0], [0, 0, 0, 1]))
object_states = {"milk1": "old_location", "milk2": "old_location", "bowl": "old_location", "spoon": "old_location",
                 "cereal": "old_location", "cup": "old_location"}
objecterinos_typos = ["milk", "bowl", "spoon", "cereal", "cup"]
# Assuming original_locations and current_locations dictionaries are defined and updated accordingly
original_locations = {"milk1": [2.5, 2, 1.02], "milk2": [2.5, 1.7, 1.02], "bowl": [2.5, 2.2, 1.02],
                      "spoon": [2.4, 2.2, 0.85], "cereal": [2.5, 2.4, 1.05], "cup": [0.5, 1.6, 1.38]}

apartment.attach(spoon, 'cabinet10_drawer_top')

pick_pose = Pose([2.7, 2.15, 1])

robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])

fluent = InterruptClient()
obj_type = ""
obj_color = ""
obj_name = ""
obj_location = ""
obj_size = ""
age = 0
place_pose = None
nav_pose = None
original_pose = None
obj_desig = None
handle_desig = None
current_cmd = None
current_location = None
destination_location = None
drawer_open_loc = None
used_arm = "left"
grasp = "front"


def update_object_state(obj_name, state, current_location):
    global object_states
    if obj_name in object_states:
        if state == "new_location" and obj_name in original_locations:
            if original_locations[obj_name] != current_location:
                object_states[obj_name] = state
        elif state == "old_location" and obj_name in original_locations:
            if original_locations[obj_name] == current_location:
                object_states[obj_name] = state


def age_map(age):
    global nav_pose, place_pose
    if age == 1:
        nav_pose = Pose([4, 3.8, 0])
        place_pose = Pose([4.8, 3.8, 0.8])
    else:
        nav_pose = Pose([3.9, 3.8, 0], [0, 0, 1, 0])
        place_pose = Pose([3, 3.8, 1.02], [0, 0, 1, 0])


def access_obj():
    global drawer_open_loc, handle_desig, obj_desig
    if obj_desig.bullet_world_object.type.lower() == "spoon":
        handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig.resolve())
        pose = Pose([1.75, 1.79, 0], [0, 0, 0.533512180079847, 0.8457923821520558])
        drawer_open_loc = AccessingLocation.Location(pose, ["left"])
        NavigateAction([drawer_open_loc.pose]).resolve().perform()
        OpenAction(object_designator_description=handle_desig,
                   arms=[drawer_open_loc.arms[0]]).resolve().perform()  # spoon.detach(apartment)
    elif obj_desig.bullet_world_object.type.lower() == "cup":
        handle_desig = ObjectPart(names=['handle_cab1_top_door'], part_of=apartment_desig.resolve())
        pose = Pose([1.2, 1.5, 0], [0, 0, 1, 0])
        drawer_open_loc = AccessingLocation.Location(pose, ["left"])
        NavigateAction([drawer_open_loc.pose]).resolve().perform()
        OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()


def get_recovery_pose() -> Pose:
    global obj_desig, obj_type
    pose = None
    if obj_desig.bullet_world_object.type.lower() in ["spoon", "cup"]:
        access_obj()
        pose = drawer_open_loc.pose
    poses = {'bowl': Pose([1.7, 1.9, 0]), 'cereal': Pose([1.7, 2, 0]), 'milk': Pose([1.7, 1.9, 0]),
             'cup': Pose([1.7, 1.9, 0]), 'spoon': pose}
    pose = poses.get(obj_type)
    return pose


def get_nav_pose(object_type, location):
    global current_location
    poses = {('bowl', 'table'): Pose([4, 3.8, 0]), ('bowl', 'countertop'): Pose([3.9, 3.8, 0], [0, 0, 1, 0]),
             ('cereal', 'table'): Pose([4, 3.6, 0]), ('cereal', 'countertop'): Pose([3.9, 3.6, 0], [0, 0, 1, 0]),
             ('milk', 'table'): Pose([4, 4, 0]), ('milk', 'countertop'): Pose([3.9, 4, 0], [0, 0, 1, 0]),
             ('spoon', 'table'): Pose([4.2, 3.7, 0]), ('spoon', 'countertop'): Pose([3.7, 3.7, 0], [0, 0, 1, 0]),
             ('cup', 'table'): Pose([4.3, 3.9, 0]), ('cup', 'countertop'): Pose([3.8, 3.9, 0], [0, 0, 1, 0]), }

    pose = poses.get((object_type, location))
    current_location = location
    return pose


mylistofobj = [dict_object(type="bowl", color="white", name="bowl", location="countertop", size="normal"),
               dict_object(type="milk", color="blue", name="milk1", location="countertop", size="normal"),
               dict_object(type="milk", color="red", name="milk2", location="countertop", size="big"),
               dict_object(type="cereal", color="green", name="cereal", location="countertop", size="normal"),
               dict_object(type="cup", color="white", name="cup", location="", size="Normal"),
               dict_object(type="spoon", color="white", name="spoon", location="kitchen_drawer", size="normal")]


def update_current_command():
    global current_cmd, obj_type, obj_color, obj_name, obj_location, obj_size, unhandled_objects, age, destination_location
    global minor_interrupt_count, major_interrupt_count, ignored_commands, changed_locations
    global mylistofobj

    current_cmd = fluent.next_command()

    if current_cmd:
        minor_cmd = current_cmd.get("minor", {}).get("command")
        major_cmd = current_cmd.get("major", {}).get("command")
        add_obj = None
        del_obj = None

        if minor_cmd == "setting_breakfast":
            minor_interrupt_count += 1
            objects_to_add = [
                dict_object(type="bowl", color="white", name="bowl", location="countertop", size="normal"),
                #dict_object(type="milk", color="blue", name="milk1", location="countertop", size="normal"),
                dict_object(type="milk", color="red", name="milk2", location="countertop", size="big"),
                dict_object(type="cereal", color="green", name="cereal", location="countertop", size="normal"),
                #dict_object(type="cup", color="white", name="cup", location="", size="Normal"),
                dict_object(type="spoon", color="blue", name="spoon", location="", size="normal")
            ]

            fluent.modify_objects_in_use(objects_to_add, [])

        elif minor_cmd == "replace_object":
            print("got replace")
            minor_interrupt_count += 1
            old_attributes = (obj_type.lower(), obj_color, obj_name.lower(), obj_location, obj_size.lower())
            del_cmd = current_cmd.get("minor", {}).get("del_object")

            if del_cmd:
                del_obj = del_cmd[0]
                del_attributes = (
                    del_obj.type.lower(), del_obj.color, del_obj.name.lower(), del_obj.location, del_obj.size.lower()
                )
                if all(attr == '' for attr in del_attributes):
                    del_attributes = None
                else:
                    print(f"Old attributes: {old_attributes}, Delete attributes: {del_attributes}")
            else:
                del_attributes = None

            add_cmd = current_cmd.get("minor", {}).get("add_object")
            if add_cmd:
                add_obj = add_cmd[0]
                new_attributes = (
                    add_obj.type.lower(), add_obj.color, add_obj.name.lower(), add_obj.location, add_obj.size.lower()
                )
                if all(attr == '' for attr in new_attributes):
                    new_attributes = None
                else:
                    print(f"New attributes: {new_attributes}")
            else:
                new_attributes = None

            if old_attributes == del_attributes and new_attributes:
                fluent.modify_objects_in_use([add_obj], [del_obj])
                #unhandled_objects.remove(obj_type)
                obj_type, obj_color, obj_name, obj_location, obj_size = new_attributes
                unhandled_objects.append(obj_type)
                print(f"Replaced object with new attributes: {new_attributes}")
            else:
                rospy.logerr("Attributes do not match for replacement or new attributes are missing. Ignoring")
                ignored_commands += 1

        elif minor_cmd == "bring_me":
            minor_interrupt_count += 1
            add_cmd = current_cmd.get("minor", {}).get("add_object")
            if add_cmd:
                add_obj = add_cmd[0]
                if obj_type != add_obj.type.lower():
                    fluent.modify_objects_in_use([add_obj], [])
                    unhandled_objects.append(add_obj.type.lower())
                else:
                    ignored_commands += 1
            else:
                ignored_commands += 1

        elif minor_cmd == "change_location":
            minor_interrupt_count += 1
            add_cmd = current_cmd.get("minor", {}).get("add_object")

            if add_cmd:
                add_obj = add_cmd[0]
                new_location = add_obj.location
            else:
                new_location = None

            if new_location and destination_location:
                if new_location != destination_location:
                    age = age ^ 1
                    if destination_location:
                        destination_location = 'table' if age == 1 else 'countertop'
                    changed_locations += 1
                else:
                    ignored_commands += 1
            else:
                ignored_commands += 1

        elif major_cmd == "stop":
            major_interrupt_count += 1
            return
        else:
            ignored_commands += 1

def monitor_func():
    global obj_type, obj_color, obj_name, obj_location, obj_size, age, current_cmd, destination_location
    if fluent.minor_interrupt.get_value():
        old_attributes = (obj_type.lower(), obj_color, obj_name.lower(), obj_location, obj_size.lower())
        old_destination_location = destination_location
        update_current_command()

        new_attributes = (obj_type.lower(), obj_color, obj_name.lower(), obj_location, obj_size.lower())
        fluent.minor_interrupt.set_value(False)

        if new_attributes != old_attributes:
            return True
        if old_destination_location != destination_location:
            raise ChangeLocationException
        return False
    elif fluent.major_interrupt.get_value():
        return MajorInterrupt
    return False


@with_simulated_robot
def move_and_detect(obj_type, obj_size, obj_color):
    global original_pose, current_location, drawer_open_loc, handle_desig
    obj_color = color_map(obj_color)
    if obj_type == "spoon":
        handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig.resolve())
        pose = Pose([1.75, 1.79, 0], [0, 0, 0.533512180079847, 0.8457923821520558])
        drawer_open_loc = AccessingLocation.Location(pose, ["left"])
        NavigateAction([drawer_open_loc.pose]).resolve().perform()
        OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
        spoon.detach(apartment)

        # Detect and pickup the spoon
        LookAtAction([apartment.get_link_pose("handle_cab10_t")]).resolve().perform()

    elif obj_type == "cup":
        handle_desig = ObjectPart(names=['handle_cab1_top_door'], part_of=apartment_desig.resolve())
        pose = Pose([1.2, 1.5, 0], [0, 0, 1, 0])
        drawer_open_loc = AccessingLocation.Location(pose, ["left"])
        NavigateAction([drawer_open_loc.pose]).resolve().perform()
        OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
        LookAtAction([Pose([0.5, 1.6, 1.48])]).resolve().perform()

    else:
        NavigateAction(target_locations=[Pose([1.7, 1.9, 0])]).resolve().perform()
        current_location = "countertop"

        LookAtAction(targets=[pick_pose]).resolve().perform()

    status, object_dict = DetectAction(technique='all').resolve().perform()
    filtered_dict = {key: obj for key, obj in object_dict.items() if (
            (obj_type is None or obj_type.strip() == "" or obj.bullet_world_object.type.lower() == obj_type.lower())
            and (
                    obj_size is None or obj_size.strip() == "" or obj.bullet_world_object.size.lower() == obj_size.lower())
            and (obj_color is None or obj_color == [] or obj.bullet_world_object.color == obj_color))}

    if not filtered_dict:
        rospy.logerr("No objects found matching the criteria.")
        return None

    object_desig = next(iter(filtered_dict.values()))
    original_pose = object_desig.pose
    return object_desig


def announce_pick(name: str, type: str, color: str, location: str, size: str):
    global sleep
    print(f"I will now pick up the {size.lower()} {color.lower()} {type.lower()}")
    # print(f"I am now interruptable for 5 seconds")
    # fluent.activate_subs()
    if sleep:
        time.sleep(5)  # fluent.deactivate_subs()  # print(f"I am not interruptable any more")


def announce_bring(name: str, type: str, color: str, location: str, size: str, destination: str):
    print(f"I will now bring the {size.lower(), color.lower(), type.lower()} to you")
    from_robot_publish("transporting_deliver", True, False, True, "countertop", destination)
    # print(f"I am now interruptable for 10 seconds")
    # fluent.activate_subs()
    global sleep
    if sleep:
        time.sleep(13)  # fluent.deactivate_subs()  # print(f"I am not interruptable any more")


def announce_recovery(old_desig):
    update_object_state(old_desig.name, "recovering", old_desig.pose.position)
    from_robot_publish("Recovery", False, True, True, "table", "countertop")
    print("Recovering from Interrupt")  # print("I am not interruptable here at the moment")


def announce_pick_place(case: str, type: str, color: str, size: str):
    print(
        f"I will now {case.lower()} the {size.lower(), color.lower(), type.lower()}")  # print("I am not interruptable here at the moment")


def place_and_pick_new_obj(old_desig, location, obj_type, obj_size, obj_color):
    global obj_desig, used_arm, grasp, drawer_open_loc

    # Code(lambda: NavigateAction([get_recovery_pose()]).resolve().perform())
    if old_desig.bullet_world_object.type.lower() in ["spoon", "cup"]:
        access_obj()
    else:
        poses = {'bowl': Pose([1.7, 1.9, 0]), 'cereal': Pose([1.7, 2, 0]), 'milk': Pose([1.7, 1.9, 0]), }
        pose = poses.get(old_desig.bullet_world_object.type.lower())
        NavigateAction([pose]).resolve().perform()

    if drawer_open_loc is not None:
        used_arm = "left" if drawer_open_loc.arms[0] == "right" else "right"
    else:
        used_arm = "left"

    PlaceAction(old_desig, [used_arm], [grasp], [location]).resolve().perform()
    if old_desig.bullet_world_object.type.lower() in ["spoon", "cup"]:
        if old_desig.bullet_world_object.type.lower() == "spoon":
            apartment.attach(spoon, 'cabinet10_drawer_top')
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        close_loc = drawer_open_loc.pose
        NavigateAction([close_loc]).resolve().perform()

        CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()

    drawer_open_loc = None
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    obj_desig = move_and_detect(obj_type, obj_size, obj_color)
    if obj_desig:
        if drawer_open_loc is not None:
            used_arm = "left" if drawer_open_loc.arms[0] == "right" else "right"
        else:
            used_arm = "left"
        grasp = "top" if obj_type == "spoon" else "front"
        PickUpAction.Action(obj_desig, used_arm, grasp).perform()
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        if obj_desig.bullet_world_object.type.lower() in ["spoon", "cup"]:
            CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
        ParkArmsAction([Arms.BOTH]).resolve().perform()


def from_robot_publish(step, interrupt, move_arm, move_base, robot_location, destination_location):
    global obj_type, obj_color, obj_name, obj_location, obj_size
    fluent.publish_from_robot(step, interrupt, obj_type, obj_color, obj_name, obj_location, obj_size, move_arm,
                              move_base, robot_location, destination_location)


def statsprint(results):
    print(f"######### Statistic #########")
    print(f"Total Commands: {results['total_commands']}")
    print(f"Objects Replaced: {results['objects_replaced']}")
    print(f"Changed Locations: {results['changed_locations']}")
    print(f"Objects Not in Correct Place: {results['objects_not_correct']}")
    print(f"Ignored Commands: {results['ignored_commands']}")
    print(f"Failure Rate: {results['failure_success_rate']}%")
    print(f"Ignored Commands Rate: {results['ignored_commands_rate']}%")
    print(f"#############################")


with (simulated_robot):
    ###### Prepare robot ######
    ParkArmsAction.Action(Arms.BOTH).perform()
    MoveTorsoAction([0.25]).resolve().perform()
    fluent.activate_subs()
    current_location = "countertop"
    from_robot_publish("set_parameters", True, False, False, current_location, "")
    handled_objects = list()
    unhandled_objects = list()

    while True:
        unhandled_objects = list(fluent.objects_in_use.keys())
        unhandled_objects = [obj for obj in unhandled_objects if obj not in handled_objects]
        previous_states = object_states.copy()  # Keep track of the states before the command
        if not unhandled_objects:
            results = calculate_statistics(minor_interrupt_count, major_interrupt_count, object_states,
                                           ignored_commands, short_str, changed_locations)
            statsprint(results)
            rospy.logwarn("Waiting for next human command")
            # fluent.activate_subs()
            fluent.minor_interrupt.pulsed().wait_for()
            # fluent.deactivate_subs()
            update_current_command()
            rospy.logwarn(
                f"Received command: {current_cmd.get('minor', {}).get('command')}")  # from_robot_publish("set_parameters", True, False, False, current_location, "")

        for obj in unhandled_objects:
            if obj in handled_objects:
                for obj in handled_objects:
                    rospy.logerr(f"Object {obj} was already handled, continuing")
                    ignored_commands += 1
                    from_robot_publish("already_done", True, False, False, current_location, "")

        # Filter out already handled objects
        unhandled_objects = [obj for obj in unhandled_objects if obj not in handled_objects]

        unhandled_object = unhandled_objects[0] if unhandled_objects else None
        if unhandled_object:
            obj = fluent.objects_in_use.get(unhandled_object, None)
        else:
            obj = None

        if obj:
            if obj.type not in objecterinos_typos:
                rospy.logerr(f"Object {obj.type} cant be handled due criteria, continuing")
                ignored_commands += 1
                from_robot_publish("already_done", True, False, False, current_location, "")
                from_robot_publish("task_done", True, False, False, current_location, "")
                if obj.type in unhandled_objects:
                    unhandled_objects.remove(obj.type)


            else:
                obj_type = obj.type
                obj_color = obj.color
                obj_name = obj.name
                obj_location = obj.location
                obj_size = obj.size
                grasp = "top" if obj_type == "spoon" else "front"

                # fluent.minor_interrupt.set_value(False)
                age = fluent.age

                ###### Announce object and wait ######
                announce = Code(lambda: announce_pick(obj_name, obj_type, obj_color, current_location, obj_size))

                ###### Move and detect object ######
                obj_desig = Code(lambda: move_and_detect(obj_type, obj_size, obj_color))

                ###### construct and monitor subplan ######
                plan = announce + obj_desig >> Monitor(monitor_func)

                ###### Execute plan ######
                _, [_, obj_desig] = RetryMonitor(plan, max_tries=5).perform()

                if obj_desig:
                    ###### Park robot ######
                    ParkArmsAction.Action(Arms.BOTH).perform()
                    grasp = "top" if obj_type == "spoon" else "front"
                    if obj_type in ["spoon", "cup"]:
                        used_arm = "left" if drawer_open_loc.arms[0] == "right" else "right"
                        PickUpAction(obj_desig, [used_arm], [grasp]).resolve().perform()

                        ParkArmsAction([Arms.BOTH]).resolve().perform()

                        close_loc = drawer_open_loc.pose
                        NavigateAction([close_loc]).resolve().perform()

                        CloseAction(object_designator_description=handle_desig,
                                    arms=[drawer_open_loc.arms[0]]).resolve().perform()

                        ParkArmsAction([Arms.BOTH]).resolve().perform()
                    else:
                        used_arm = "left"
                        ###### Pickup Object ######
                        from_robot_publish("transporting_fetch", True, True, False, "countertop", "")
                        PickUpAction.Action(obj_desig, used_arm, grasp).perform()
                        update_object_state(obj_desig.name, "being_moved", obj_desig.pose.position)

                        ###### Park robot ######
                        ParkArmsAction.Action(Arms.BOTH).perform()

                    destination_location = 'table' if age == 1 else 'countertop'

                    ###### Announce Navigation action and wait ######
                    announce = Code(
                        lambda: announce_bring(obj_name, obj_type, obj_color, obj_location, obj_size, destination_location))

                    ###### Construct subplan ######
                    plan = Code(lambda: NavigateAction(
                        [get_nav_pose(obj_type, destination_location)]).resolve().perform()) + announce >> Monitor(
                        monitor_func)

                    ###### Construct recovery behaviour (Navigate to island => place object => detect new object => pick up new object ######
                    recover_normal = Code(lambda: announce_recovery(old_desig=obj_desig)) + Code(
                        lambda: place_and_pick_new_obj(obj_desig, original_pose, obj_type, obj_size, obj_color))

                    ###### Construct recovery behaviour (Navigate to island => place object => detect new object => pick up new object ######
                    recover_location = Code(lambda: announce_recovery(old_desig=obj_desig)) + Code(
                        lambda: NavigateAction([get_nav_pose(obj_type, destination_location)]).resolve().perform())

                    ###### Execute plan ######
                    RetryMonitor(plan, max_tries=5, recovery={ChangeLocationException: recover_location,
                                                              PlanFailure: recover_normal}).perform()

                    ###### Hand the object according to Scenario 5 ######
                    from_robot_publish("transporting_deliver", True, True, False, current_location, "")
                    grasp = "top" if obj_type == "spoon" else "front"
                    PlaceAction(obj_desig, [used_arm], [grasp],
                                [get_place_pose(obj_type, destination_location)]).resolve().perform()
                    update_object_state(obj_desig.name, "new_location", obj_desig.pose.position)

                    ParkArmsAction([Arms.BOTH]).resolve().perform()
                    current_snapshot(obj_type)
                    from_robot_publish("task_done", True, False, False, current_location, "")

                    if obj_type in unhandled_objects:
                        unhandled_objects.remove(obj_type)

                    if obj_type not in handled_objects:
                        handled_objects.append(obj_type)

                    obj_type = ""
                    obj_color = ""
                    obj_name = ""
                    obj_location = ""
                    obj_size = ""
                    obj_desig = None
