#!/usr/bin/env python3
import time

from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.external_interfaces.interrupt_actionclient import InterruptClient
from pycram.failure_handling import Retry, RetryMonitor
from pycram.fluent import Fluent
from pycram.language import Monitor, Code
from pycram.plan_failures import MajorInterrupt
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher

try:
    from speech_processing.msg import dict_object
except ModuleNotFoundError as e:
    rospy.logwarn("Failed to import speech_processing messages, frontiers can not be used")

world = BulletWorld("DIRECT")
viz = VizMarkerPublisher()
robot = Object("pr2", "robot", "pr2.urdf", pose=Pose([1, 2, 0]))
apartment = Object("apartment", "environment", "apartment.urdf")

milk1 = Object("milk1", "milk", "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[0, 0, 1, 1])
milk2 = Object("milk2", "milk", "milk.stl", pose=Pose([2.5, 1.7, 1.02]), color=[1, 0, 0, 1], size="big")
bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([2.5, 2.2, 1.02]), color=[1, 1, 1, 1])
spoon = Object("spoon", "spoon", "spoon.stl", pose=Pose([2.4, 2.2, 0.85]), color=[0, 0, 1, 1])
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([2.5, 2.4, 1.05]), color=[0, 1, 0, 1])
cup = Object("cup", "cup", "jeroen_cup.stl", pose=Pose([2.5, 1.85, 0.95]))

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
age = ""
place_pose = None
nav_pose = None
original_pose = None
obj_desig = None
handle_desig = None
current_cmd = None
current_location = None
drawer_open_loc = None
used_arm = "left"
grasp = "front"


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


def age_map(age):
    global nav_pose, place_pose
    if age == 1:
        nav_pose = Pose([4, 3.8, 0])
        place_pose = Pose([4.8, 3.8, 0.8])
    else:
        nav_pose = Pose([3.9, 3.8, 0], [0, 0, 1, 0])
        place_pose = Pose([3, 3.8, 1.02], [0, 0, 1, 0])


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


def access_obj():
    global drawer_open_loc, handle_desig, obj_desig
    if obj_desig.bullet_world_object.type.lower()  == "spoon":
        handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig.resolve())
        drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(),
                                            robot_desig=robot_desig.resolve()).resolve()
        NavigateAction([drawer_open_loc.pose]).resolve().perform()
        OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
        # spoon.detach(apartment)


def get_recovery_pose() -> Pose:
    global obj_desig, obj_type
    pose = None
    if obj_desig.bullet_world_object.type.lower() == "spoon":
        access_obj()
        pose = drawer_open_loc.pose
    poses = {
        'bowl': Pose([1.7, 1.9, 0]),
        'cereal': Pose([1.7, 2, 0]),
        'milk': Pose([1.7, 1.9, 0]),
        'cup': Pose([1.7, 1.9, 0]),
        'spoon': pose
    }
    pose = poses.get(obj_type)
    return pose


def get_nav_pose(object_type, location):
    global current_location
    poses = {
        ('bowl', 'table'): Pose([4, 3.8, 0]),
        ('bowl', 'countertop'): Pose([3.9, 3.8, 0], [0, 0, 1, 0]),
        ('cereal', 'table'): Pose([4, 3.6, 0]),
        ('cereal', 'countertop'): Pose([3.9, 3.6, 0], [0, 0, 1, 0]),
        ('milk', 'table'): Pose([4, 4, 0]),
        ('milk', 'countertop'): Pose([3.9, 4, 0], [0, 0, 1, 0]),
        ('spoon', 'table'): Pose([4.2, 3.7, 0]),
        ('spoon', 'countertop'): Pose([3.7, 3.7, 0], [0, 0, 1, 0]),
        ('cup', 'table'): Pose([4.3, 3.9, 0]),
        ('cup', 'countertop'): Pose([3.8, 3.9, 0], [0, 0, 1, 0]),
    }

    pose = poses.get((object_type, location))
    current_location = location
    return pose


def update_current_command():
    global current_cmd, obj_type, obj_color, obj_name, obj_location, obj_size, unhandled_objects
    current_cmd = fluent.next_command()

    if current_cmd:
        minor_cmd = current_cmd.get("minor", {}).get("command")
        major_cmd = current_cmd.get("major", {}).get("command")

        if minor_cmd == "setting_breakfast":
            objects_to_add = [
                dict_object(type="bowl", color="", name="", location="", size=""),
                dict_object(type="milk", color="", name="", location="", size=""),
                dict_object(type="cereal", color="", name="", location="", size=""),
                dict_object(type="cup", color="", name="", location="", size=""),
                dict_object(type="spoon", color="", name="", location="", size="")
            ]
            fluent.modify_objects_in_use(objects_to_add, [])

        # elif minor_cmd == "object":
        elif minor_cmd == "replace_object":
            old_attributes = (obj_type.lower(), obj_color, obj_name.lower(), obj_location, obj_size.lower())

            del_cmd = current_cmd.get("minor", {}).get("del_object")
            add_obj = None
            del_obj = None
            if del_cmd:
                del_obj = del_cmd[0]
                del_attributes = (
                    del_obj.type.lower(), del_obj.color, del_obj.name.lower(), del_obj.location, del_obj.size.lower())
            else:
                del_attributes = None

            add_cmd = current_cmd.get("minor", {}).get("add_object")
            if add_cmd:
                add_obj = add_cmd[0]
                new_attributes = (
                    add_obj.type.lower(), add_obj.color, add_obj.name.lower(), add_obj.location, add_obj.size.lower())
            else:
                new_attributes = None

            if old_attributes == del_attributes and new_attributes:
                fluent.modify_objects_in_use([add_obj], [del_obj])
                unhandled_objects.remove(obj_type)
                obj_type, obj_color, obj_name, obj_location, obj_size = new_attributes
                unhandled_objects.append(obj_type)
        elif minor_cmd == "bring_me":
            add_cmd = current_cmd.get("minor", {}).get("add_object")
            if add_cmd:
                add_obj = add_cmd[0]
                fluent.modify_objects_in_use([add_obj], [])
                unhandled_objects.append(add_obj.type.lower())
                print(f"Added {add_obj.type.lower()}")
        elif major_cmd == "stop":
            return


def monitor_func():
    global obj_type, obj_color, obj_name, obj_location, obj_size, age, current_cmd
    if fluent.minor_interrupt.get_value():
        old_attributes = (obj_type.lower(), obj_color, obj_name.lower(), obj_location, obj_size.lower())
        update_current_command()

        new_attributes = (obj_type.lower(), obj_color, obj_name.lower(), obj_location, obj_size.lower())
        fluent.minor_interrupt.set_value(False)

        if new_attributes != old_attributes:
            return True
        return False
    elif fluent.major_interrupt.get_value():
        return MajorInterrupt
    return False


@with_simulated_robot
def move_and_detect(obj_type, obj_size, obj_color):
    global original_pose, current_location
    obj_color = color_map(obj_color)

    if obj_type == "spoon":
        global drawer_open_loc, handle_desig
        handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig.resolve())
        drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(),
                                            robot_desig=robot_desig.resolve()).resolve()
        NavigateAction([drawer_open_loc.pose]).resolve().perform()
        OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
        spoon.detach(apartment)

        # Detect and pickup the spoon
        LookAtAction([apartment.get_link_pose("handle_cab10_t")]).resolve().perform()

    else:
        NavigateAction(target_locations=[Pose([1.7, 1.9, 0])]).resolve().perform()
        current_location = "countertop"

        LookAtAction(targets=[pick_pose]).resolve().perform()
        # object_desig = DetectAction(BelieveObject(types=[obj_type] if obj_type else None,
        #                                          sizes=[obj_size] if obj_size else None,
        #                                          colors=[obj_color] if obj_color else None)).resolve().perform()
    status, object_dict = DetectAction(technique='all').resolve().perform()
    filtered_dict = {
        key: obj
        for key, obj in object_dict.items()
        if ((obj_type is None or obj_type.strip() == "" or obj.bullet_world_object.type.lower() == obj_type.lower()) and
            (obj_size is None or obj_size.strip() == "" or obj.bullet_world_object.size.lower() == obj_size.lower()) and
            (obj_color is None or obj_color == [] or obj.bullet_world_object.color == obj_color))
    }

    object_desig = next(iter(filtered_dict.values()))
    original_pose = object_desig.pose
    return object_desig


def announce_pick(name: str, type: str, color: str, location: str, size: str):
    print(f"I will now pick up the {size.lower()} {color.lower()} {type.lower()} at location {location.lower()} ")
    print(f"I am now interruptable for 5 seconds")
    fluent.activate_subs()
    time.sleep(5)
    fluent.deactivate_subs()
    print(f"I am not interruptable any more")


def announce_bring(name: str, type: str, color: str, location: str, size: str, destination: str):
    print(f"I will now bring the {size.lower(), color.lower(), type.lower()} to you")
    from_robot_publish("transporting", True, False, True, "countertop", destination)
    print(f"I am now interruptable for 10 seconds")
    fluent.activate_subs()
    time.sleep(10)
    fluent.deactivate_subs()
    print(f"I am not interruptable any more")


def announce_recovery():
    from_robot_publish("Recovery", False, True, True, "table", "countertop")
    print("Recovering from Interrupt")
    print("I am not interruptable here at the moment")


def announce_pick_place(case: str, type: str, color: str, size: str):
    print(f"I will now {case.lower()} the {size.lower(), color.lower(), type.lower()}")
    print("I am not interruptable here at the moment")


def place_and_pick_new_obj(old_desig, location, obj_type, obj_size, obj_color):
    global obj_desig, used_arm, grasp

    # Code(lambda: NavigateAction([get_recovery_pose()]).resolve().perform())
    if old_desig.bullet_world_object.type.lower() == "spoon":
        access_obj()
    else:
        poses = {
            'bowl': Pose([1.7, 1.9, 0]),
            'cereal': Pose([1.7, 2, 0]),
            'milk': Pose([1.7, 1.9, 0]),
        }
        pose = poses.get(old_desig.bullet_world_object.type.lower())
        NavigateAction([pose]).resolve().perform()

    used_arm = "left" if drawer_open_loc.arms[0] == "right" else "right"

    PlaceAction(old_desig, [used_arm], [grasp], [location]).resolve().perform()
    if old_desig.bullet_world_object.type.lower() == "spoon":
        apartment.attach(spoon, 'cabinet10_drawer_top')
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        close_loc = drawer_open_loc.pose
        NavigateAction([close_loc]).resolve().perform()

        CloseAction(object_designator_description=handle_desig,
                    arms=[drawer_open_loc.arms[0]]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()
    obj_desig = move_and_detect(obj_type, obj_size, obj_color)
    used_arm = "left"
    PickUpAction.Action(obj_desig, "left", "front").perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()


def from_robot_publish(step, interrupt, move_arm, move_base, robot_location, destination_location):
    global obj_type, obj_color, obj_name, obj_location, obj_size
    fluent.publish_from_robot(step, interrupt, obj_type, obj_color, obj_name, obj_location, obj_size, move_arm,
                              move_base, robot_location, destination_location)


with simulated_robot:
    ###### Prepare robot ######
    ParkArmsAction.Action(Arms.BOTH).perform()

    MoveTorsoAction([0.25]).resolve().perform()
    # handle_desig = ObjectPart(names=['handle_cab1_top_door'], part_of=apartment_desig.resolve())
    # handle_desig = ObjectPart(names=['cabinet1_door_top_left'], part_of=apartment_desig.resolve())

    # open_loc = AccessingLocation(handle_desig=handle_desig.resolve(),
    #                                     robot_desig=robot_desig.resolve()).resolve()
    # NavigateAction([open_loc.pose]).resolve().perform()
    # OpenAction(object_designator_description=handle_desig, arms=[open_loc.arms[0]]).resolve().perform()

    # drawer_open_loc[0]
    current_location = "countertop"
    from_robot_publish("initial", True, False, False, current_location, "")
    handled_objects = list()
    unhandled_objects = list()

    while True:
        unhandled_objects = list(fluent.objects_in_use.keys())
        unhandled_objects = [obj for obj in unhandled_objects if obj not in handled_objects]
        if not unhandled_objects:
            print("Waiting for next human command")
            fluent.activate_subs()
            fluent.minor_interrupt.pulsed().wait_for()
            fluent.deactivate_subs()
            update_current_command()
            print(f"Received command: {current_cmd.get('minor', {}).get('command')}")

        for obj in unhandled_objects:
            if obj in handled_objects:
                print(f"Object {obj} was already handled, continuing")
                from_robot_publish("already_done", False, False, False, current_location, "")

        # Filter out already handled objects
        unhandled_objects = [obj for obj in unhandled_objects if obj not in handled_objects]

        unhandled_object = unhandled_objects[0] if unhandled_objects else None
        print(f"Unhandled objects: {unhandled_objects}")
        if unhandled_object:
            obj = fluent.objects_in_use.get(unhandled_object, None)
        else:
            obj = None

        if obj:
            obj_type = obj.type
            obj_color = obj.color
            obj_name = obj.name
            obj_location = obj.location
            obj_size = obj.size
            grasp = "top" if obj_type == "spoon" else "front"

            fluent.minor_interrupt.set_value(False)
            age = fluent.age

            ###### Announce object and wait ######
            announce = Code(lambda: announce_pick(obj_name, obj_type, obj_color, obj_location, obj_size))

            ###### Move and detect object ######
            obj_desig = Code(lambda: move_and_detect(obj_type, obj_size, obj_color))

            ###### construct and monitor subplan ######
            plan = announce + obj_desig >> Monitor(monitor_func)

            ###### Execute plan ######
            _, [_, obj_desig] = RetryMonitor(plan, max_tries=5).perform()

            ###### Park robot ######
            ParkArmsAction.Action(Arms.BOTH).perform()
            grasp = "top" if obj_type == "spoon" else "front"
            if obj_type == "spoon":
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
                from_robot_publish("picking_up", True, True, False, "countertop", "")
                PickUpAction.Action(obj_desig, used_arm, grasp).perform()

                ###### Park robot ######
                ParkArmsAction.Action(Arms.BOTH).perform()

            destination_location = 'table' if age == 1 else 'countertop'

            ###### Announce Navigation action and wait ######
            announce = Code(
                lambda: announce_bring(obj_name, obj_type, obj_color, obj_location, obj_size, destination_location))

            ###### Construct subplan ######
            plan = Code(lambda: NavigateAction([get_nav_pose(obj_type, destination_location)]).resolve().perform()) + announce >> Monitor(
                monitor_func)

            ###### Construct recovery behaviour (Navigate to island => place object => detect new object => pick up new object ######
            recover = Code(lambda: announce_recovery()) + Code(lambda: place_and_pick_new_obj(obj_desig, original_pose, obj_type, obj_size, obj_color))

            ###### Execute plan ######
            RetryMonitor(plan, max_tries=5, recovery=recover).perform()

            ###### Hand the object according to Scenario 5 ######
            from_robot_publish("placing", True, True, False, current_location, "")
            grasp = "top" if obj_type == "spoon" else "front"
            PlaceAction(obj_desig, [used_arm], [grasp],
                        [get_place_pose(obj_type, destination_location)]).resolve().perform()
            ParkArmsAction([Arms.BOTH]).resolve().perform()
            from_robot_publish("task_done", False, False, False, current_location, "")

            if obj_type in unhandled_objects:
                unhandled_objects.remove(obj_type)

            if obj_type not in handled_objects:
                handled_objects.append(obj_type)
