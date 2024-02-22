import rospy

import pycram.plan_failures
from demos.pycram_transporting_demo.init_setup import breakfast_context_apartment
from pycram.bullet_world import BulletWorld, Object
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot

pick_pose = Pose([2.7, 2.15, 1])

world = BulletWorld()
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1, 2, 0]))
robot_desig = BelieveObject(names=["pr2"])
apart_desig = BelieveObject(names=["apartment"])
current_context = breakfast_context_apartment  # Or dinner_context, depending on the scenario
current_context.spawn_objects()
objects = current_context.get_all_objects()

def search_for_object(object_type):
    # Define the probabilities for each location given an object
    location_probabilities = {"spoon": {"drawer": 0.6, "dishwasher": 0.3, "countertop": 0.1},
        "bowl": {"cupboard": 0.5, "dishwasher": 0.3, "countertop": 0.2},
        # Add more objects and their location probabilities as needed
    }

    # Check if the object is in the predefined list
    if object_type in location_probabilities:
        # Get the location probabilities for the given object
        probabilities = location_probabilities[object_type]

        # Find the location with the highest probability
        most_likely_location = max(probabilities, key=probabilities.get)

        return most_likely_location
    else:
        return "Unknown"  # Object not recognized


with (simulated_robot):
    for obj in objects:
        MoveTorsoAction([0.25]).resolve().perform()
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()
        NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()
        LookAtAction(targets=[pick_pose]).resolve().perform()
        MoveTorsoAction([0.25]).resolve().perform()

        object_desig = DetectAction(technique='all').resolve().perform()
        object_dict = object_desig[1]
        for key, value in object_dict.items():
            if object_dict[key].type == "Cutlery" or object_dict[key].type == ObjectType.BOWL:
                grasp = "top"
                marging_cm = 0.08
            else:
                grasp = "front"
                marging_cm = 0.2
            arm = "left"
            PickUpAction(object_dict[key], ["left"], [grasp]).resolve().perform()
            ParkArmsAction([Arms.BOTH]).resolve().perform()

            place_pose, nav_pose = find_reachable_location_and_nav_pose(enviroment_link="table_area_main",
                                                                        enviroment_desig=apart_desig.resolve(),
                                                                        object_desig=object_dict[key],
                                                                        robot_desig=robot_desig.resolve(), arm="left",
                                                                        world=world, marging_cm=marging_cm)
            if not nav_pose:
                rospy.logerr("No location found")
            else:
                NavigateAction(target_locations=[nav_pose]).resolve().perform()
                MoveTorsoAction([0.25]).resolve().perform()
                PlaceAction(object_dict[key], ["left"], [grasp], [place_pose]).resolve().perform()

                ParkArmsAction([Arms.BOTH]).resolve().perform()
                NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()
                MoveTorsoAction([0.25]).resolve().perform()
