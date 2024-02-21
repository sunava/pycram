import pycram.plan_failures
from pycram.bullet_world import BulletWorld, Object
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot
from demos.pycram_transporting_demo.init_setup import breakfast_context_apartment

world = BulletWorld()
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1, 2, 0]))
robot_desig = BelieveObject(names=["pr2"])
apart_desig = BelieveObject(names=["apartment"])
current_context = breakfast_context_apartment  # Or dinner_context, depending on the scenario
current_context.spawn_objects()
objects = current_context.get_all_objects()


def search_for_object(obj):
    location_to_search = current_context.search_locations(obj)
    if location_to_search == "drawer" or location_to_search == "dishwasher" or location_to_search == "island_countertop"\
            or location_to_search == "cupboard" or location_to_search == "cabinet":
        # Finding and navigating to the location
        handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apart_desig.resolve())
        drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(),
                                            robot_desig=robot_desig.resolve()).resolve()
        NavigateAction([drawer_open_loc.pose]).resolve().perform()

        OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        close_loc = drawer_open_loc.pose
        close_loc.position.y += 0.1
        NavigateAction([close_loc]).resolve().perform()

        CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()
    else:
        pose = SemanticCostmapLocation(urdf_link_name=location_to_search,
                                             part_of=apart_desig.resolve()).resolve()
        placing_loc = CostmapLocation(target=pose, reachable_for=robot_desig.resolve()).resolve()
        NavigateAction([placing_loc.pose]).resolve().perform()

with (simulated_robot):
    for obj in objects:
        MoveTorsoAction([0.25]).resolve().perform()
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        search_for_object(obj)

        # ParkArmsAction([Arms.BOTH]).resolve().perform()
        # NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()
        # LookAtAction(targets=[pick_pose]).resolve().perform()
        # MoveTorsoAction([0.25]).resolve().perform()
        #
        # object_desig = DetectAction(technique='all').resolve().perform()
        # object_dict = object_desig[1]
        # for key, value in object_dict.items():
        #     if object_dict[key].type == "Cutlery" or object_dict[key].type == ObjectType.BOWL:
        #         grasp = "top"
        #         marging_cm = 0.08
        #     else:
        #         grasp = "front"
        #         marging_cm = 0.2
        #     arm = "left"
        #     PickUpAction(object_dict[key], ["left"], [grasp]).resolve().perform()
        #     ParkArmsAction([Arms.BOTH]).resolve().perform()
        #
            # place_pose, nav_pose = find_reachable_location_and_nav_pose(enviroment_link="table_area_main",
            #                                                             enviroment_desig=kitchen_desig.resolve(),
            #                                                             object_desig=object_dict[key],
            #                                                             robot_desig=robot_desig.resolve(), arm="left",
            #                                                             world=world, marging_cm=marging_cm)
        #     if not nav_pose:
        #         print("No location found")
        #     else:
        #
        #         print("Moving to pose")
        #         NavigateAction(target_locations=[nav_pose]).resolve().perform()
        #         MoveTorsoAction([0.25]).resolve().perform()
        #         PlaceAction(object_dict[key], ["left"], [grasp], [place_pose]).resolve().perform()
        #
        #         ParkArmsAction([Arms.BOTH]).resolve().perform()
        #         NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()
        #         MoveTorsoAction([0.25]).resolve().perform()
