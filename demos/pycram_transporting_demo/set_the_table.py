import rospy

from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.enums import ObjectType

from pycram.ros.viz_marker_publisher import VizMarkerPublisher

world = BulletWorld()
#v = VizMarkerPublisher()
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1, 2, 0]))
apartment = Object("apartment", ObjectType.ENVIRONMENT, "apartment-small.urdf")

milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])
cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([2.5, 2.3, 1.05]), color=[0, 1, 0, 1])
#spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([2.4, 2.2, 0.85]), color=[0, 0, 1, 1])
bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.38, 2.2, 1.02]), color=[1, 1, 0, 1])
#apartment.attach(spoon, 'cabinet10_drawer_top')
world.get_objects_by_name("floor")[0].set_color([0.824, 0.706, 0.549, 0.8])
#apartment.set_color([0.5, 0.5, 0.5, 0.7])
pick_pose = Pose([2.7, 2.15, 1])

robot_desig = BelieveObject(names=["pr2"])
cereal_desig = BelieveObject(names=["cereal"])
#apartment_desig = BelieveObject(names=["apartment"])





@with_simulated_robot
def move_and_detect(obj_type):
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(technique='default', object_type=obj_type).resolve().perform()

    return object_desig
def create_semantic_costmap_location_reachable(urdf_link_name, part_of, robot, reachable_arm=None):
    """
    Creates a SemanticCostmapLocation that is filtered by reachability constraints.

    :param urdf_link_name: Name of the URDF link for which a location distribution is calculated.
    :param part_of: The object of which the URDF link is a part.
    :param robot: The robot for which reachability should be calculated.
    :param reachable_arm: Specific arm name to check reachability, if applicable.
    :return: A generator yielding SemanticCostmapLocation.Location instances that are reachable.
    """
    sem_costmap_loc = SemanticCostmapLocation(urdf_link_name=urdf_link_name, part_of=part_of)
    rospy.loginfo("SemanticCostmapLocation found")

    for sem_location in sem_costmap_loc:
        try:
            world.current_bullet_world.add_vis_axis(sem_location.pose)
            # rospy.loginfo("Checking if reachable")
            # reachable_location = CostmapLocation(
            #     target=sem_location.pose,
            #     reachable_for=robot,
            #     reachable_arm=reachable_arm
            # ).resolve()
            # rospy.loginfo("Location is reachable")
            yield sem_location

        except StopIteration:
            pass
    # Exit the loop when there are no more elements to process
    # Properly handle StopIteration by breaking the loop
    rospy.loginfo("No more reachable locations.")

with (simulated_robot):
    kitchen_desig = ObjectDesignatorDescription(names=["apartment"])
    # create_semantic_costmap_location_reachable(urdf_link_name="table_area_main", part_of=kitchen_desig.resolve(),
    #                                            robot=robot_desig.resolve(), reachable_arm="left")
    location_desig = SemanticCostmapLocation(urdf_link_name="island_countertop", part_of=kitchen_desig.resolve(),
                                              for_object=cereal_desig.resolve())
    for location in location_desig:
         world.current_bullet_world.add_vis_axis(location_desig.resolve().pose)
         try:
             x = location.pose.pose.position.x
             y = location.pose.pose.position.y
             z = location.pose.pose.position.z
             print("calculating reachable location")
             reachable_location = CostmapLocation(
                 target=Pose([x, y, z]),
                 reachable_for=robot_desig.resolve(),
                 reachable_arm="left"
             ).resolve()
             print("Location is reachable")
             world.current_bullet_world.add_vis_axis(reachable_location.pose)
         except StopIteration:
             pass
    print("No location found")
    # print(location_desig)
    # You would use this function like this:
    # reachable_locations = create_semantic_costmap_location_reachable("table_area_main", kitchen_desig.resolve(), robot_desig.resolve(), "left")
    # # Example: Iterate through reachable locations and process them
    # for location in reachable_locations:
    #     # Process each reachable location
    #     print(location)

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
    #     else:
    #         grasp = "front"
    #     # PickUpAction(object_dict[key], ["left"], [grasp]).resolve().perform()
    #     # ParkArmsAction([Arms.BOTH]).resolve().perform()
    #     # place_loc = CostmapLocation(target=Pose([4.8, 3.55, 0.8]), reachable_for=robot_desig.resolve(),
    #     #                             reachable_arm="left").resolve()
    #     # NavigateAction(target_locations=[place_loc.pose]).resolve().perform()
    #     # MoveTorsoAction([0.25]).resolve().perform()
    #
    #     apart_desig = ObjectDesignatorDescription(names=["apartment"])
    #     location_desig = create_semantic_costmap_location_reachable("table_area_main", apart_desig.resolve(), robot_desig.resolve(), "left")
    #     print(location_desig)
        # location = location_desig.resolve()
        # locatin_pose = location.pose
        # place_loc = Pose([locatin_pose.pose.position.x, locatin_pose.pose.position.y, locatin_pose.pose.position.z])
        # print(place_loc)
        # print(locatin_pose)
        #
        #
        # # Now, create a CostmapLocation instance with the resolved target pose, considering reachability and visibility
        # costmap_location = CostmapLocation(
        #     target=place_loc, reachable_for=robot_desig.resolve(), reachable_arm="left").resolve()
        #     #visible_for=robot_desig.resolve(),  # Optionally, the robot for which visibility should be calculated
        #       # Replace 'arm_name' with the specific arm name, if applicable
        #
        #
        # print(costmap_location)
        # print("test")
        # Resolve the costmap location to get a reachable and visible location
        # reachable_visible_location = costmap_location.ground()
        #
        #
        #
        #
        # PlaceAction(object_dict[key], ["left"], [grasp], [Pose([4.6, 3.55, 0.8])]).resolve().perform()
        # ParkArmsAction([Arms.BOTH]).resolve().perform()
        # NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()
        # MoveTorsoAction([0.25]).resolve().perform()