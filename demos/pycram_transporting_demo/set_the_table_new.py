import rospy

from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose
from pycram.process_module import simulated_robot
from demos.pycram_transporting_demo.init_setup import test_context_apartment, breakfast_context_apartment
from pycram.ros.viz_marker_publisher import VizMarkerPublisher

# Initialize the simulation world and visual markers
world = BulletWorld()
VizMarkerPublisher()

# Initialize the robot
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1, 2, 0]))
robot_desig = BelieveObject(names=["pr2"])

# Define environment and objects
# todo rework this and use the proba model
apart_desig = BelieveObject(names=["apartment"])
current_context = breakfast_context_apartment  # or breakfast_context_apartment
current_context.spawn_objects()
objects = current_context.get_all_objects()


def search_for_object(obj_name):
    location_to_search = current_context.search_locations(obj_name)
    if location_to_search in ["drawer", "dishwasher", "cupboard", "cabinet"]:
        grasp, arm, detected_object = access_and_pickup(location_to_search, obj_name, open_container=True)
    elif location_to_search in ["island_countertop"]:
        grasp, arm, detected_object = access_and_pickup(location_to_search, obj_name)
    else:
        rospy.logerr("Location not found")
        #todo bowl is not yet found i need to use enums
    return grasp, arm, detected_object


def access_and_pickup(location_to_search, target_object, open_container=False):
    """
    Accesses a specified location and picks a specified object. Optionally, it can also navigate to and open a container.

    Args:
    - location_to_search: The location to search for the object.
    - target_object: The object to pick up.
    - open_container: A boolean indicating whether to navigate to and open a container before picking up the object.
    """
    #todo maybe a check which arm is free?
    arm = "left"  # Default arm, can be made dynamic or parameterized
    link_name = get_link_name_from_location(location_to_search)
    link_pose = current_context.environment_object.get_link_pose(link_name)

    if open_container:
        #todo this needs to be handled differently
        current_context.environment_object.detach(current_context.spoon_)  # Assuming spoon_ needs to be detached
        handle_desig = ObjectPart(names=[link_name], part_of=apart_desig.resolve())
        drawer_open_location = AccessingLocation(handle_desig=handle_desig.resolve(),
                                                 robot_desig=robot_desig.resolve()).resolve()
        NavigateAction([drawer_open_location.pose]).resolve().perform()
        OpenAction(object_designator_description=handle_desig, arms=[drawer_open_location.arms[0]]).resolve().perform()
        arm = select_alternate_arm(drawer_open_location.arms[0])
    else:
        #todo this should be from KB depending on the location
        location_pose = Pose([1.7, 2, 0])
        NavigateAction(target_locations=[location_pose]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()
    LookAtAction([link_pose]).resolve().perform()
    #todo i guess i should do "all" and then check for types
    status, object_dict = DetectAction(technique='specific', object_type=target_object).resolve().perform()

    if status:
        for key, value in object_dict.items():
            detected_object = object_dict[key]
            grasp = pickup_target_object(detected_object, arm)
    else:
        rospy.logerr("Object not found")
        grasp, detected_object = None, None

    if open_container:
        CloseAction(object_designator_description=handle_desig, arms=[drawer_open_location.arms[0]]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()
    return grasp, arm, detected_object

def place_object(grasp_type, target_location, detected_object, arm):
    """
    Places an object at a specified location based on the grasp type and the target location.

    Args:
    - grasp_type: The type of grasp used ('top' or 'front').
    - target_location: The target location to place the object (e.g., 'table').
    - detected_object: The object that has been detected and is to be placed.
    - arm: The arm used to perform the operation.
    """
    # Set margin based on grasp type
    if grasp_type == "top":
        margin_cm = 0.08
    elif grasp_type == "front":
        margin_cm = 0.2
    else:
        margin_cm = 0.1  # Default margin if grasp type is unspecified

    # Set environment link based on target location
    if target_location == "table":
        environment_link = "table_area_main"
    else:
        environment_link = target_location  # Default to using target_location as the environment link if unspecified

    # Find a reachable location and navigation pose
    place_pose, nav_pose = find_reachable_location_and_nav_pose(enviroment_link=environment_link,
                                                                enviroment_desig=apart_desig.resolve(),
                                                                object_desig=detected_object,
                                                                robot_desig=robot_desig.resolve(),
                                                                arm=arm,
                                                                world=world,
                                                                margin_cm=margin_cm)
    # Check if a navigation pose was found
    if not nav_pose:
        rospy.logerr("No navigable location found")
        return False

    # Navigate to the location, adjust the torso, and place the object
    NavigateAction(target_locations=[nav_pose]).resolve().perform()
    MoveTorsoAction([0.25]).resolve().perform()  # Adjust torso height as needed
    PlaceAction(detected_object, [arm], [grasp_type], [place_pose]).resolve().perform()

    return True


def pickup_target_object(detected_object, arm):
    ParkArmsAction([arm]).resolve().perform()
    if detected_object.type == "spoon" or detected_object.type == "bowl":
        grasp = "top"
    else:
        grasp = "front"
    PickUpAction(detected_object, [arm], [grasp]).resolve().perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    return grasp


def select_alternate_arm(current_arm):
    # Selects the alternate arm based on the current arm.
    return "right" if current_arm == "left" else "left"


def get_link_name_from_location(location):
    # This function retrieves handle name based on the location.
    if location == "drawer":
        return "handle_cab10_t"
    elif location == "countertop" or location == "island_countertop":
        return "island_countertop"
    return None  # Add more conditions as needed

def set_the_table(target_location, context):
    rospy.loginfo("Setting the table")
    with simulated_robot:
        for obj in objects:
            MoveTorsoAction([0.25]).resolve().perform()
            ParkArmsAction([Arms.BOTH]).resolve().perform()
            grasp, arm, detected_object = search_for_object(obj)
            place_object(grasp, target_location, detected_object, arm)

set_the_table("table", "breakfast")

#todo allgemein failure handling fehlt noch und mehr generic und die locations besondern
# fuer hin und her transporting muessten anders sein
