import time
from enum import Enum

import rospy.core

from demos.pycram_clean_the_table_demo.utils.misc import *
from pycram.process_module import real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher

from pycram.external_interfaces.knowrob import get_table_pose, get_table_test_pose, get_handle_pos

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Fork", "Metalbowl"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)


# x pose of the end of the couch table
table_pose = 4.84
# 5.2

# Initialize the Bullet world for simulation
world = BulletWorld("DIRECT")

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")
giskardpy.init_giskard_interface()

robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
apart_desig = BelieveObject(names=["kitchen"])
# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)

handle_name = "sink_area_dish_washer_door_handle"

door_name = "sink_area_dish_washer_door"

move_to_the_middle_pose = [2.2, 1.85, 0]
move_to_the_middle_left_side_dishwasher = [2.2, -0.1, 0]
giskardpy.sync_worlds()


class PlacingXPose(Enum):
    """
    Differentiate the z pose for placing
    """
    CUTLERY = 2.376
    SPOON = 2.376
    FORK = 2.376
    PLASTICKNIFE = 2.376
    KNIFE = 2.376
    METALBOWL = 2.83
    METALMUG = 2.79
    METALPLATE = 2.8


class PlacingYPose(Enum):
    """
    Differentiate the z pose for placing
    """
    CUTLERY = -1.59
    SPOON = -1.59
    FORK = -1.59
    PLASTICKNIFE = -1.59
    KNIFE = -1.59
    METALBOWL = -1.73
    METALMUG = -1.75
    METALPLATE = -1.65


def pickup_and_place_objects(sorted_obj: list):
    """
    For picking up and placing the objects in the given object designator list.

    :param sorted_obj: the distance sorted list of seen object designators.
    """
    global move_to_the_middle_pose, move_to_the_middle_left_side_dishwasher, table_pose, CUTLERY

    for value in range(len(sorted_obj)):
        print("first navigation")
        z = 0.48
        # define grasping pose
        grasp = "front"
        if sorted_obj[value].type in CUTLERY:
            sorted_obj[value].type = "Cutlery"
            sorted_obj[value].pose.position.z = 0.43

        if sorted_obj[value].type == "Metalbowl":
            sorted_obj[value].pose.position.z -= 0.02

        if sorted_obj[value].type in ["Metalbowl", "Cutlery"]:
            grasp = "top"

        if sorted_obj[value].type == "Metalplate":
            TalkingMotion("Can you please give me the plate on the table.").resolve().perform()
            MoveGripperMotion("open", "left").resolve().perform()
            time.sleep(3)
            TalkingMotion("Grabing.").resolve().perform()
            MoveGripperMotion("close", "left").resolve().perform()
            z = 0.495
        else:
            # change object x pose if the grasping pose is too far in the table
            if sorted_obj[value].type == "Cutlery" and sorted_obj[value].pose.position.x > table_pose + 0.125:
                sorted_obj[value].pose.position.x -= 0.08

            TalkingMotion("Picking up with: " + grasp).resolve().perform()
            try_pick_up(robot, sorted_obj[value], grasp)

        # placing the object
        ParkArmsAction([Arms.LEFT]).resolve().perform()

        NavigateAction(target_locations=[Pose(move_to_the_middle_pose, [0, 0, 1, 0])]).resolve().perform()

        if sorted_obj[value].type == "Metalplate" or sorted_obj[value].type == "Metalbowl":
            MoveJointsMotion(["arm_roll_joint"], [-1.5]).resolve().perform()
        x_y_pos = get_pos(sorted_obj[value].type)

        NavigateAction(
            target_locations=[Pose(move_to_the_middle_left_side_dishwasher, [0, 0, -1, 1])]).resolve().perform()

        x_pos = x_y_pos[0]
        y_pos = x_y_pos[1]

        # navigate_to(2.0, -0.2, "dishwasher")

        if x_pos >= 2.75:
            navigate_to("dishwasher_left")
        else:
            navigate_to("dishwasher_right")

        TalkingMotion("Placing").resolve().perform()
        grasp = "front"

        PlaceAction(sorted_obj[value], ["left"], [grasp],
                    [Pose([x_pos, y_pos, z])]).resolve().perform()
        #For the safety in cases where the HSR is not placing, better drop the object to not colide with the kitchen drawer when moving to parkArms arm config
        MoveGripperMotion("open","left").resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()

        # TalkingMotion("Navigating").resolve().perform()
        # navigate_to(3.9, 2, "popcorn table")

        # navigates back if a next object exists
        if value + 1 < len(sorted_obj):
            NavigateAction(target_locations=[Pose(move_to_the_middle_pose, [0, 0, 1, 1])]).resolve().perform()
            navigate_to("couch table", sorted_obj[value + 1].pose.position.y)


def get_pos(obj_type: str):
    """
      Getter for x and y value for placing the given object type.

      :param obj_type: type of object, which x and y pose for placing we want
      :return: the tupel of x and y value for placing that object
      """
    x_val = PlacingXPose[obj_type.upper()].value
    y_val = PlacingYPose[obj_type.upper()].value
    return x_val, y_val


def navigate_to(location_name: str, y: Optional[float] = None):
    """
    Navigates to the couch table or to the dishwasher on different sides.

    :param y: y pose to navigate to the couch table for picking up objects
    :param location_name: defines the name of the location to move to
    """
    # Todo popcorn table umbenennen couch table
    if location_name == "couch table" and y is not None:
        NavigateAction(target_locations=[Pose([3.9, y, 0], [0, 0, 0, 1])]).resolve().perform()
    elif location_name == "dishwasher_left":
        NavigateAction(target_locations=[Pose([3.5, -1.32, 0], [0, 0, 1, 0])]).resolve().perform()
    elif location_name == "dishwasher_right":
        NavigateAction(target_locations=[Pose([1.95, -1.48, 0], [0, 0, 0, 1])]).resolve().perform()
    elif location_name == "dishwasher":
        NavigateAction(target_locations=[Pose([2.55, -0.9, 0], [0, 0, -1, 1])]).resolve().perform()
    else:
        rospy.logerr("Failure. Y-Value must be set for the navigateAction to the couch table")


def navigate_and_detect():
    """
    Navigates to the couch table and perceives.

    :return: tupel of State and dictionary of found objects in the FOV
    """
    TalkingMotion("Navigating").resolve().perform()
    # navigate_to(False, 1.8, "popcorn table")
    navigate_to("couch table", 2.35)  # 1.6

    MoveTorsoAction([0.1]).resolve().perform()

    # couch table
    LookAtAction(targets=[Pose([5.0, 2.45, 0.1])]).resolve().perform()  # 0.18 vanessa
    TalkingMotion("Perceiving").resolve().perform()
    try:
        object_desig = DetectAction(technique='region', state="couch_table").resolve().perform()
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        object_desig = {}
    return object_desig


# Main interaction sequence with real robot
def failure_handling1(sorted_obj: list):
    """
    Part 1 of the failure handling consists of perceiving a second time and pick up and placing the seen objects.

    :param sorted_obj: list of seen objects.
    :return: list of seen objects in the second round. Empty list when nothing perceived or all objects already found.
    """
    global LEN_WISHED_SORTED_OBJ_LIST, wished_sorted_obj_list, move_to_the_middle_pose
    new_sorted_obj = []
    print(f"length of sorted obj: {len(sorted_obj)}")

    # if not all needed objects found, the robot will perceive, pick up and
    # place new-found objects again.
    if len(sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
        print("first Check")
        for value in sorted_obj:
            # remove objects that were seen and transported so far except the silverware
            if value.type in wished_sorted_obj_list:
                wished_sorted_obj_list.remove(value.type)
        print(f"übrige List: {wished_sorted_obj_list}")
        #TalkingMotion("Navigating").resolve().perform()

        # NavigateAction(target_locations=[Pose(move_to_the_middle_pose, [0, 0, 1, 1])]).resolve().perform()
        new_object_desig = navigate_and_detect()
        new_sorted_obj = sort_objects(robot, new_object_desig, wished_sorted_obj_list)
        # pickup_and_place_objects(new_sorted_obj)

    return new_sorted_obj


def failure_handling2(sorted_obj: list, new_sorted_obj: list):
    """
    Part 2 of the failure handling, when object is not seen again, the robot is asking for human support.

    :param sorted_obj: list of already seen and transported objects
    :param new_sorted_obj: list of objects that were seen in the first part of the failure handling
    """
    global LEN_WISHED_SORTED_OBJ_LIST, wished_sorted_obj_list
    # failure handling part 2
    final_sorted_obj = sorted_obj + new_sorted_obj
    if len(final_sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
        #TalkingMotion("Navigating").resolve().perform()
        if robot.get_pose().pose.position.y < 0:
             NavigateAction(target_locations=[Pose(move_to_the_middle_pose, [0, 0, 1, 1])]).resolve().perform()
        navigate_to("couch table", 2.45)
        print("second Check")

        for value in final_sorted_obj:
            # remove all objects that were seen and transported so far
            if value.type in wished_sorted_obj_list:
                wished_sorted_obj_list.remove(value.type)

    print(f"übrige List: {wished_sorted_obj_list}")


        # for val in range(len(wished_sorted_obj_list)):
        #     grasp = "front"
        #
        #     print(f"next object is: {wished_sorted_obj_list[val]}")
        #     TalkingMotion(f"Can you please give me the {wished_sorted_obj_list[val]} "
        #                   f"on the table?").resolve().perform()
        #     time.sleep(4)
        #     TalkingMotion("Grabing.").resolve().perform()
        #     MoveGripperMotion("close", "left").resolve().perform()
        #
        #     ParkArmsAction([Arms.LEFT]).resolve().perform()
        #     #TalkingMotion("Navigating").resolve().perform()
        #     # navigate_to(True, 1.8, "long table")
        #     # navigate_to(False, y_pos, "long table")
        #     x_y_pos = get_pos(wished_sorted_obj_list[val])
        #     x_pos = x_y_pos[0]
        #     y_pos = x_y_pos[1]
        #
        #     # navigate_to(2.0, -0.2, "dishwasher")
        #     NavigateAction(target_locations=[Pose(move_to_the_middle_pose, [0, 0, 0, 1])]).resolve().perform()
        #     NavigateAction(
        #         target_locations=[Pose(move_to_the_middle_left_side_dishwasher, [0, 0, -1, 1])]).resolve().perform()
        #
        #     if wished_sorted_obj_list[val] == "Metalplate" or wished_sorted_obj_list[val] == "Metalbowl":
        #         MoveJointsMotion(["arm_roll_joint"], [-1.5]).resolve().perform()
        #     if x_pos >= 2.75:
        #         navigate_to("dishwasher_left")
        #     else:
        #         navigate_to("dishwasher_right")
        #
        #     TalkingMotion("Placing").resolve().perform()
        #     # obj = ObjectDesignatorDescription([wished_sorted_obj_list[val]],[wished_sorted_obj_list[val]])
        #     # obj2 = Object(name=wished_sorted_obj_list[val], type=wished_sorted_obj_list[val])
        #     # object_desig = BelieveObject(obj2)
        #     # #todo was mit placingAction machen, plate wird sonst falsch platziert, aber wie erzeuge ich mir eine eigene description ohne perception
        #     if wished_sorted_obj_list[val] == "Metalplate":
        #         #PlaceGivenObjAction([wished_sorted_obj_list[val]], ["left"],
        #                             #[Pose([x_pos, y_pos, 0.3])], [grasp], False).resolve().perform()
        #         TalkingMotion("Please take the plate and place it in the dishwasher").resolve().perform()
        #         time.sleep(2)
        #         TalkingMotion("Droping object now").resolve().perform()
        #         MoveGripperMotion("open", "left").resolve().perform()
        #     else:
        #         PlaceGivenObjAction([wished_sorted_obj_list[val]], ["left"],
        #                 [Pose([x_pos, y_pos, 0.3])],[grasp]).resolve().perform()
        #     ParkArmsAction([Arms.LEFT]).resolve().perform()
        #     # navigates back if a next object exists
        #
        #     if val + 1 < len(wished_sorted_obj_list):
        #         #TalkingMotion("Navigating").resolve().perform()
        #         # navigate_to(True, 2, "popcorn table")
        #         # navigate_to(False, 1.8, "popcorn table")
        #         #navigate_to(3.9, 2, "popcorn table")
        #         NavigateAction(target_locations=[Pose(move_to_the_middle_pose, [0, 0, 1, 1])]).resolve().perform()
        #         navigate_to("couch table", 2.45)

def test_knowledge_interface():
    location = get_table_test_pose("couch_table")
    print(f"table_pose: {location}")
    print(f"table_y: {location.posestamped.pose.orientation.y}")
    handle = get_handle_pos(handle_name)
    print(f"handle_pose: {handle}")

with ((real_robot)):
    rospy.loginfo("Starting demo")
    TalkingMotion("Starting demo").resolve().perform()

    # navigate_to("dishwasher")
    #
    # MoveJointsMotion(["wrist_roll_joint"], [-1.5]).resolve().perform()
    #
    # OpenDishwasherAction(handle_name, door_name, 0.6, 1.3, ["left"]).resolve().perform()
    #
    # TalkingMotion("Please pull out the lower rack").resolve().perform()

    ParkArmsAction([Arms.LEFT]).resolve().perform()
    MoveGripperMotion("open", "left").resolve().perform()

    # detect objects
    #NavigateAction(target_locations=[Pose(move_to_the_middle_pose, [0, 0, 1, 1])]).resolve().perform()
    object_desig = navigate_and_detect()

    # sort objects based on distance and which we like to keep
    sorted_obj = sort_objects(robot, object_desig, wished_sorted_obj_list)
    print(f"Bowl z pos before: {sorted_obj[0].pose.position.z}")
    if sorted_obj[0].type == "Metalbowl":
        sorted_obj[0].pose.position.z -= 0.02
    print(f"Bowl z pos after: {sorted_obj[0].pose.position.z}")

    #picking up and placing objects
    #pickup_and_place_objects(sorted_obj)

    new_obj_desig = failure_handling1(sorted_obj)
    failure_handling2(sorted_obj, new_obj_desig)
    test_knowledge_interface()
    rospy.loginfo("Done!")
    TalkingMotion("Done").resolve().perform()




# def navigate_to(turn_around, y, table_name):
#     """
#     Navigates to the popcorntable or to the table on the other side.
#
#     :param x: x pose to navigate to
#     :param y: y pose to navigate to
#     :param orientation: defines the orientation of the robot respectively the name of the table to move to
#     """
# table = get_table_pose(table_name)

#       print(f"table_pose: {table}")
# if turn_around:
#  NavigateAction(target_locations=[
#                   Pose([2.0, y,
#                         table.posestamped.pose.position.z], [table.posestamped.pose.orientation.x,
#                         table.posestamped.pose.orientation.y, table.posestamped.pose.orientation.z,
#                                                             table.posestamped.pose.orientation.w])]).resolve().perform()
# else:
#     NavigateAction(target_locations=[table.posestamped.pose.position.x, y,
#      #                         table.posestamped.pose.position.z], [table.posestamped.pose.orientation.x,
#      #                         table.posestamped.pose.orientation.y, table.posestamped.pose.orientation.z,
#      #                                                             table.posestamped.pose.orientation.w])]).resolve().perform()]).resolve().perform()


# lt = LocalTransformer()
#     tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
#     tf_listener = tf2_ros.TransformListener(tf_buffer)
#     transform = tf_buffer.lookup_transform("sink_area_dish_washer_main", "base_link", 0, rospy.Duration(1.0))
#     base_link = PoseStamped()
#     base_link.header.frame_id = "base_link"
#     pose_transformed = tf2_geometry_msgs.do_transform_pose(base_link ,transform)
#     rTb = Pose(frame= "base_link")
#     rTd = lt.transform_pose(rTb, "iai_kitchen/sink_area_dish_washer_main")