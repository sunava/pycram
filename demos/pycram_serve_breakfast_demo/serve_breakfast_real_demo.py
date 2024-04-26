from enum import Enum
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_serve_breakfast_demo.utils.misc import *
from demos.pycram_hsrb_real_test_demos.utils.misc import try_pick_up

# from pycram.external_interfaces.knowrob import get_table_pose
# TODO: im Allgemeinen müssen die Werte beim Navigieren und detecten angepasst werden

# list of cutlery objects
CUTLERY = ["Spoon", "Fork", "Knife", "Plasticknife"]

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalbowl", "Cerealbox", "Milkpack", "Spoon"]

# length of wished list for failure handling
LEN_WISHED_SORTED_OBJ_LIST = len(wished_sorted_obj_list)

# x pose of the end of the popcorn table
# TODO: Wert anpassen
table_pose = 1.04

# the bowl to pour in
bowl = None

# if we already have places to place or not
place_pose_exists = False

# if bowl perceived or not
bowl_exists = False

# free places for placing
place_pose = None

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")
giskardpy.init_giskard_interface()

robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "couch-kitchenmarch1.urdf")

# TODO: bestimme die richtige Objectorentation
# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 90)

giskardpy.sync_worlds()


class PlacingZPose(Enum):
    """
    Differentiate the z pose for placing
    """
    CUTLERY = 0.8
    SPOON = 0.8
    FORK = 0.8
    PLASTICKNIFE = 0.8
    KNIFE = 0.8
    Metalbowl = 0.84
    MILKPACK = 0.88
    METALMUG = 0.8
    CEREALBOX = 0.9
    METALPLATE = 0.9  # table height plus radius of plate


def try_detect(pose: Pose):
    """
    Navigates to the popcorn table and perceives.

    :return: tupel of State and dictionary of found objects in the FOV
    """
    LookAtAction(targets=[pose]).resolve().perform()
    TalkingMotion("Perceiving").resolve().perform()
    try:
        object_desig = DetectAction(technique='all').resolve().perform()
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        object_desig = {}
    return object_desig


def navigate_to(x: float, y: float, table_name: str):
    """
    Navigates to the popcorn table or to the table on the other side.

    :param x: x pose to navigate to
    :param y: y pose to navigate to
    :param table_name: defines the name of the table to move to
    """
    if table_name == "shelf":
        # TODO: Orientierung mus noch angepasst werden
        NavigateAction(target_locations=[Pose([x, y, 0], [0, 0, -1, 1])]).resolve().perform()
    elif table_name == "long table":
        NavigateAction(target_locations=[Pose([x, y, 0], [0, 0, 0, 1])]).resolve().perform()


def pickup_and_place_objects(sorted_obj: list):
    """
    For picking up and placing the objects in the given object designator list.

    :param sorted_obj: the distance sorted list of seen object designators.
    """
    global table_pose, CUTLERY, bowl, place_pose_exists, bowl_exists, place_pose

    for value in range(len(sorted_obj)):
        # define grasping pose
        grasp = "front"
        if sorted_obj[value].type in CUTLERY:
            sorted_obj[value].type = "Cutlery"

        if sorted_obj[value].type in ["Metalbowl", "Cutlery"]:
            grasp = "top"

        # change object x pose if the grasping pose is too far in the table
        # TODO: Wert von table_pose anpassen
        if sorted_obj[value].type == "Cutlery" and sorted_obj[value].pose.position.x < table_pose - 0.125:
            sorted_obj[value].pose.position.x += 0.1

        TalkingMotion("Picking up with: " + grasp).resolve().perform()
        try_pick_up(robot, sorted_obj[value], grasp)

        place_objects(True, sorted_obj, value, grasp)
        #
        # # placing the object
        # ParkArmsAction([Arms.LEFT]).resolve().perform()
        # TalkingMotion("Navigating").resolve().perform()
        # # MoveGripperMotion("open", "left").resolve().perform()
        # # TODO: Werte anpassen
        # navigate_to(1.8, 1.8, "long table")
        #
        # if not place_pose_exists or not bowl_exists:
        #     object_desig = try_detect(Pose([4.2, 2, 2.1], [0, 0, 1, 0]))
        #     # Todo: freier Platz zum Platzieren erkennen (vielleicht eine Funktion dazu schreiben)
        #     if not place_pose_exists:
        #         place_pose = Pose([0, 0, 0], [0, 0, 1, 0])  # detect free space to place
        #         place_pose_exists = True
        #     if sorted_obj[value].type in ["Cerealbox", "Milkpack", "Cutlery"]:
        #         if not bowl_exists:
        #             bowl = get_bowl(object_desig)
        #             bowl_exists = True
        #
        # if sorted_obj[value].type in ["Cerealbox", "Milkpack", "Cutlery"]:
        #     if bowl is None:
        #         # TODO: Failure handling
        #
        #         # ein Schritt nach rechts, links oder hinten :)
        #         navigate_to(robot.get_pose().pose.position.x, robot.get_pose().pose.position.y - 0.1, "long table")
        #         new_object_deign = try_detect(Pose([4.2, 2, 2.1], [0, 0, 1, 0]))
        #         bowl = get_bowl(new_object_deign)
        #
        #     if bowl is not None:
        #         if sorted_obj[value].type in ["Cerealbox", "Milkpack"]:
        #             # TODO: Werte anpassen
        #             navigate_to(4, bowl.pose.position.y, "long table")
        #             if robot.get_pose().pose.position.y <= bowl.pose.position.y:
        #                 PouringAction([bowl.pose], ["left"], ["left"], [-1.6]).resolve().perform()
        #             else:
        #                 PouringAction([bowl.pose], ["left"], ["right"], [1.6]).resolve().perform()
        #         else:
        #             place_pose.pose.position.x = bowl.pose.position.x
        #             place_pose.pose.position.y = bowl.pose.position.y - 0.2
        #             place_pose.pose.position.x = bowl.pose.position.z
        #
        # # TODO: Werte anpassen
        # navigate_to(4, place_pose.pose.position.y, "long table")
        # TalkingMotion("Placing").resolve().perform()
        # z = get_z(sorted_obj[value].type)
        # PlaceAction(sorted_obj[value], ["left"], [grasp],
        #             [Pose([place_pose.pose.position.x, place_pose.pose.position.y, z])]).resolve().perform()
        #
        # ParkArmsAction([Arms.LEFT]).resolve().perform()
        #
        # # navigates back if a next object exists
        # if value + 1 < len(sorted_obj):
        #     TalkingMotion("Navigating").resolve().perform()
        #     # TODO: Werte anpassen
        #     navigate_to(sorted_obj[value + 1].pose.position.x, 1.87, "shelf")


def place_objects(first_placing, objects_list, index, grasp):
    global bowl, place_pose_exists, bowl_exists, place_pose

    if first_placing:
        object_type = objects_list[index].type
    else:
        object_type = objects_list[index]

    # placing the object
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    TalkingMotion("Navigating").resolve().perform()
    # MoveGripperMotion("open", "left").resolve().perform()
    # TODO: Werte anpassen
    navigate_to(1.8, 1.8, "long table")

    # TODO: vielleicht nur nach bowl fragen, da wir sowieso nach dem detecten von bowl nicht mehr detecten und
    #       davor einmal detecten sollen
    # TODO: vielleicht place_pose ändern
    if not place_pose_exists or not bowl_exists:
        object_desig = try_detect(Pose([4.2, 2, 2.1], [0, 0, 1, 0]))
        # Todo: freier Platz zum Platzieren erkennen (vielleicht eine Funktion dazu schreiben)
        if not place_pose_exists:
            place_pose = Pose([0, 0, 0], [0, 0, 1, 0])  # detect free space to place
            place_pose_exists = True
        if object_type in ["Cerealbox", "Milkpack", "Cutlery"]:
            if not bowl_exists:
                bowl = get_bowl(object_desig)
                bowl_exists = True

    if object_type in ["Cerealbox", "Milkpack", "Cutlery"]:
        if bowl is None:
            # TODO: Failure handling

            # ein Schritt nach rechts, links oder hinten :)
            navigate_to(robot.get_pose().pose.position.x, robot.get_pose().pose.position.y - 0.1, "long table")
            new_object_deign = try_detect(Pose([4.2, 2, 2.1], [0, 0, 1, 0]))
            bowl = get_bowl(new_object_deign)
            ########################################
            # oder die letzte place_pose nehmen (die Pose vom bowl)
            #TODO: richtig anpassen
            if bowl is None:
                bowl = place_pose

        if object_type in ["Cerealbox", "Milkpack"]:
            # TODO: Werte anpassen
            navigate_to(4, bowl.pose.position.y, "long table")
            if robot.get_pose().pose.position.y <= bowl.pose.position.y:
                PouringAction([bowl.pose], ["left"], ["left"], [-1.6]).resolve().perform()
            else:
                PouringAction([bowl.pose], ["left"], ["right"], [1.6]).resolve().perform()
        else:
            place_pose.pose.position.x = bowl.pose.position.x
            place_pose.pose.position.y = bowl.pose.position.y - 0.2
            place_pose.pose.position.x = bowl.pose.position.z

    # TODO: Werte anpassen
    navigate_to(4, place_pose.pose.position.y, "long table")
    TalkingMotion("Placing").resolve().perform()
    z = get_z(object_type)
    if first_placing:
        PlaceAction(objects_list[index], ["left"], [grasp],
                    [Pose([place_pose.pose.position.x, place_pose.pose.position.y, z])]).resolve().perform()
    else:
        PlaceGivenObjAction([objects_list[index]], ["left"],
                            [Pose([place_pose.pose.position.x, place_pose.pose.position.y, z])],
                            [grasp]).resolve().perform()

    ParkArmsAction([Arms.LEFT]).resolve().perform()

    # navigates back if a next object exists
    if value + 1 < len(objects_list):
        TalkingMotion("Navigating").resolve().perform()
        # TODO: Werte anpassen
        navigate_to(sorted_obj[index + 1].pose.position.x, 1.87, "shelf")


def get_z(obj_type: str):
    """
    Getter for z value for placing the given object type.

    :param obj_type: type of object, which z pose we want
    :return: the int z value for placing that object
    """
    return PlacingZPose[obj_type.upper()].value


with ((real_robot)):
    rospy.loginfo("Starting demo")
    TalkingMotion("Starting demo").resolve().perform()
    # TODO: vielleicht entfernen in der finalen Demo
    # MoveGripperMotion(motion="open", gripper="left").resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    TalkingMotion("Navigating").resolve().perform()

    # navigate to door
    # TODO: koordinaten bestimmen
    NavigateAction(target_locations=[Pose([0, 0, 0], [0, 0, 0, 1])]).resolve().perform()
    # navigate to shelf
    # TODO: Orientierung mus noch angepasst werden
    navigate_to(2.02, 1.87, "shelf")
    #NavigateAction(target_locations=[Pose([2.02, 1.87, 0], [0, 1, 0, 0])]).resolve().perform()
    # TODO: Pose muss noch angepasst werden
    obj_desig = try_detect(Pose([0.8, 1.8, 0.21], object_orientation))
    # TODO: test sorted_obj
    sorted_obj = sort_objects(obj_desig, wished_sorted_obj_list)
    pickup_and_place_objects(sorted_obj)

    # failure handling part 1
    new_sorted_obj = []
    print(f"length of sorted obj: {len(sorted_obj)}")

    # if not all needed objects found, the robot will perceive and pick up and
    # place new-found objects again.
    if len(sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
        print("first Check")
        for value in sorted_obj:
            # remove objects that were seen and transported so far except the silverware
            if value.type in wished_sorted_obj_list:
                wished_sorted_obj_list.remove(value.type)

        TalkingMotion("Navigating").resolve().perform()
        navigate_to(1.6, 1.8, "shelf")
        new_object_desig = try_detect(Pose([0.8, 1.8, 0.21], object_orientation))
        new_sorted_obj = sort_objects(new_object_desig, wished_sorted_obj_list)
        pickup_and_place_objects(new_sorted_obj)

        # failure handling part 2
        final_sorted_obj = sorted_obj + new_sorted_obj
        if len(final_sorted_obj) < LEN_WISHED_SORTED_OBJ_LIST:
            navigate_to(1.6, 1.8, "shelf")
            print("second Check")

            for value in final_sorted_obj:
                # remove all objects that were seen and transported so far
                if value.type in wished_sorted_obj_list:
                    wished_sorted_obj_list.remove(value.type)
                # if the type is cutlery, the real type is not easily reproducible.
                # remove one cutlery object of the list with the highest chance that it was found and transported.
                if value.type == "Cutlery":
                    if "Fork" in wished_sorted_obj_list:
                        print("deleted fork")
                        wished_sorted_obj_list.remove("Fork")
                    elif "Spoon" in wished_sorted_obj_list:
                        print("deleted spoon")
                        wished_sorted_obj_list.remove("Spoon")
                    elif "Plasticknife" in wished_sorted_obj_list:
                        print("deleted knife")
                        wished_sorted_obj_list.remove("Plasticknife")

            for val in range(len(wished_sorted_obj_list)):
                grasp = "front"
                if wished_sorted_obj_list[val] in (["Metalbowl"] + CUTLERY):
                    grasp = "top"

                print(f"next object is: {wished_sorted_obj_list[val]}")
                TalkingMotion(f"Can you please give me the {wished_sorted_obj_list[val]} "
                              f"on the table?").resolve().perform()
                time.sleep(4)
                MoveGripperMotion("close", "left").resolve().perform()

                place_objects(False, wished_sorted_obj_list, val, grasp)


    rospy.loginfo("Done!")
    TalkingMotion("Done").resolve().perform()

########################### PSEUDO-CODE ################################
# if door open then enter room
# navigate to shelf
# perceive
# sort objects in a list like this (bowl , cereal, milk, spoon)
# loop: pickup object
####### navigate to table
####### if object milk or cereal or spoon -> perceive bowl
#######  if milk or cereal -> pour into bowl
#######   if spoon place object right of bowl, ELSE: place on free space
#######
