from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.designators.motion_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import real_robot
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
import pycram.external_interfaces.giskard as giskardpy
from pycram.helper import sort_objects

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
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "couch-kitchen.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)

# Wished objects for the Demo
wished_sorted_obj_list = ["Metalplate", "Bowl", "Metalmug", "Fork", "Cerealbox"]

# list of cutlery objects
cutlery = ["Spoon", "Fork", "Knife", "Plasticknife"]

# y pose for placing the object
y_pos = 1.66

# x pose of the end of the popcorntable
table_pose = 1.04


def try_pick_up(robot, obj, grasps):
    """
    Picking up any object with failure handling.

    :param robot: the robot
    :param obj: the object that should be picked up
    :param grasps: how to pick up the object
    """
    try:
        PickUpAction(obj, ["left"], [grasps]).resolve().perform()
    except (EnvironmentUnreachable, GripperClosedCompletely):
        print("try pick up again")
        TalkingMotion("Try pick up again")
        # after failed attempt to pick up the object, the robot moves 30cm back on x pose
        NavigateAction(
            [Pose([robot.get_pose().position.x - 0.3, robot.get_pose().position.y, robot.get_pose().position.z],
                  robot.get_pose().orientation)]).resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        # try to detect the object again
        if EnvironmentUnreachable:
            object_desig = DetectAction(technique='default').resolve().perform()
            # TODO nur wenn key (name des vorherigen objektes) in object_desig enthalten ist
            # TODO umschreiben, geht so nicht mehr, da das dict in einem tupel ist
            new_object = object_desig[1][obj.name]
        # when the robot just grabed next to the object
        # TODO wieso unterscheiden wir hier überhaupt, wenn er daneben gegriffen hat, hat er das objekt
        # TODO wahrscheinlich verschoben und sollte auch nochmal perceiven
        else:
            new_object = obj
        # second try to pick up the object
        try:
            PickUpAction(new_object, ["left"], [grasps]).resolve().perform()
        # ask for human interaction if it fails a second time
        except:
            TalkingMotion(f"Can you pleas give me the {obj.name} on the table?")
            MoveGripperMotion("open", "left").resolve().perform()
            time.sleep(4)
            MoveGripperMotion("close", "left").resolve().perform()


def pickUp_and_place_objects(robot, sorted_obj):
    """
    For picking up and placing the objects in the given object designator list.

    :param robot: the robot
    :param sorted_obj: the distance sorted list of seen object designators.
    """
    global y_pos, table_pose, cutlery

    for value in range(len(sorted_obj)):
        print("first navigation")

        # define grasping pose
        grasp = "front"
        if sorted_obj[value].type in cutlery:
            sorted_obj[value].type = "Cutlery"

        if sorted_obj[value].type in ["Bowl", "Cutlery"]:
            grasp = "top"

        if sorted_obj[value].type == "Metalplate":
            TalkingMotion("Can you please give me the plate on the table.").resolve().perform()
            MoveGripperMotion("open", "left").resolve().perform()
            time.sleep(3)
            MoveGripperMotion("close", "left").resolve().perform()
            print("picked up plate")
        else:
            # change object x pose if the grasping pose is to close to the handle ending
            if sorted_obj[value].type == "Cutlery" and sorted_obj[value].pose.position.x + 0.05 >= table_pose:
                print("adjusted x")
                sorted_obj[value].pose.position.x -= 0.1
            TalkingMotion("Picking up with: " + grasp).resolve().perform()
            try_pick_up(sorted_obj[value], grasp)

        # placing the object
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        TalkingMotion("Navigating").resolve().perform()
        navigate_to(1.8, 1.8, "table")
        navigate_to(4.1, y_pos, "table")
        TalkingMotion("Placing").resolve().perform()

        # differentiate the z pose for placing
        if sorted_obj[value].type == "Cutlery":
            z = 0.8
        elif sorted_obj[value].type == "Bowl":
            z = 0.84
        elif sorted_obj[value].type == "Metalmug":
            z = 0.8
        elif sorted_obj[value].type == "Milkpack":
            z = 0.88
        elif sorted_obj[value].type == "Cerealbox":
            z = 0.9
        if sorted_obj[value].type == "Metalplate":
            # with special defined placing movement for the plate
            PlaceGivenObjAction([sorted_obj[value].type], ["left"], [Pose([4.86, y_pos, 0])],
                                ["front"]).resolve().perform()
        else:
            PlaceAction(sorted_obj[value], ["left"], [grasp], [Pose([4.9, y_pos, z])]).resolve().perform()

        ParkArmsAction([Arms.LEFT]).resolve().perform()
        TalkingMotion("Navigating").resolve().perform()
        navigate_to(3.9, 2, "popcorntable")

        # adjust y_pos for the next placing round
        if sorted_obj[value].type == "Metalplate":
            y_pos += 0.3
        else:
            y_pos += 0.16

        # navigates back if a next object exists
        if value + 1 < len(sorted_obj):
            navigate_to(1.6, sorted_obj[value + 1].pose.position.y, "popcorntable")


def navigate_to(x, y, orientation):
    """
    Navigates to the popcorntable or to the table on the other side.

    :param x: x pose to navigate to
    :param y: y pose to navigate to
    :param orientation: defines the orientation of the robot respectively the name of the table to move to
    """
    if orientation == "popcorntable":
        NavigateAction(target_locations=[Pose([x, y, 0], [0, 0, 1, 0])]).resolve().perform()
    elif orientation == "table":
        NavigateAction(target_locations=[Pose([x, y, 0], [0, 0, 0, 1])]).resolve().perform()


def navigate_and_detect():
    """
    Navigates to the popcorntable and perceives.

    :return: tupel of State and dictionary of found objects in the FOV
    """
    TalkingMotion("Navigating").resolve().perform()
    navigate_to(1.6, 1.8, "popcorntable")

    # popcorntable
    # todo gucken ob ein aufruf genügt
    LookAtAction(targets=[Pose([0.8, 1.8, 0.21], object_orientation)]).resolve().perform()
    LookAtAction(targets=[Pose([0.8, 1.8, 0.21], object_orientation)]).resolve().perform()
    TalkingMotion("Perceiving").resolve().perform()
    try:
        object_desig = DetectAction(technique='all').resolve().perform()
    except PerceptionObjectNotFound:
        object_desig = {}
    return object_desig


# Main interaction sequence with real robot
with ((real_robot)):
    rospy.loginfo("Starting demo")
    TalkingMotion("Starting demo").resolve().perform()
    MoveGripperMotion(motion="open", gripper="left").resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    # detect objects
    object_desig = navigate_and_detect()

    # sort objects based on distance and which we like to keep
    sorted_obj = sort_objects(robot, object_desig, wished_sorted_obj_list)

    # picking up and placing objects
    pickUp_and_place_objects(robot, sorted_obj)

    # failure handling part 1
    new_sorted_obj = []
    print(f"length of sorted obj: {len(sorted_obj)}")

    # if not all needed objects found, the robot will perceive and pick up and
    # place new-found objects again.
    if len(sorted_obj) < len(wished_sorted_obj_list):
        print("first Check")
        new_object_desig = navigate_and_detect()
        new_sorted_obj = sort_objects(robot, new_object_desig, wished_sorted_obj_list)
        pickUp_and_place_objects(robot, new_sorted_obj)

    final_sorted_obj = sorted_obj + new_sorted_obj

    # todo maybe delete this if not needed
    for obj in sorted_obj:
        print(f"sorted obj: {obj.type}")
    for obj in new_sorted_obj:
        print(f"new sorted obj: {obj.type}")
    for obj in final_sorted_obj:
        print(f"final sorted obj: {obj.type}")

    # failure handling part 2
    if len(final_sorted_obj) < len(wished_sorted_obj_list):
        navigate_to(1.6, 1.8, "popcorntable")
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
            if wished_sorted_obj_list[val] in ["Bowl", "Spoon", "Fork", "Knife", "Plasticknife"]:
                grasp = "top"

            print(f"in here with object {wished_sorted_obj_list[val]}")
            TalkingMotion(f"Can you please give me the {wished_sorted_obj_list[val]} on the table?").resolve().perform()
            time.sleep(4)
            MoveGripperMotion("close", "left").resolve().perform()

            # if the Metalplate was first not found and is now placed somewhere in the
            # middle of the table, add space to the right object.
            if wished_sorted_obj_list[val] == "Metalplate":
                y_pos += 0.14

            if wished_sorted_obj_list[val] in ["Spoon", "Fork", "Knife", "Plasticknife"]:
                z = 0.8
            elif wished_sorted_obj_list[val] == "Bowl":
                z = 0.84
            elif wished_sorted_obj_list[val] == "Metalmug":
                z = 0.8
            elif wished_sorted_obj_list[val] == "Milkpack":
                z = 0.88
            elif wished_sorted_obj_list[val] == "Cerealbox":
                z = 0.9
            elif wished_sorted_obj_list[val] == "Metalplate":
                z = 0

            ParkArmsAction([Arms.LEFT]).resolve().perform()
            TalkingMotion("Navigating").resolve().perform()
            navigate_to(1.8, 1.8, "table")
            navigate_to(4.1, y_pos, "table")
            TalkingMotion("Placing").resolve().perform()

            PlaceGivenObjAction([wished_sorted_obj_list[val]], ["left"], [Pose([4.86, y_pos, z])],
                                [grasp]).resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            TalkingMotion("Navigating").resolve().perform()
            if wished_sorted_obj_list[val] == "Metalplate":
                y_pos += 0.3
            else:
                y_pos += 0.16

            # navigates back if a next object exists
            # todo sollen wir ihn echt zum tisch navigieren lassen, wenn er das objekt vom menschen bekommt?
            if val + 1 < len(wished_sorted_obj_list):
                navigate_to(3.9, 2, "popcorntable")
                navigate_to(1.6, 1.8, "popcorntable")

    rospy.loginfo("Done!")
    TalkingMotion("Done").resolve().perform()
