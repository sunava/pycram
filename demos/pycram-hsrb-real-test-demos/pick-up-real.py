import random
from math import sqrt

import rospy
from tmc_control_msgs.msg import GripperApplyEffortActionGoal
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.designators.motion_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot, semi_real_robot, real_robot
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
import pycram.external_interfaces.giskard as giskardpy
import rospy

from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.designators.motion_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot, semi_real_robot, real_robot
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
import pycram.external_interfaces.giskard as giskardpy

from geometry_msgs.msg import Point
# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Initialize Giskard interface for motion planning

#giskardpy.spawn_kitchen()
# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")
giskardpy.init_giskard_interface()
#giskardpy.removing_of_objects()
#giskardpy.sync_worlds()
robot.set_color([0.5, 0.5, 0.9, 1])
robot_pose = robot.get_pose()

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "couch-kitchen.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)

# Wished objects for the Demo
wished_sorted_obj_list = ["Bowl", "Metalmug", "Fork", "Metalplate", "Cerealbox"]

y_pos = 1.66
table_pose = 1.04


def sort_objects(obj_dict, wished_sorted_obj_list):
    sorted_objects= []
    object_tuples = []
    isPlate = True
    #print(f"gefundene liste: {obj_dict}")
    if len(obj_dict) == 0:
        return sorted_objects
    first, *remaining = obj_dict
    print(f"type: {type(remaining)} and type of obj_dict: {type(obj_dict)}")
    robot_pose = robot.get_pose()
    for dict in remaining:
        for value in dict.values():
            print(f"value type: {type(value)}")
            distance = sqrt(pow((value.pose.position.x - robot_pose.position.x), 2) +
                         pow((value.pose.position.y - robot_pose.position.y), 2) +
                         pow((value.pose.position.z - robot_pose.position.z), 2))

            print(f"object name: {value.name} and distance: {distance}")

            if isPlate or value.type != "Metalplate":
                object_tuples.append((value, distance))
                if value.type == "Metalplate":
                     isPlate = False

    sorted_object_list = sorted(object_tuples, key= lambda distance: distance[1])

    for (object, distance) in sorted_object_list:
        #todo: reminder types are written with small letter and names now with big beginning letter
        if object.type in wished_sorted_obj_list:
            if object.type == "Metalplate":
                sorted_objects.insert(0, object)
            else:
                sorted_objects.append(object)

    test_list = []
    for test_object in sorted_objects:
         test_list.append(test_object.name)
    print(test_list)

    return sorted_objects

def try_pick_up(obj, grasps):
    try:
        PickUpAction(obj, ["left"], [grasps]).resolve().perform()
    except (EnvironmentUnreachable, GripperClosedCompletely):
        print("try pick up again")
        TalkingMotion("Try pick up again")
        NavigateAction([Pose([robot.get_pose().position.x - 0.3, robot.get_pose().position.y, robot.get_pose().position.z],
                             robot.get_pose().orientation)]).resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        if EnvironmentUnreachable:
             object_desig = DetectAction(technique='default').resolve().perform()
             # TODO nur wenn key (name des vorherigen objektes) in object_desig enthalten ist
             new_object = object_desig[obj.name]
        else:
             new_object = obj
        try:
            PickUpAction(new_object, ["left"], [grasps]).resolve().perform()

        except:
            TalkingMotion(f"Can you pleas give me the {obj.name} object on the table? Thanks")
            TalkingMotion(f"Please push down my hand, when I can grab the {obj.name}.")

def pickUp_and_place_objects(sorted_obj):
    global y_pos, table_pose
    #for value in sorted_obj:
    for value in range(len(sorted_obj)):
        print("first navigation")
        cutlery = ["Spoon", "Fork", "Knife", "Plasticknife"]
        grasp = "front"
        if (sorted_obj[value].type in cutlery):
            sorted_obj[value].type = "Cutlery"

        if sorted_obj[value].type in ["Bowl", "Cutlery"]:
            grasp = "top"

        if sorted_obj[value].type == "Metalplate":
            MoveGripperMotion("open", "left").resolve().perform()
            TalkingMotion("Can you please give me the plate on the table.").resolve().perform()
            # TalkingMotion("Please push down my hand, when I can grab the plate.").resolve().perform()
            print("picked up plate")
            time.sleep(3)
            MoveGripperMotion("close", "left").resolve().perform()
        else:
            if sorted_obj[value].type == "Cutlery" and sorted_obj[value].pose.position.x + 0.05 >= table_pose:
                print("adjusted x")
                sorted_obj[value].pose.position.x -= 0.1
            TalkingMotion("Picking Up with: " + grasp).resolve().perform()
            try_pick_up(sorted_obj[value], grasp)

        ParkArmsAction([Arms.LEFT]).resolve().perform()
        TalkingMotion("Navigating").resolve().perform()
        NavigateAction(target_locations=[Pose([1.8, 1.8, 0], [0, 0, 0, 1])]).resolve().perform()  # 1.6 für x
        NavigateAction(target_locations=[Pose([4.1, y_pos, 0], [0, 0, 0, 1])]).resolve().perform()
        TalkingMotion("Placing").resolve().perform()
        # Todo: Objekte in z unterscheiden
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
            PlaceGivenObjAction([sorted_obj[value].type], ["left"], [Pose([4.86, y_pos, 0])], ["front"]).resolve().perform()
        else:
            PlaceAction(sorted_obj[value], ["left"], [grasp], [Pose([4.9, y_pos, z])]).resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        TalkingMotion("Navigating").resolve().perform()
        NavigateAction(target_locations=[Pose([3.9, 2, 0], [0, 0, 1, 0])]).resolve().perform()  # 4.1 für x
        if sorted_obj[value].type == "Metalplate":
            y_pos += 0.3
        else:
            y_pos += 0.16
        if value + 1 < len(sorted_obj):
            NavigateAction(target_locations=[Pose([1.6, sorted_obj[value + 1].pose.position.y, 0], [0, 0, 1, 0])]).resolve().perform()



def navigate_and_detect():
    TalkingMotion("Navigating").resolve().perform()
    NavigateAction(target_locations=[Pose([1.6,1.8, 0], [0, 0, 1, 0])]).resolve().perform()
    # popcorntable
    LookAtAction(targets=[Pose([0.8, 1.8, 0.21], object_orientation)]).resolve().perform()
    LookAtAction(targets=[Pose([0.8, 1.8, 0.21], object_orientation)]).resolve().perform()
    TalkingMotion("Perceiving").resolve().perform()
    try:
         object_desig = DetectAction(technique='all').resolve().perform()
    except PerceptionObjectNotFound:
            object_desig = {}
    return object_desig


# Main interaction sequence with semi-real robot
with ((real_robot)):
    rospy.loginfo("Starting demo")
    TalkingMotion("Starting demo").resolve().perform()
    MoveGripperMotion(motion="open", gripper="left").resolve().perform()
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    object_desig = navigate_and_detect()

    sorted_obj = sort_objects(object_desig, wished_sorted_obj_list)

    pickUp_and_place_objects(sorted_obj)
    new_sorted_obj = []
    print(f"length of sorted obj: {len(sorted_obj)}")
    if len(sorted_obj) < len(wished_sorted_obj_list):
        new_object_desig = navigate_and_detect()
        print("first Check")
        print(new_object_desig)
        new_sorted_obj = sort_objects(new_object_desig, wished_sorted_obj_list)
        pickUp_and_place_objects(new_sorted_obj)


    final_sorted_obj = sorted_obj + new_sorted_obj

    for obj in sorted_obj:
        print(f"sorted obj: {obj.type}")
    for obj in new_sorted_obj:
        print(f"new sorted obj: {obj.type}")
    for obj in final_sorted_obj:
        print(f"final sorted obj: {obj.type}")
    print(f"y_pos: {y_pos}")

    if len(final_sorted_obj) < len(wished_sorted_obj_list):
        NavigateAction(target_locations=[Pose([1.6, 1.8, 0], [0, 0, 1, 0])]).resolve().perform()
        print("second Check")

        for value in final_sorted_obj:
            # TODO: Cutlery noch behandeln und doppelte Objektnamen, wie Fork
                if value.type in wished_sorted_obj_list:
                    wished_sorted_obj_list.remove(value.type)
                if value.type == "Cutlery":
                    if "Fork" in wished_sorted_obj_list:
                        print("deleted fork")
                        wished_sorted_obj_list.remove("Fork")
                    elif "Spoon" in wished_sorted_obj_list:
                        print("deleted spoon")
                        wished_sorted_obj_list.remove("Spoon")

        for name in wished_sorted_obj_list:
            # random_id = random.randint(50,100)
            # print(f"random number: {random_id}")
            # missed_obj = Object(name=name, type=name, id= random_id)
            # #missed_obj_desig = ObjectDesignatorDescription(names=[name]).resolve()
            # missed_obj_desig = BelieveObject(types=[name])
            grasp = "front"
            if name in ["Bowl", "Spoon", "Fork", "Knife", "Plasticknife"]:
                grasp = "top"

            print(f"in here with object {name}")
            TalkingMotion(f"Can you please give me the {name} on the table?").resolve().perform()
            rospy.sleep(1)
            #TalkingMotion(f"Please push down my hand, when I can grab the {name}.")
            time.sleep(3)
            MoveGripperMotion("close", "left").resolve().perform()

            if name == "Metalplate":
                 y_pos += 0.14

            if name in ["Spoon", "Fork", "Knife", "Plasticknife"]:
                z = 0.8
            elif name == "Bowl":
                z = 0.84
            elif name == "Metalmug":
                z = 0.8
            elif name == "Milkpack":
                z = 0.88
            elif name == "Cerealbox":
                z = 0.9
            elif name == "Metalplate":
                z = 0

            ParkArmsAction([Arms.LEFT]).resolve().perform()
            TalkingMotion("Navigating").resolve().perform()
            NavigateAction(target_locations=[Pose([1.8, 1.8, 0], [0, 0, 0, 1])]).resolve().perform()  # 1.6 für x
            NavigateAction(target_locations=[Pose([4.1, y_pos, 0], [0, 0, 0, 1])]).resolve().perform()
            TalkingMotion("Placing").resolve().perform()
            # TODO: PlaceGivenObjAction in ActionDesignator für die anderen Objekte anpassen
            # TODO: Was machen wenn Teller nicht gesehen, direkt y_pos + 0.14, damit Abstand zum letzten Objekt 0.3?

            print(f"y_pos failure handling: {y_pos}")
            PlaceGivenObjAction([name],["left"], [Pose([4.86, y_pos, z])], [grasp]).resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            TalkingMotion("Navigating").resolve().perform()
            NavigateAction(target_locations=[Pose([3.9, 2, 0], [0, 0, 1, 0])]).resolve().perform()  # 4.1 für x
            if name == "Metalplate":
                y_pos += 0.3
            else:
                y_pos += 0.16
            NavigateAction(target_locations=[Pose([1.6, 1.8, 0], [0, 0, 1, 0])]).resolve().perform()

    rospy.loginfo("Done!")
    TalkingMotion("Done").resolve().perform()




