import os
import threading
import time

import rospkg

import utils
from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import WorldMode, LoggerLevel, TaskStatus
from pycram.designators.GAP import *
from pycram.designators.action_designator import *
from pycram.external_interfaces import giskard
from pycram.helper import perform, an
from pycram.perception import detect
from pycram.process_module import real_robot
from pycram.ros_utils.robot_state_updater import WorldStateUpdater
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher, ManualMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld
from pycrap.ontologies import PouringTool
from pycrap.ontologies import Robot, Environment, Bowl
from sound_play.libsoundplay import SoundClient
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from pycram.perception import detect
from pycram.helper import perform, an
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.designators.GAP import *
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import PoseStamped
from pycram.process_module import simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycrap.ontologies import Robot, Apartment, Milk, Cereal, Spoon, Bowl, PouringTool, Apple, Food
from pycram.language import CodePlan, ParallelPlan, MonitorPlan
from pycram.ros_utils.force_torque_sensor import ForceTorqueSensor
from pycram.ros import set_logger_level
from pycram.ros import create_publisher, Duration
from pycram.datastructures.pose import PoseStamped, Vector3Stamped

# set_logger_level(LoggerLevel.DEBUG)
extension = ObjectDescription.get_file_extension()
world = BulletWorld(WorldMode.DIRECT)
robot = Object("pr2", Robot, f"pr2{extension}", pose=PoseStamped.from_list([1, 2, 0]))
apartment = Object("apartment", Apartment, f"apartment{extension}")
robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])
# VizMarkerPublisher()
WorldStateUpdater("/tf", "/joint_states")
from pycram.language import CodePlan, ParallelPlan, RepeatPlan, MonitorPlan


#
# obj_tool_ = Object("jeroen_cup", PouringTool, "jeroen_cup.stl", pose=PoseStamped.from_list([3.4, 2, 1.0], [0, 0, 0, 1]))
# m = ManualMarkerPublisher()


def look_at():
    looking_pose = PoseStamped.from_list([2.3, 2, 0.97])
    with real_robot:
        perform(an(LookAtActionDescription(looking_pose)))


def move_the_base():
    location_pose = PoseStamped.from_list([1.7, 2, 0])
    with real_robot:
        perform(an(NavigateActionDescription(location_pose)))


def move_in_simulation():
    location_pose = PoseStamped.from_list([1.7, 2, 0])
    with simulated_robot:
        perform(an(NavigateActionDescription(location_pose)))


def cutting_simulation():
    tool_pose = PoseStamped.from_list([2.0449586673391935, 1.5384467778416917, 1.229705326966067],
                                      [0.14010099565491793, -0.7025332835765593,
                                       0.15537176280408957, 0.6802046102510538])

    with simulated_robot:
        perform(an(ParkArmsActionDescription([Arms.BOTH])))
        perform(an(MoveTorsoActionDescription([TorsoState.HIGH])))
        move_in_simulation()
        knife = Object("knife", PouringTool, "big-knife.stl", pose=tool_pose, size=[0.2, 0.1, 0.05])
        rotate_q = utils.axis_angle_to_quaternion([0, 0, 1], 180)
        tool_frame = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_tool_frame()
        World.current_world.robot.attach(child_object=obj_tool_, parent_link=tool_frame)
        # bread1_ = Object("bread1", Food, "bread.stl",
        #                  pose=PoseStamped.from_list([2.4, 2, 1.0], [0, 0, -1, -1]),
        #                  color=Color.from_list([1, 1, 0, 1]), size=[0.1, 0.2, 0.1])
        # perform(an(CuttingActionDescription(bread1_, knife, [Arms.RIGHT])))


def spawn_world():
    # das sollte eign innerhalb den sync modules geupdated werden

    giskard.spawn_environment()
    giskard.initial_adding_objects()


def perception():
    with real_robot:
        perform(an(DetectActionDescription(technique=DetectionTechnique.ALL)))


def park():
    with real_robot:
        # perform(an(CarryActionDescription([Arms.BOTH], True, "knife", AxisIdentifier.Z, AxisIdentifier.Z,
        #                                   "torso_lift_link")))

        perform(an(ParkArmsActionDescription([Arms.BOTH])))


def pick_up(arm, object_designator):
    with real_robot:
        manu = ManualMarkerPublisher()

        robot_desig_resolved = BelieveObject(names=[RobotDescription.current_robot_description.name]).resolve()
        # ParkArmsAction(Arms.BOTH).perform()
        pickup_loc = CostmapLocation(target=object_designator,
                                     reachable_for=robot_desig_resolved,
                                     reachable_arm=[arm])
        # Tries to find a pick-up position for the robot that uses the given arm
        pickup_pose = pickup_loc.resolve()
        manu.publish(pickup_pose)
        if not pickup_pose:
            raise ObjectUnfetchable(
                f"Found no pose for the robot to grasp the object: {object_designator} with arm: {arm}")

        NavigateAction(pickup_pose, True).perform()
        PickUpAction(object_designator, pickup_pose.arm,
                     grasp_description=pickup_pose.grasp_description).perform()


def pouring():
    tool_pose = PoseStamped.from_list([2.0449586673391935, 1.5384467778416917, 1.09705326966067], [0, 0, 0, 1])
    obj_tool_.pose = tool_pose
    location_pose = PoseStamped.from_list([1.7, 2, 0])
    looking_pose = PoseStamped.from_list([2.3, 2, 0.97])
    # generic_obj_BO = BelieveObject(names=[obj_target_.name]).resolve()
    tool_BO = BelieveObject(names=[obj_tool_.name]).resolve()

    with real_robot:
        print("parking arms")
        perform(an(ParkArmsActionDescription([Arms.BOTH])))
        print("Navigating")
        perform(an(NavigateActionDescription([location_pose])))
        print("MoveTorso")
        perform(an(MoveTorsoActionDescription([TorsoState.HIGH])))

        # attach tool to robot
        # tool_frame = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_tool_frame()
        # World.current_world.robot.attach(child_object=obj_tool_, parent_link=tool_frame)
        print("Looking")
        perform(an(LookAtActionDescription([looking_pose])))
        print("Detect")
        perceived_objects = detect(BelieveObject(types=[Bowl]))
        # query(PouringActionDescription)
        print("Pouring")
        perform(an(PouringActionDescription(perceived_objects[0], tool_BO, [Arms.RIGHT], None, angle=90)))
        print("ParkArms")
        perform(an(ParkArmsActionDescription([Arms.BOTH])))


def cutting():
    # Object("apple", Apple, stl_path, pose=PoseStamped.from_list([2.5, 2, 1.0],
    #                                                             [0, 0, 0, 1]),
    #        color=Color.from_list([1, 0, 0, 0, 1]))
    Object("bread", Food, "bread.stl", pose=PoseStamped.from_list([2.5, 2, 1.0],
                                                                  [0, 0, 0, 1]),
           color=Color.from_list([1, 0, 0, 0, 1])).name


def pause_plan(myplan):
    myplan.root.pause()
    fts = ForceTorqueSensor(robot_name='pr2')
    thread = threading.Thread(target=fts.human_touch_monitoring(myplan), args=[myplan])
    thread.start()
    myplan.perform()
    thread.join()


def hri():
    with real_robot:
        perform(an(SetGripperActionDescription(Arms.RIGHT, GripperState.MEDIUM)))
        say_and_display_text("Waiting for Human")
        myplan = MoveGripperMotion(motion=GripperState.CLOSE, gripper=Arms.RIGHT)
        pause_plan(myplan)
        say_and_display_text("Continue my Plan")


def say_and_display_text(
        text: str,
        voice: str = 'voice_us2_mbrola',
        volume: float = 100.0,
        frame_id: str = 'base_link',
        position: tuple = (0, 0, 1.5)
):
    """
    Makes the robot say the given text and display it in RViz.

    Args:
        text (str): The text to say and display.
        voice (str): Voice to use for speech.
        volume (float): Volume level.
        frame_id (str): TF frame in which to display the text in RViz.
        position (tuple): (x, y, z) coordinates for text in RViz.
    """

    # Speech
    soundhandle = SoundClient()
    time.sleep(1)
    loginfo(f"Saying: {text}")
    soundhandle.say(text, voice, volume)
    from pycram.ros import ros1
    # RViz text marker

    marker_pub = create_publisher('visualization_marker', Marker, queue_size=1)

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "speech_display"
    marker.id = 0
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position = Point(*position)
    marker.scale.z = 0.2  # height of the text
    marker.color.a = 1.0  # alpha
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.text = text
    marker.lifetime = Duration(10)  # visible for 5 seconds

    time.sleep(0.5)

    marker_pub.publish(marker)
    time.sleep(1)


def demo():
    with real_robot:
        WorldStateUpdater("/tf", "/joint_states")

        perform(an(CarryActionDescription([Arms.BOTH], True, "r_gripper_tool_frame", AxisIdentifier.X,
                                          "torso_lift_link", AxisIdentifier.X)))
        tool_frame = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_tool_frame()

        tool_pose = robot.get_link_pose(tool_frame)
        tool_pose.position.x += 0.1
        q1 = utils.axis_angle_to_quaternion([1, 0, 0], 180)
        tool_pose.rotate_by_quaternion(q1)

        knife = Object("knife", PouringTool, "big-knife.stl", pose=tool_pose, size=[0.2, 0.1, 0.05])

        perform(an(SetGripperActionDescription(Arms.RIGHT, GripperState.MEDIUM)))
        say_and_display_text("Waiting for Human")
        myplan = MoveGripperMotion(motion=GripperState.CLOSE, gripper=Arms.RIGHT)
        pause_plan(myplan)

        World.robot.attach(knife, tool_frame)
        giskard.achieve_attached(knife, tool_frame)
        say_and_display_text("Continue my Plan")

        zuc = Object("zuc", Food, "cucumber.stl",
                     pose=PoseStamped.from_list([2.4, 2, 1.0], [0, 0, -1, -1]),
                     color=Color.from_list([1, 1, 0, 1]), size=[0.1, 0.2, 0.1])

        location_pose = PoseStamped.from_list([1.7, 2, 0])

        perform(an(NavigateActionDescription(location_pose)))
        perform(an(CuttingActionDescription(zuc, knife, [Arms.RIGHT])))


spawn_world()
demo()
