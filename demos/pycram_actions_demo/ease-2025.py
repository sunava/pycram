import os
import threading
import time

import rospkg

import pycram.utils
from pycram.failures import ObjectAlreadyExists
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
from pycrap.ontologies import *
from pycram.language import CodePlan, ParallelPlan, MonitorPlan
from pycram.ros_utils.force_torque_sensor import ForceTorqueSensor
from pycram.ros import set_logger_level
from pycram.ros import create_publisher, Duration
from pycram.datastructures.pose import PoseStamped, Vector3Stamped
from pycram.datastructures.pose import *

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

pot_pose = PoseStamped.from_list([2.6, 1.7, 1.0])
# pot = Object("Pot", Pot, "pot.stl", pose=pot_pose, size=[0.2, 0.1, 0.05])


#
# obj_tool_ = Object("jeroen_cup", PouringTool, "jeroen_cup.stl", pose=PoseStamped.from_list([3.4, 2, 1.0], [0, 0, 0, 1]))
# m = ManualMarkerPublisher()

def look_at():
    looking_pose = PoseStamped.from_list([2.3, 4, 0.97])
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


zuchinni = None
big_knife = None


def demo(start_step=1, continue_steps=True, clean=False):
    global big_knife, zuchinni  # declare them global at the top of the function

    with real_robot:
        # === STEP 1: Initialize World State ===
        if start_step <= 1 and (continue_steps or start_step == 1):
            WorldStateUpdater("/tf", "/joint_states")

        # === STEP 2: Perform Carry Action ===
        if start_step <= 2 and (continue_steps or start_step == 2):
            perform(an(CarryActionDescription(
                [Arms.BOTH], True, "r_gripper_tool_frame", AxisIdentifier.X,
                "torso_lift_link", AxisIdentifier.X
            )))

        # === STEP 3: Get Tool Frame and Modify Pose ===
        if start_step <= 3 and (continue_steps or start_step == 3):
            tool_frame = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_tool_frame()
            tool_pose = robot.get_link_pose(tool_frame)
            tool_pose.position.x += 0.1
            q1 = utils.axis_angle_to_quaternion([1, 0, 0], 180)
            tool_pose.rotate_by_quaternion(q1)

        # === STEP 4: Spawn Knife Object ===
        if start_step <= 4 and (continue_steps or start_step == 4):
            try:
                knife = Object("knife", PouringTool, "big-knife.stl", pose=tool_pose, size=[0.2, 0.1, 0.05])
                big_knife = knife
            except ObjectAlreadyExists:
                knife = world.get_object_by_type(PouringTool)
                big_knife = knife
        # fallback

        # === STEP 5: Human Interaction Prompt ===
        if start_step <= 5 and (continue_steps or start_step == 5):
            say_and_display_text("Updating Belief State")

        # === STEP 6: Prepare Gripper Motion ===
        if start_step <= 6 and (continue_steps or start_step == 6):
            # myplan = MoveGripperMotion(motion=GripperState.CLOSE, gripper=Arms.RIGHT)
            # pause_plan(myplan)
            MoveGripperMotion(motion=GripperState.CLOSE, gripper=Arms.RIGHT)
        # === STEP 7: Attach Knife to Robot ===
        if start_step <= 7 and (continue_steps or start_step == 7):
            try:
                giskard.achieve_attached(knife, tool_frame)
                world.robot.attach(knife, tool_frame)
            except Exception:
                pass

        # === STEP 8: Resume and Continue Plan ===
        if start_step <= 8 and (continue_steps or start_step == 8):
            say_and_display_text("Continuing")


        # === STEP 9: Perform Navigation Action ===
        if start_step <= 9 and (continue_steps or start_step == 9):
            robot_pose = PoseStamped(
                pose=Pose(position=Vector3(x=1.66, y=3.45, z=8.877688908143049e-05),
                          orientation=Quaternion(x=-0.00011441241581541074, y=0.003105769003427517,
                                                 z=-0.013065672388969038, w=0.9999098105897954)))
            perform(an(NavigateActionDescription(robot_pose)))


        # === STEP 10: Look at Table Position ===
        if start_step <= 10 and (continue_steps or start_step == 10):
            looking_pose = PoseStamped.from_list([2.4, 2.77, 0.97])
            perform(an(LookAtActionDescription(looking_pose)))
            perform(an(CarryActionDescription(
                [Arms.BOTH], True, "r_gripper_tool_frame", AxisIdentifier.Z,
                "cabinet1", AxisIdentifier.Y
            )))


        # === STEP 11: Detect Objects ===
        if start_step <= 11 and (continue_steps or start_step == 11):
            say_and_display_text("Perceiving")
            detect_objects = detect(BelieveObject(types=[Zucchini]))
            try:
                zuc = detect_objects[0]
                zuchinni = zuc
            except IndexError:
                detect_objects = detect(BelieveObject(types=[Cucumber]))
                try:
                    zuc = detect_objects[0]
                    zuchinni = zuc
                except IndexError:
                    return "No Perception Object found, redo from STEP 9"
            perform(an(CarryActionDescription(
                [Arms.BOTH], True, "r_gripper_tool_frame", AxisIdentifier.X,
                "torso_lift_link", AxisIdentifier.X
            )))
        # # === STEP 12: Perform Navigation Action ===
        # if start_step <= 12 and (continue_steps or start_step == 12):
        #     robot_pose = PoseStamped(
        #         pose=Pose(position=Vector3(x=1.66, y=3.35, z=8.877688908143049e-05),
        #                   orientation=Quaternion(x=-0.00011441241581541074, y=0.003105769003427517,
        #                                          z=-0.013065672388969038, w=0.9999098105897954)))
        #     perform(an(NavigateActionDescription(robot_pose)))
        #
        # # === STEP 13: Look at Table Position ===
        # if start_step <= 13 and (continue_steps or start_step == 13):
        #     looking_pose = PoseStamped.from_list([2.4, 2.77, 0.97])
        #     perform(an(LookAtActionDescription(looking_pose)))

        # === STEP 14: Perform Cutting Action -> 0.024===
        if start_step <= 14 and (continue_steps or start_step == 14):
            say_and_display_text("Planning")
            perform(an(CuttingActionDescription(zuchinni, big_knife, [Arms.RIGHT], slice_thickness=0.01)))

        # === STEP 15: Perform Carry Action ===

        if start_step <= 15 and (continue_steps or start_step == 15):
            say_and_display_text("Moving into Home position")
            perform(an(CarryActionDescription(
                [Arms.BOTH], True, "r_gripper_tool_frame", AxisIdentifier.X,
                "torso_lift_link", AxisIdentifier.X
            )))

        # === STEP 14: Clean Up ===
        if start_step <= 16 and (continue_steps or start_step == 16) and clean:
            world.remove_object(zuchinni)
            world.remove_vis_axis()

# def demo2():
#     robot_desig_resolved = BelieveObject(names=[RobotDescription.current_robot_description.name]).resolve()
#     ParkArmsActionDescription(Arms.BOTH).perform()
#     pickup_loc = CostmapLocation(target=self.object_designator,
#                                  reachable_for=robot_desig_resolved,
#                                  reachable_arm=[self.arm])
#     # Tries to find a pick-up position for the robot that uses the given arm
#     pickup_pose = pickup_loc.resolve()
#     if not pickup_pose:
#         raise ObjectUnfetchable(
#             f"Found no pose for the robot to grasp the object: {self.object_designator} with arm: {self.arm}")
#
#     NavigateActionDescription(pickup_pose, True).perform()
#     PickUpActionDescription(self.object_designator, pickup_pose.arm,
#                             grasp_description=pickup_pose.grasp_description).perform()
