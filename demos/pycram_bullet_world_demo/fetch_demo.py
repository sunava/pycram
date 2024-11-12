from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType, WorldMode, TorsoState
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color

extension = ObjectDescription.get_file_extension()

world = BulletWorld(WorldMode.DIRECT)
world.allow_publish_debug_poses = True
viz = VizMarkerPublisher()
tf = TFBroadcaster()

robot_name = "fetch"
robot = Object(robot_name, ObjectType.ROBOT, f"{robot_name}{extension}", pose=Pose([1, 2, 0]))

apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment-small{extension}")
milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([0.5, 2.5, 1], [0, 0, 0, 1]))
milk.color = Color(0, 0, 1, 1)
milk_desig = BelieveObject(names=["milk"])
robot_desig = BelieveObject(names=[robot_name])
apartment_desig = BelieveObject(names=["apartment"])

with simulated_robot:
    start_pose = Pose([1.3, 2.7, 0], [0, 0, 1, 0])
    milk_target_pose = Pose([5.34, 3.55, 0.8])

    NavigateAction([start_pose]).resolve().perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()

    handle_designator = ObjectPart(names=["handle_cab3_door_top"], part_of=apartment_desig.resolve())
    closed_location, opened_location = AccessingLocation(handle_desig=handle_designator.resolve(),
                                                         robot_desig=robot_desig.resolve()).resolve()
    OpenAction(object_designator_description=handle_designator, arms=[closed_location.arms[0]],
               start_goal_location=[closed_location, opened_location]).resolve().perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    NavigateAction([start_pose]).resolve().perform()
    LookAtAction(targets=[milk_desig.resolve().pose]).resolve().perform()
    object_designator = DetectAction(milk_desig).resolve().perform()

    side_grasp, _ = calculate_object_faces(object_designator)

    try:
        pickup_loc = CostmapLocation(
            target=object_designator,
            reachable_for=robot_desig.resolve(),
            reachable_arm=closed_location.arms[0],
            used_grasps=[side_grasp]
        ).resolve()
    except StopIteration:
        raise ReachabilityFailure(
            f"No reachable location found for the pickup location: {object_designator.pose}"
        )

    NavigateActionPerformable(pickup_loc.pose).perform()
    PickUpAction(object_designator, [closed_location.arms[0]], [side_grasp]).resolve().perform()
    ParkArmsActionPerformable(Arms.BOTH).perform()


    try:
        place_loc = CostmapLocation(
            target=milk_target_pose,
            reachable_for=robot_desig.resolve(),
            reachable_arm=closed_location.arms[0],
            used_grasps=[side_grasp],
            object_in_hand=object_designator
        ).resolve()
    except StopIteration:
        raise ReachabilityFailure(
            f"No reachable location found for the target location: {milk_target_pose}"
        )

    NavigateActionPerformable(place_loc.pose).perform()
    PlaceAction(object_designator, [milk_target_pose], [closed_location.arms[0]]).resolve().perform()
    ParkArmsActionPerformable(Arms.BOTH).perform()

    CloseAction(object_designator_description=handle_designator, arms=[opened_location.arms[0]],
                start_goal_location=[opened_location, closed_location]).resolve().perform()
    ParkArmsActionPerformable(Arms.BOTH).perform()

