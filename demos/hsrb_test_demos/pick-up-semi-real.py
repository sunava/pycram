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
world = BulletWorld("DIRECT")

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Initialize Giskard interface for motion planning
giskardpy.init_giskard_interface()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "hsrb.urdf", pose=Pose([1, 2, 0]))
robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
#apartment = Object("kitchen", ObjectType.ENVIRONMENT, "test-room.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 0)

# Define a breakfast cereal object
breakfast_cereal = Object("breakfast_cereal", "breakfast_cereal", "breakfast_cereal.stl", pose=Pose([4.8, 2.6, 0.87]),color=[0, 1, 0, 1])
# fork = Object("Fork", "fork", "spoon.stl", pose=Pose([-2.8, 2.3, 0.368], object_orientation), color=[1, 0, 0, 1])
# spoon = Object("Spoon", "spoon", "spoon.stl", pose=Pose([-2.5, 2.3, 0.368], object_orientation), color=[0, 1, 0, 1])
# metalmug = Object("Metalmug", "metalmug", "bowl.stl", pose=Pose([-3.1, 2.3, 0.39]), color=[0, 1, 0, 1])
# plate = Object("Plate", "plate", "board.stl", pose=Pose([-2.2, 2.3, 0.368], object_orientation), color=[0, 1, 0, 1])
#bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([4.8, 2.6, 0.87]), color=[0, 1, 0, 1])

# Set the world's gravity
#world.set_gravity([0.0, 0.0, 9.81])

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")


# Define a function for object detection and interaction
def look_and_detect(obj):
    """
    Directs the robot to look at an object and perform detection.

    Args:
    obj (Object): The object to detect.

    Returns:
    ObjectDesignator: The detected object designator.
    """
    LookAtAction(targets=[obj.pose]).resolve().perform()
    object_desig = DetectAction(BelieveObject(types=[obj.type]), technique='default').resolve().perform()
    return object_desig


# Main interaction sequence with semi-real robot
with semi_real_robot:
    rospy.loginfo("Starting demo")
    # Initial setup actions
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    MoveTorsoAction([0.25]).resolve().perform()

    # Navigate to cereal location
    NavigateAction(target_locations=[Pose([4.1, 2.2, 0], object_orientation)]).resolve().perform()

    # Detect and interact with the cereal
    dict_desig = look_and_detect(obj=breakfast_cereal)
    giskardpy.avoid_all_collisions()
    PickUpAction((dict_desig["breakfast_cereal"]), ["left"], ["front"]).resolve().perform()

    # Place the cereal at the target location
    cereal_target_pose = Pose([4.1, 2.2, 0], object_orientation)
    NavigateAction(target_locations=[cereal_target_pose]).resolve().perform()
    PlaceAction(dict_desig["breakfast_cereal"], ["left"], ["front"], [Pose([4.8, 2.6, 0.97])]).resolve().perform()

    # Finalize actions
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    rospy.loginfo("Done!")

