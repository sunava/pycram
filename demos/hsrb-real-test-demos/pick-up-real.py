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


giskardpy.init_giskard_interface()
#giskardpy.removing_of_objects()
giskardpy.sync_worlds()
#giskardpy.spawn_kitchen()
# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([1, 2, 0]))
robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "test-room.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 0)

# Define a breakfast cereal object
#breakfast_cereal = Object("breakfast_cereal", "breakfast_cereal", "breakfast_cereal.stl", pose=Pose([4.8, 2.6, 0.87]),color=[0, 1, 0, 1])
# fork = Object("Fork", "fork", "spoon.stl", pose=Pose([-2.8, 2.3, 0.368], object_orientation), color=[1, 0, 0, 1])
# spoon = Object("Spoon", "spoon", "spoon.stl", pose=Pose([-2.5, 2.3, 0.368], object_orientation), color=[0, 1, 0, 1])
# metalmug = Object("Metalmug", "metalmug", "bowl.stl", pose=Pose([-3.1, 2.3, 0.39]), color=[0, 1, 0, 1])
# plate = Object("Plate", "plate", "board.stl", pose=Pose([-2.2, 2.3, 0.368], object_orientation), color=[0, 1, 0, 1])
#bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([4.8, 2.6, 0.87]), color=[0, 1, 0, 1])

# Set the world's gravity
#world.set_gravity([0.0, 0.0, 9.81])

# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

# Main interaction sequence with semi-real robot
with real_robot:
    rospy.loginfo("Starting demo")
    TalkingMotion("Starting demo").resolve().perform()

    MoveGripperMotion(motion="open", gripper="left").resolve().perform()
    #
    # #Initial setup actions
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    LookAtAction(targets=[Pose([3.8, 1.3, 0.33])]).resolve().perform()
    #LookAtAction(targets=[Pose([3.8, 1.3, 0.33])]).resolve().perform()
    # # Navigate to cereal location
    TalkingMotion("Navigating").resolve().perform()
    #NavigateAction(target_locations=[Pose([4.1, 2, 0], object_orientation)]).resolve().perform()
    object_desig = DetectAction(technique='default').resolve().perform()
    object_dict = object_desig[1]
    one = False
    #todo only pick up when on table
    for key, value in object_dict.items():
        #if not one:
        if object_dict[key].type == "Bowl":
            TalkingMotion("Picking Up").resolve().perform()
            grasp = "top"
            PickUpAction(object_dict[key], ["left"], [grasp]).resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            NavigateAction(target_locations=[Pose([4.1, 2, 0], object_orientation)]).resolve().perform()


            PlaceAction(object_dict[key], ["left"], [grasp], [Pose([4.8, 2.6, 0.80])]).resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            #one = True
            # MoveGripperMotion(motion="open", gripper="left").resolve().perform()
            # ParkArmsAction([Arms.LEFT]).resolve().perform()


    #Place the cereal at the target location
    # cereal_target_pose = Pose([4.1, 2.2, 0], object_orientation)
    # NavigateAction(target_locations=[cereal_target_pose]).resolve().perform()
    # PlaceAction(dict_desig["breakfast_cereal"], ["left"], ["front"], [Pose([4.8, 2.6, 0.97])]).resolve().perform()
    #
    # #Finalize actions
    # ParkArmsAction([Arms.LEFT]).resolve().perform()
    #giskardpy.removing_of_objects()
    rospy.loginfo("Done!")
    TalkingMotion("Done").resolve().perform()

