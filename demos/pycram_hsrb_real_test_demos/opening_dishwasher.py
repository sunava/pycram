from enum import Enum
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_hsrb_real_test_demos.utils.misc import *
# from pycram.external_interfaces.knowrob import get_table_pose
import pycram.helper as helper
import rospkg
# publish robot on param server for gripper tool frame
rospack = rospkg.RosPack()
name = "hsrb.urdf"
package_path = rospack.get_path('pycram') + '/resources/' + name
urdf_string = helper.urdf_to_string(package_path)
rospy.set_param('robot_description', urdf_string)


world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))

# Update robot state

giskardpy.init_giskard_interface()
RobotStateUpdater("/tf", "/giskard_joint_states")
robot.set_color([0.5, 0.5, 0.9, 1])

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "couch-dishwasher-kitchen.urdf")
apart_desig= BelieveObject(names=["kitchen"])
# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)

#giskardpy.sync_worlds()



# Main interaction sequence with real robot
with ((real_robot)):
    rospy.loginfo("Starting demo")
   # # ParkArmsAction([Arms.LEFT]).resolve().perform()
   #  handle_desig = ObjectPart(names=["dishwasher:dishwasher:dishwasher_door_handle"], part_of=apart_desig.resolve())
   #  OpenAction(object_designator_description=handle_desig, arms=["left"]).resolve().perform()