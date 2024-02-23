import rospy

from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose
from pycram.process_module import simulated_robot
from init_setup import test_context_apartment, breakfast_context_apartment
from pycram.ros.viz_marker_publisher import VizMarkerPublisher

# Initialize the simulation world and visual markers
world = BulletWorld()
VizMarkerPublisher()

# Initialize the robot
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1, 2, 0]))
robot_desig = BelieveObject(names=["pr2"])

# Define environment and objects
# todo rework this and use the proba model
apart_desig = BelieveObject(names=["apartment"])
current_context = breakfast_context_apartment  # or breakfast_context_apartment
current_context.spawn_objects()




def set_the_table(target_location, _current_context):
    with simulated_robot:
     rospy.loginfo("Setting the table")
     TransportAction(target_location, _current_context).resolve().perform()

set_the_table("table", current_context)

#todo allgemein failure handling fehlt noch und mehr generic und die locations besondern
# fuer hin und her transporting muessten anders sein
