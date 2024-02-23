import rospy
from IPython.display import display, clear_output, HTML

from .init_setup import breakfast_context_apartment
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose
from pycram.process_module import simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher


def set_the_table(context):
    display(HTML('<img src="https://i.gifer.com/XVo6.gif" alt="Hourglass animation" width="50">'))
    # Initialize the simulation world and visual markers
    world = BulletWorld("DIRECT")
    VizMarkerPublisher()

    # Initialize the robot
    robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1, 2, 0]))
    current_context = breakfast_context_apartment  # or breakfast_context_apartment
    current_context.spawn_objects()

    def set_the_table(target_location, _current_context):
        with simulated_robot:
            rospy.loginfo("Setting the table with context: %s", context)
            TransportAction(target_location, _current_context).resolve().perform()

    set_the_table("table", current_context)


    clear_output(wait=True)
    rospy.loginfo("Transporting task completed!")
