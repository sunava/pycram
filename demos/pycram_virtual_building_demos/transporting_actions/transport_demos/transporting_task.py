from pycram.context_knowledge import generate_context
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose
from pycram.process_module import simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher


def start_transporting_demo(location: str = "table_area_main", context: str = "breakfast",
                        environment: str = "apartment-small.urdf"):
    BulletWorld("DIRECT")
    VizMarkerPublisher()
    current_context = generate_context(context, environment)
    #current_context.spawn_objects()
    Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1, 2, 0]))
    with simulated_robot:
        TransportAction(location, current_context).resolve().perform()
    rospy.loginfo("Transporting task completed!")

#start_set_the_table()