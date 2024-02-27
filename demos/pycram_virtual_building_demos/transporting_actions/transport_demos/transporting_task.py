from pycram.context_knowledge import generate_context
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose
from pycram.process_module import simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from IPython.display import display, HTML, clear_output

def start_transporting_demo(location: str = "table_area_main", context: str = "breakfast",
                        environment: str = "apartment-small.urdf"):
    display(HTML('<img src="https://i.gifer.com/XVo6.gif" alt="Hourglass animation" width="50">'))
    world = BulletWorld("DIRECT")
    VizMarkerPublisher(interval=0.8)
    current_context = generate_context(context, environment)
    with simulated_robot:
        TransportAction(location, current_context).resolve().perform()

    clear_output(wait=True)
    rospy.loginfo("Transporting task completed!")
