import rospkg
from pycram.context_knowledge import generate_context
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose
from pycram.process_module import simulated_robot
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from IPython.display import display, HTML, clear_output

def start_transporting_demo(location: str = "table_area_main", context: str = "breakfast",
                        environment: str = "apartment-small.urdf"):
    display(HTML('<img src="https://i.gifer.com/XVo6.gif" alt="Hourglass animation" width="50">'))
    world = BulletWorld("DIRECT")
    VizMarkerPublisher(interval=0.8)
    current_context = generate_context(context, environment)
    # Initialize a ROS package object
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('pycram') + '/resources/' + environment
    urdf_string = helper.urdf_to_string(package_path)
    rospy.set_param('kitchen_description', urdf_string)
    TFBroadcaster(interval=0.0002)


    with simulated_robot:
        TransportAction(current_context, target_location=location).resolve().perform()

    clear_output(wait=True)
    rospy.loginfo("Transporting task completed!")
