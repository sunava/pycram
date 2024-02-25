from pycram.designators.action_designator import TransportAction
from pycram.process_module import simulated_robot
from pycram.pose import Pose
from pycram.enums import ObjectType
from pycram.context_knowledge import generate_context
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.bullet_world import BulletWorld, Object
from demos.pycram_virtual_building_demos.transporting_actions.transport_demos.transporting_task import \
    start_transporting_demo

location: str = "table_area_main"
context: str = "breakfast"
environment: str = "apartment-small.urdf"

world = BulletWorld("DIRECT")
VizMarkerPublisher()
current_context = generate_context(context, environment)
Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1, 2, 0]))
with simulated_robot:
    TransportAction(location, current_context).resolve().perform()