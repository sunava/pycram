from demos.pycram_transporting_demo.init_setup import breakfast_context_apartment
from pycram.bullet_world import BulletWorld, Object
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot


world = BulletWorld()
# v = VizMarkerPublisher()


robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1, 2, 0]))
current_context = breakfast_context_apartment  # Or dinner_context, depending on the scenario
current_context.spawn_objects()
location_to_search = current_context.search_locations("spoon")
print(location_to_search)