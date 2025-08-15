from pycram.robot_execution.motion_executioner import  execute_leaf_motions_with_giskard
from pycram.worlds.bullet_world import BulletWorld
from pycram.datastructures.enums import WorldMode
from pycram.process_module import simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription as URDFObjectDescription
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycrap.ontologies import Robot
from pycram.robot_plans import *

np.random.seed(420)
extension = URDFObjectDescription.get_file_extension()
world = BulletWorld(WorldMode.DIRECT)
viz = VizMarkerPublisher()

robot = Object("pr2", Robot, f"pr2{extension}", pose=PoseStamped.from_list([1, 2, 0]))
# apartment = Object("apartment", Apartment, f"apartment{extension}")

# milk = Object("milk", Milk, "milk.stl", pose=PoseStamped.from_list([2.5, 2, 1.02], [0, 0, 0, 1]),
#               color=Color(1, 0, 0, 1))
# cereal = Object("cereal", Cereal, "breakfast_cereal.stl",
#                 pose=PoseStamped.from_list([2.45, 2.4, 1.05], [0, 0, 0, 1]), color=Color(0, 1, 0, 1))
# spoon = Object("spoon", Spoon, "spoon.stl", pose=PoseStamped.from_list([2.4, 2.24, 0.85]),
#                color=Color(0, 0, 1, 1))
# bowl = Object("bowl", Bowl, "bowl.stl", pose=PoseStamped.from_list([2.35, 2.2, 0.98]),
#               color=Color(1, 1, 0, 1))
# apartment.attach(spoon, 'cabinet10_drawer_top')
location_pose = PoseStamped.from_list([1.7, 2, 0])
looking_pose = PoseStamped.from_list([2.5, 2, 0.97])
with simulated_robot:
    # sp = ParkArmsActionDescription(Arms.BOTH)

    # sp.perform()
    # Usage example:
    sq = SequentialPlan(NavigateActionDescription(location_pose))
    sq.perform()

    execute_leaf_motions_with_giskard(sq)
    # execute_plan_with_giskard(sq)
    # print("gisard")
    # sq.perform()

    # print("demo is done")

    # Simulation as before
    # result_sim = NavigateAction(location_pose).perform()

    # Real robot run that reuses the same Action and Motion,
    # but performs the MoveMotion through Giskard instead of World.robot.set_pose

# viz._stop_publishing()
# world.exit()
