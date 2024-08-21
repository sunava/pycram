# from pycram.designators.motion_designator import TalkingMotion


from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot, semi_real_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color
import pycram.external_interfaces.giskard as giskardpy

extension = ObjectDescription.get_file_extension()
world = BulletWorld(WorldMode.DIRECT)

v = VizMarkerPublisher()
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "apartment.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
extension = ObjectDescription.get_file_extension()


robot = Object("hsrb", ObjectType.ROBOT, f"hsrb{extension}", pose=Pose([1, 2, 0]))

# robot.set_color(0.5, 0.5, 0.9, 1)
robot_desig = ObjectDesignatorDescription(names=["hsrb"])
table_pose = Pose([6.6, 4.9, 0.0], [0.0, 0.0, 0, 1])
long_table_1 = Pose([6.65, 4.6, 0],[0, 0, 0, 1])
long_table_pick = Pose([6.60, 4.6, 0],[0, 0, 0, 1])

long_table_1_rotated = Pose([6.65, 4.6, 0],[0, 0, 1, 0])
shelf_1 = Pose([6.2, 5.6, 0],[0, 0, 1, 0])
shelf_1_rotated1 = Pose([6.2, 5.6, 0],[0, 0, -0.7, 0.7])
shelf_1_rotated = Pose([6.2, 5.6, 0],[0, 0, 0, 1])
lt = LocalTransformer()

milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])
cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([2.5, 2.3, 1.05]),
                color=[0, 1, 0, 1])
spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([2.4, 2.2, 0.85]), color=[0, 0, 1, 1])
bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.5, 2.2, 1.02]), color=[1, 1, 0, 1])
human_female = Object("human_female", ObjectType.HUMAN, "female_standing.stl", pose=Pose([3, 3, 0]), color=[1, 1, 0, 1])


world.simulate(seconds=1, real_time=True)

@giskardpy.init_giskard_interface
def test_move():
    pose1 = Pose([1, 1, 0])
    giskardpy.teleport_robot(pose1)
    with semi_real_robot:
        NavigateAction([Pose([2, 2, 0])]).resolve().perform()
        giskardpy.teleport_robot(pose1)

def test_all_perception():
    with semi_real_robot:
        perceived_obj = DetectAction(technique=PerceptionTechniques.ALL).resolve().perform()
        for obj_desig in perceived_obj.values():
            print(obj_desig)

def test_types_perception():
    with semi_real_robot:
        perceived_obj = DetectAction(technique=PerceptionTechniques.TYPES,
                                     object_designator=BelieveObject(types=[ObjectType.MILK])).resolve().perform()
        for obj_desig in perceived_obj.values():
            print(obj_desig)


def test_talk():
    with semi_real_robot:
        TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").perform()


def test_costmap():
    sem = SemanticCostmap(kitchen_desig.resolve().world_object, "island_countertop")
    sem.visualize()