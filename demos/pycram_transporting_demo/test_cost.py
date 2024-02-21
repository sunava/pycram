import pycram.plan_failures
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
apartment = Object("apartment", ObjectType.ENVIRONMENT, "apartment-small.urdf")

milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])
cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([2.5, 2.3, 1.05]),
                color=[0, 1, 0, 1])
# spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([2.4, 2.2, 0.85]), color=[0, 0, 1, 1])
bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.38, 2.2, 1.02]), color=[1, 1, 0, 1])
# apartment.attach(spoon, 'cabinet10_drawer_top')
# world.get_objects_by_name("floor")[0].set_color([0.824, 0.706, 0.549, 0.8])
# apartment.set_color([0.5, 0.5, 0.5, 0.7])
pick_pose = Pose([2.7, 2.15, 1])
kitchen_desig = ObjectDesignatorDescription(names=["apartment"])
robot_desig = BelieveObject(names=["pr2"])
cereal_desig = BelieveObject(names=["cereal"])


with (simulated_robot):
    place_loc = CostmapLocation(target=Pose([4.8, 3.55, 0.8]), reachable_for=robot_desig.resolve(),
                                reachable_arm="left")
    for locations in place_loc.resolve():
        print(locations)

