from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.enums import ObjectType

from pycram.ros.viz_marker_publisher import VizMarkerPublisher

world = BulletWorld()
v = VizMarkerPublisher()
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1, 2, 0]))
apartment = Object("apartment", ObjectType.ENVIRONMENT, "apartment-small.urdf")

milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])
cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([2.5, 2.3, 1.05]), color=[0, 1, 0, 1])
#spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([2.4, 2.2, 0.85]), color=[0, 0, 1, 1])
bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.38, 2.2, 1.02]), color=[1, 1, 0, 1])
#apartment.attach(spoon, 'cabinet10_drawer_top')
world.get_objects_by_name("floor")[0].set_color([0.824, 0.706, 0.549, 0.8])
apartment.set_color([0.5, 0.5, 0.5, 0.7])
pick_pose = Pose([2.7, 2.15, 1])

robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])

@with_simulated_robot
def move_and_detect(obj_type):
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(technique='default', object_type=obj_type).resolve().perform()

    return object_desig


with (simulated_robot):
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()
    LookAtAction(targets=[pick_pose]).resolve().perform()
    MoveTorsoAction([0.25]).resolve().perform()

    object_desig = DetectAction(technique='all').resolve().perform()
    object_dict = object_desig[1]
    for key, value in object_dict.items():
        if object_dict[key].type == "Cutlery" or object_dict[key].type == ObjectType.BOWL:
            grasp = "top"
        else:
            grasp = "front"
        PickUpAction(object_dict[key], ["left"], [grasp]).resolve().perform()
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        place_loc = CostmapLocation(target=Pose([4.8, 3.55, 0.8]), reachable_for=robot_desig.resolve(),
                                    reachable_arm="left").resolve()
        NavigateAction(target_locations=[place_loc.pose]).resolve().perform()
        MoveTorsoAction([0.25]).resolve().perform()
        PlaceAction(object_dict[key], ["left"], [grasp], [Pose([4.6, 3.55, 0.8])]).resolve().perform()
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()
        MoveTorsoAction([0.25]).resolve().perform()