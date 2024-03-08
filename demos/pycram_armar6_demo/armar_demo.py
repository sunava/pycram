from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher


world = BulletWorld("DIRECT")
viz = VizMarkerPublisher()
robot = Object("Armar6", "robot", "Armar6.urdf", pose=Pose([1, 2, 0]))
apartment = Object("apartment", "environment", "apartment-small.urdf")

milk = Object("milk", "milk", "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([2.5, 2.4, 1.05]), color=[0, 1, 0, 1])
spoon = Object("spoon", "spoon", "spoon.stl", pose=Pose([2.4, 2.2, 0.85]), color=[0, 0, 1, 1])
bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([2.5, 2.2, 1.02]), color=[1, 1, 0, 1])

#milk = Object("milk", "milk", "milk.stl", pose=Pose([4.8, 4.2, 0.8]), color=[1, 0, 0, 1])
#cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([4.8, 4, 0.8]), color=[0, 1, 0, 1])
#spoon = Object("spoon", "spoon", "spoon.stl", pose=Pose([4.8, 3.5, 0.8]), color=[0, 0, 1, 1])
#bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([4.8, 3.7, 0.8]), color=[1, 1, 0, 1])

apartment.attach(spoon, 'cabinet10_drawer_top')

pick_pose_milk = Pose([2.7, 2.15, 1])
pick_pose_cereal = Pose([2.7, 2.35, 1])


robot_desig = BelieveObject(names=["Armar6"])
apartment_desig = BelieveObject(names=["apartment"])

@with_simulated_robot
def move_and_detect(obj_type, pick_pose):
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    status, object_dict = DetectAction(technique='specific', object_type=obj_type).resolve().perform()
    if status:
        for key, value in object_dict.items():
            detected_object = object_dict[key]

    return detected_object


with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([-0.1]).resolve().perform()

    # milk
    milk_desig = move_and_detect("milk", pick_pose_milk)

    NavigateAction([Pose([1.7, 1.5, 0], [0, 0, 0, 1])]).resolve().perform()

    PickUpAction.Action(milk_desig, "left", "front").perform()

    ParkArmsAction.Action(Arms.BOTH).perform()

    NavigateAction([Pose([4.1, 3.8, 0], [0, 0, 0, 1])]).resolve().perform()

    PlaceAction(milk_desig, ["left"], ["front"], [Pose([4.8, 4.2, 0.8])]).resolve().perform()

    ParkArmsAction.Action(Arms.BOTH).perform()

    # cereal
    cereal_desig = move_and_detect("cereal", pick_pose_cereal)

    NavigateAction([Pose([1.7, 1.8, 0], [0, 0, 0, 1])]).resolve().perform()

    PickUpAction.Action(cereal_desig, "left", "front").perform()

    ParkArmsAction.Action(Arms.BOTH).perform()

    NavigateAction([Pose([4.1, 3.8, 0], [0, 0, 0, 1])]).resolve().perform()

    PlaceAction(cereal_desig, ["left"], ["front"], [Pose([4.8, 4, 0.8], [0, 0, 0, 1])]).resolve().perform()

    ParkArmsAction.Action(Arms.BOTH).perform()
    
    # bowl
    bowl_desig = move_and_detect("bowl", pick_pose_cereal)

    NavigateAction([Pose([1.7, 2.5, 0], [0, 0, 0, 1])]).resolve().perform()

    PickUpAction.Action(bowl_desig, "right", "top").perform()

    ParkArmsAction.Action(Arms.BOTH).perform()

    NavigateAction([Pose([4.1, 3.8, 0], [0, 0, 0, 1])]).resolve().perform()

    PlaceAction(bowl_desig, ["right"], ["top"], [Pose([4.8, 3.7, 0.8], [0, 0, 0, 1])]).resolve().perform()

    ParkArmsAction.Action(Arms.BOTH).perform()

