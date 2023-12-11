from pycram.process_module import  with_real_robot
from pycram.designators.action_designator import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
# worked on 11.12.2023

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

world.set_gravity([0, 0, -9.8])

# initialize hsr robot in BulletWorld
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()

# initialize giskard
giskardpy.init_giskard_interface()


@with_real_robot
def pick_up_and_drop():
    """
    Let the HSR detect, pick up and drop five different objects.
    """
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    NavigateAction(target_locations=[Pose([3.9, 1.85, 0])]).resolve().perform()

    # take perceiving pose
    LookAtAction(targets=[Pose([4.4, 1.85, 0.8])]).resolve().perform()

    # use robokudo to perceive object poses
    object_desig = DetectAction(BelieveObject(types=[ObjectType.MILK]), technique='all').resolve().perform()
    object_list = [object_desig["Milkpack"], object_desig["Cerealbox"], object_desig["Pringleschipscan"],
                   object_desig["Crackerbox"], object_desig["Metalmug"]]

    # pick up and drop all five objects
    for index in range(len(object_list)):
        new_pose = object_list[index].pose

        ParkArmsAction([Arms.LEFT]).resolve().perform()
        # navigate hsr to the next object to pick up
        NavigateAction(target_locations=[Pose([4.0, new_pose.position.y, 0])]).resolve().perform()
        # differentiate the grasping movement between the metalmug and other objects
        if object_list[index].name == "Metalmug":
            PickUpAction(object_designator_description=object_list[index],
                         arms=["left"],
                         grasps=["top"]).resolve().perform()
            print("grasp from top")
        else:
            PickUpAction(object_designator_description=object_list[index],
                         arms=["left"],
                         grasps=["front"]).resolve().perform()
            print("grasp from front")

        # inform user before dropping the object
        TalkingMotion("I'll drop the object now!").resolve().perform()
        # drop the object
        MoveGripperMotion("open", "left", allow_gripper_collision=True).resolve().perform()

    # navigate hsr back to the starting position
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    NavigateAction(target_locations=[Pose([3.9, 1.85, 0])]).resolve().perform()
    print("finished")
