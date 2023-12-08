import rospy
from tmc_msgs.msg import Voice
from pycram.process_module import real_robot, semi_real_robot, with_real_robot
from pycram.designators.action_designator import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater

# worked on 8.12.2023 together with Mohammad

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

world.set_gravity([0, 0, -9.8])

# turn robot and some object for 90 degrees
object_orientation = axis_angle_to_quaternion([0, 0, 1], 90)

# Initialize objects in BulletWorld
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()

kitchen = Object("kitchen", "environment", "kitchen.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

milk = Object("milk", "milk", "milk.stl", pose=Pose([-2.7, 2.3, 0.43]), color=[1, 0, 0, 1])
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([-2.5, 2.3, 0.43], object_orientation),
                color=[0, 1, 0, 1])
metalmug = Object("metalmug", "metalmug", "bowl.stl", pose=Pose([-2.9, 2.3, 0.43], object_orientation), color=[0, 1, 0, 1])
cheezels = Object("cheezels", "cheezels", "breakfast_cereal.stl", pose=Pose([-2.3, 2.3, 0.43], object_orientation), color=[0, 1, 0, 1])
pringles = Object("pringles", "pringles", "milk.stl", pose=Pose([-3.1, 2.3, 0.43]), color=[0, 1, 0, 1])


milk_desig = BelieveObject(names=["milk"])
cereal_desig = BelieveObject(names=["cereal"])
metalmug_desig = BelieveObject(names=["metalmug"])
cheezels_desig = BelieveObject(names=["cheezels"])
pringles_desig = BelieveObject(names=["pringles"])


object_list = [pringles_desig, metalmug_desig, milk_desig, cereal_desig, cheezels_desig]

# design publish message for informing about dropping the object
pub = rospy.Publisher('/talk_request', Voice, queue_size=10)
drop_str = Voice()
drop_str.language = 1
drop_str.sentence = "I drop the object now!"

# initialize and sync giskard
giskardpy.init_giskard_interface()
giskardpy.sync_worlds()

RobotStateUpdater("/tf", "/joint_states")


with semi_real_robot:
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    LookAtAction(targets=[Pose([-2.6, 2.0, 0])]).resolve().perform()

    # navigate to table
    NavigateAction(target_locations=[Pose([-2.6, 0, 0], object_orientation)]).resolve().perform()
    NavigateAction(target_locations=[Pose([-2.6, 1.5, 0], object_orientation)]).resolve().perform()

    # take perceiving pose
    LookAtAction(targets=[Pose([-2.6, 2.0, 0.8])]).resolve().perform()

    # pick up and drop all five objects
    for index in range(len(object_list)):
        # use robokudo to perceive object pose
        object_desig = DetectAction(object_list[index]).resolve().perform()
        new_pose = object_desig.pose
        # navigate hsr to the next object to pick up
        NavigateAction(target_locations=[Pose([new_pose.position.x, 1.5, 0], object_orientation)]).resolve().perform()
        # differentiate the grasping movement between metalmug and other objects
        if object_list[index] == metalmug_desig:
            PickUpAction(object_designator_description=object_desig,
                         arms=["left"],
                         grasps=["top"]).resolve().perform()
            print("grasp from top")
        else:
            PickUpAction(object_designator_description=object_desig,
                         arms=["left"],
                         grasps=["front"]).resolve().perform()
            print("grasp from front")

        print(object_desig.pose)
        # inform user before dropping the object
        pub.publish(drop_str)
        print(drop_str)
        # drop/place the object
        place_pose = Pose([new_pose.position.x - 0.3, new_pose.position.y, new_pose.position.z])
        PlaceAction(object_desig, [place_pose], ["left"]).resolve().perform()
        print("placing finished")
        # go back for perceiving the next object
        NavigateAction(target_locations=[Pose([-2.6, 1.5, 0], object_orientation)]).resolve().perform()

    ParkArmsAction([Arms.LEFT]).resolve().perform()
    NavigateAction(target_locations=[Pose([-2.6, 0, 0], object_orientation)]).resolve().perform()
    print("finished")
# MoveGripperMotion("open", "left", allow_gripper_collision=True).resolve().perform()



#manipulation perceive pose
#perceived_items[objectdesig] = perception call
#     spawn perceived_items
#     syn.world giskard
#     add.mesh to giskard
#     loop for items
#        wenn items[i] == item1 dann
#         item1 = items[i]
#        ...
#      item1 pick up
#      drop item / place item1 in simulation
#      ...
#
