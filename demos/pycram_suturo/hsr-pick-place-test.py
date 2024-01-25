import rospy

from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.designators.motion_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot, semi_real_robot, real_robot
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
import pycram.external_interfaces.giskard as giskardpy
from pycram.external_interfaces.robokudo import queryEmpty, queryHuman
world = BulletWorld("DIRECT")
v = VizMarkerPublisher()
robot = Object("hsrb", ObjectType.ROBOT, "hsrb.urdf", pose=Pose([1, 2, 0]))
# human = Object("human", ObjectType.MILK, "human_male.stl", pose=Pose([0, 0, 0]))

#apartment = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")
# turn robot and some object for 90 degrees
object_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
#host = Object("humanhost", ObjectType.HUMAN, "human_male.stl", pose=Pose([-2.5, 1.5, 0]))
#mug = Object("milk", "milk", "milk.stl", pose=Pose([-2.7, 2.5, 0.43]), color=[1, 0, 0, 1])
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([-2.5, 1.5, 0.43], object_orientation),
                color=[0, 1, 0, 1])
# metalmug = Object("metalmug", "metalmug", "bowl.stl", pose=Pose([-2.9, 2.3, 0.43], object_orientation), color=[0, 1, 0, 1])
# cheezels = Object("cheezels", "cheezels", "breakfast_cereal.stl", pose=Pose([-2.3, 2.3, 0.43], object_orientation), color=[0, 1, 0, 1])
# pringles = Object("pringles", "pringles", "milk.stl", pose=Pose([-3.1, 2.3, 0.43]), color=[0, 1, 0, 1])

#
#milk_desig = BelieveObject(names=["milk"])
cereal_desig = BelieveObject(names=["cereal"])
# metalmug_desig = BelieveObject(names=["metalmug"])
# cheezels_desig = BelieveObject(names=["cheezels"])
# pringles_desig = BelieveObject(names=["pringles"])

robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = BelieveObject(names=["hsr"])
apartment_desig = BelieveObject(names=["kitchen"])

#object_list = [pringles_desig, metalmug_desig, milk_desig, cereal_desig, cheezels_desig]


# # initialize and sync giskard
giskardpy.init_giskard_interface()
#giskardpy.sync_worlds()
#
#RobotStateUpdater("/tf", "/joint_states")
#

# @with_simulated_robot
def move_and_detect(obj):
   # NavigateAction(target_locations=[Pose([-2.6, 0.8, 0], object_orientation)]).resolve().perform()
   # LookAtAction(targets=[obj.pose]).resolve().perform()
    #LookAtAction(targets=[obj.pose]).resolve().perform()

    # unkown_obj = BelieveObject(names=[ObjectType.MILK])
    # detected_objects= DetectingMotion(BelieveObject(types=[obj.type]), technique="all").resolve().perform()
    # print(detected_objects)
    # print("done")
    # rospy.loginfo("detected objects: {}".format(detected_objects))
    object_desig = DetectAction(BelieveObject(types=[obj.type]), technique='all').resolve().perform()
    #print(list(object_desig))
    print(list(object_desig)[0])
    # print(object_desig.values()[0].name)
    return object_desig


with real_robot:
    #query=queryHuman()
    #print(query)
    #human = DetectingMotion(object_type=ObjectType.SPOON,
     #               technique="human").resolve().perform()
    #print(human)
    #object_desig = DetectAction(BelieveObject(types=[cereal.type]), technique='human').resolve().perform()
    TalkingMotion("test").resolve().perform()
    # ParkArmsAction([Arms.LEFT]).resolve().perform()
    dict_desig = move_and_detect(obj=cereal)
    # MoveTorsoAction([0.25]).resolve().perform()
    #PickUpAction(dict_desig['Crackerbox'], ["left"], ["top"]).resolve().perform()
    print("starting mit giskard")
    giskardpy.initial_adding_objects()
    giskardpy.spawn_kitchen()
    print("done")
    #PickUpAction(dict_desig['Crackerbox'], ["left"], ["left"]).resolve().perform()
    # ParkArmsAction([Arms.LEFT]).resolve().perform()
    # PlaceAction(dict_desig['cereal'], [Pose([-2.3, 2.2, 0.43], object_orientation)], ["left"]).resolve().perform()
    # MoveTorsoAction([0.25]).resolve().perform()
    # ParkArmsAction([Arms.LEFT]).resolve().perform()
    # #todo parkarms does not move the torso atm
    # NavigateAction(target_locations=[Pose([-2.7, 0, 0.43], object_orientation)]).resolve().perform()

# rospy.loginfo("milk pose: {}".format(milk_desig.pose))
#
# cereal_desig = move_and_detect(obj=cereal)
# rospy.loginfo("cereal pose: {}".format(cereal_desig.pose))
#
# milk_desig = move_and_detect(ObjectType.MILK)
#
# TransportAction(milk_desig, ["left"], [Pose([4.8, 3.55, 0.8])]).resolve().perform()
#
# cereal_desig = move_and_detect(ObjectType.BREAKFAST_CEREAL)
#
# TransportAction(cereal_desig, ["right"], [Pose([5.2, 3.4, 0.8], [0, 0, 1, 1])]).resolve().perform()
#
# bowl_desig = move_and_detect(ObjectType.BOWL)
#
# TransportAction(bowl_desig, ["left"], [Pose([5, 3.3, 0.8], [0, 0, 1, 1])]).resolve().perform()
#
# # Finding and navigating to the drawer holding the spoon
# handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig.resolve())
# drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(), robot_desig=robot_desig.resolve()).resolve()
#
# NavigateAction([drawer_open_loc.pose]).resolve().perform()
#
# OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
# spoon.detach(apartment)
#
# # Detect and pickup the spoon
# LookAtAction([apartment.get_link_pose("handle_cab10_t")]).resolve().perform()
#
# spoon_desig = DetectAction(BelieveObject(types=[ObjectType.SPOON])).resolve().perform()
#
# pickup_arm = "left" if drawer_open_loc.arms[0] == "right" else "right"
# PickUpAction(spoon_desig, [pickup_arm], ["top"]).resolve().perform()
#
# ParkArmsAction([Arms.BOTH]).resolve().perform()
#
# close_loc = drawer_open_loc.pose
# close_loc.position.y += 0.1
# NavigateAction([close_loc]).resolve().perform()
#
# CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
#
# ParkArmsAction([Arms.BOTH]).resolve().perform()
#
# MoveTorsoAction([0.15]).resolve().perform()
#
# # Find a pose to place the spoon, move and then place it
# spoon_target_pose = Pose([4.85, 3.3, 0.8], [0, 0, 1, 1])
# placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve()).resolve()
#
# NavigateAction([placing_loc.pose]).resolve().perform()
#
# PlaceAction(spoon_desig, [spoon_target_pose], [pickup_arm]).resolve().perform()
#
# ParkArmsAction([Arms.BOTH]).resolve().perform()
