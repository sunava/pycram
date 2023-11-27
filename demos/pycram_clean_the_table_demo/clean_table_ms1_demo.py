from pycram.process_module import simulated_robot, with_simulated_robot, real_robot, with_real_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper
import pycram.external_interfaces.giskard as giskardpy

from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.external_interfaces import robokudo
from pycram.designators.object_designator import *
from pycram.enums import ObjectType


from pycram.language import macros, par
import sys
world = BulletWorld("DIRECT")
v = VizMarkerPublisher()
# Calculate Quaternion for a 180-degree turn of the robot
robo_orientation = axis_angle_to_quaternion([0, 0, 1], 180)
x = robo_orientation[0]
y = robo_orientation[1]
z = robo_orientation[2]
w = robo_orientation[3]

# Initialize objects in BulletWorld
robot = Object("hsrb", ObjectType.ROBOT, "hsrb.urdf", pose=Pose([0.03, 1.8, 0], robo_orientation))
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
kitchen = Object("kitchen", "environment", "kitchen.urdf")
robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
milk = Object("milk", "milk", "milk.stl", pose=Pose([-0.7841, 2.1089, 0.9]), color=[1, 0, 0, 1])
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([-0.7841, 2.3, 0.9]), color=[0, 1, 0, 1])
cereal2 = Object("cereal2", "cereal", "breakfast_cereal.stl", pose=Pose([-0.7841, 2.6, 0.9]), color=[0, 1, 0, 1])
milk2 = Object("milk2", "milk", "milk.stl", pose=Pose([-0.7841, 1.8, 0.9]), color=[0, 1, 0, 1])

milk_desig = BelieveObject(names=["milk"])
cereal_desig = BelieveObject(names=["cereal"])
cereal2_desig = BelieveObject(names=["cereal2"])
milk2_desig = BelieveObject(names=["milk2"])
object_list = [milk2_desig, milk_desig, cereal_desig, cereal2_desig]
robot_pose_list = [Pose([-0.7841, 1.8, 0.9]), Pose([-0.7841, 2.1089, 0.9]), Pose([-0.7841, 2.3, 0.9]), Pose([-0.7841, 2.6, 0.9])]
# object_type_list = [ObjectType.MILK, ObjectType.MILK, ObjectType.BREAKFAST_CEREAL, ObjectType.BREAKFAST_CEREAL]
# spawning_poses = {
#
# }


#giskardpy.init_giskard_interface()
#giskardpy.sync_worlds()

with simulated_robot:
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    MoveTorsoAction([0.33]).resolve().perform()
   # NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    # dishwasher_desig = ObjectPart(names=["dishwasher"], part_of=kitchen_desig.resolve())
  #  NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform() #dishwasher
    for index in range(len(object_list)):
        #LookAtAction(targets=[robot_pose_list[index]]).resolve().perform()
        #object_desig = DetectAction(BelieveObject(types=[object_type_list[index])).resolve().perform()
        # dann würde hier die Liste der object_list wegfallen
        PickUpAction(object_designator_description=object_list[index],
                     arms=["left"],
                     grasps=["front"]).resolve().perform()
        PlaceAction(object_list[index], [robot_pose_list[index]], ["left"]).resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        NavigateAction(target_locations=[Pose([robot_pose_list[index].pose.x, robot_pose_list[index].pose.y + 0.7, robot_pose_list[index].pose.z])]).resolve().perform()

with real_robot:
    ParkArmsAction([Arms.LEFT]).resolve().perform()
   # LookAtAction(targets=[Pose([-0.7841, 1.8, 0.9]), Pose([-0.7841, 2.1089, 0.9]), Pose([-0.7841, 2.3, 0.9]), Pose([-0.7841, 2.6, 0.9])]).resolve().perform()
    MoveTorsoAction([0.33]).resolve().perform()
    object_desig_desc = ObjectDesignatorDescription(types=["ObjectType.MILK", "ObjectType.MILK", "ObjectType.BREAKFAST_CEREAL", "ObjectType.BREAKFAST_CEREAL"])
    # gibt ein dictionary von Poses/PoseStamped?? zurück
    object_pose_list = robokudo.query(object_desig_desc)
    for index in range(len(object_list)):
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        pickup_pose_object = CostmapLocation(target=object_list[index].resolve(), reachable_for=robot_desig).resolve()
        NavigateAction(target_locations=[pickup_pose_object.pose]).resolve().perform()
        PickUpAction(object_designator_description=object_list[index],
                     arms=["left"],
                     grasps=["front"]).resolve().perform()
        # drop object, vorher textausgabe, mittels subscriber auf /chatter oder /speech?
        robot.set_joint_state(robot_description.hand_motor_joint, 0.1)
        #oder hand_palm_link?
        # wie mit object_pose_list arbeiten





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
