import rospy
from tmc_msgs.msg import Voice

from pycram import pose
from pycram.external_interfaces import robokudo
from pycram.process_module import real_robot, semi_real_robot, with_real_robot
from pycram.designators.action_designator import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

world.set_gravity([0, 0, -9.8])

robo_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
x = robo_orientation[0]
y = robo_orientation[1]
z = robo_orientation[2]
w = robo_orientation[3]

cereal_orientation = axis_angle_to_quaternion([0, 0, 1], 90)

# Initialize objects in BulletWorld
# TODO: Warum ist die Orientierung vom falschen Datentyp? Kann ich das ignorieren?
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()

kitchen = Object("kitchen", "environment", "kitchen.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

milk = Object("milk", "milk", "milk.stl", pose=Pose([-2.7, 2.3, 0.43]), color=[1, 0, 0, 1])
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([-2.5, 2.3, 0.43], cereal_orientation),
                color=[0, 1, 0, 1])
# cereal2 = Object("cereal2", "cereal", "breakfast_cereal.stl", pose=Pose([-2.9, 2.3, 0.43], cereal_orientation), color=[0, 1, 0, 1])
# milk2 = Object("milk2", "milk", "milk.stl", pose=Pose([-2.3, 2.3, 0.43]), color=[0, 1, 0, 1])

milk_desig = BelieveObject(types=["milk"])
cereal_desig = BelieveObject(types=["cereal"])

pub = rospy.Publisher('/talk_request', Voice, queue_size=10)
drop_str = Voice()
drop_str.language = 1
drop_str.sentence = "I drop the object now!"

# Giskard initialisieren und syncen
giskardpy.init_giskard_interface()
giskardpy.sync_worlds()

RobotStateUpdater("/tf", "/joint_states")

# @with_real_robot
# def move_and_detect(obj_type):
#     NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()
#
#     LookAtAction(targets=[pick_pose]).resolve().perform()
#
#     object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()
#
#     return object_desig

with semi_real_robot:
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    LookAtAction(targets=[Pose([-2.6, 2.0, 0])]).resolve().perform()

    NavigateAction(target_locations=[Pose([-2.6, 0, 0], robo_orientation)]).resolve().perform()
    NavigateAction(target_locations=[Pose([-2.6, 1.5, 0], robo_orientation)]).resolve().perform()

    pick_pose = Pose([-2.6, 2.0, 0.8])
    LookAtAction(targets=[pick_pose]).resolve().perform()
    object_desig = DetectAction(milk_desig).resolve().perform()
   
    PickUpAction(object_designator_description=object_desig,
                 arms=["left"],
                 grasps=["front"]).resolve().perform()
    print(object_desig.pose)

    pub.publish(drop_str)
    new_pose = object_desig.pose
    place_pose = Pose([new_pose.position.x, new_pose.position.y + 0.3, new_pose.position.z])
    PlaceAction(object_desig, [place_pose], ["left"]).resolve().perform()
    print("place finished")
# MoveGripperMotion("open", "left", allow_gripper_collision=True).resolve().perform()
