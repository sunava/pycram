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

apartment = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")
# turn robot and some object for 90 degrees
object_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([-2.5, 1.5, 0.43]),
                color=[0, 1, 0, 1])

#
cereal_desig = BelieveObject(names=["cereal"])

robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = BelieveObject(names=["hsr"])
apartment_desig = BelieveObject(names=["kitchen"])

giskardpy.init_giskard_interface()
#todo make a function that removes giskard stuff without bullet
#giskardpy.removing_of_objects()
#giskardpy.sync_worlds()

RobotStateUpdater("/tf", "/joint_states")



with semi_real_robot:
    print("starting mit giskard")
    lt = LocalTransformer()
    #NavigateAction(target_locations=[Pose([0, 0, 0])]).resolve().perform()
    gripper_name = robot_description.get_tool_frame("left")
    posetest = Pose([0, 0, 0], [0, 0, 0, 1])
    gri = BulletWorld.robot.get_link_tf_frame(gripper_name)
    base_name = robot.get_link_tf_frame("base_link")
    while True:
        print(lt.lookupTransform(BulletWorld.robot.get_link_tf_frame(gripper_name),"map", rospy.Time(0)))

    #print(lt.transform_pose(posetest, gri))
    # print(robot.get_link_pose(gripper_name))
    # for i in robot.get_complete_joint_state():
    #     print(i)

    #print(robot.get_complete_joint_state())
    #print(robot.get_joint_state("hand_palm_joint"))
