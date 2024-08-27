# from pycram.designators.motion_designator import TalkingMotion
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA

from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
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
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "../../resources/after_robocup.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
extension = ObjectDescription.get_file_extension()

robot = Object("hsrb", ObjectType.ROBOT, f"hsrb{extension}", pose=Pose([1, 2, 0]))
color = Color().from_list([0.9, 0.5, 0.9, 1])
robot.set_color(rgba_color=color)

# robot.set_color(0.5, 0.5, 0.9, 1)
robot_desig = ObjectDesignatorDescription(names=["hsrb"])
table_pose = Pose([6.6, 4.9, 0.0], [0.0, 0.0, 0, 1])
long_table_1 = Pose([6.65, 4.6, 0], [0, 0, 0, 1])
long_table_pick = Pose([6.60, 4.6, 0], [0, 0, 0, 1])

long_table_1_rotated = Pose([6.65, 4.6, 0], [0, 0, 1, 0])
shelf_1 = Pose([6.2, 5.6, 0], [0, 0, 1, 0])
shelf_1_rotated1 = Pose([6.2, 5.6, 0], [0, 0, -0.7, 0.7])
shelf_1_rotated = Pose([6.2, 5.6, 0], [0, 0, 0, 1])

milk = Object("milk", ObjectType.MILK, "../../resources/milk.stl", pose=Pose([7.6, 5.2, 0.87]), color=[1, 0, 0, 0.95])
cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "../../resources/breakfast_cereal.stl",
                pose=Pose([2.5, 2.3, 0.95]), color=[0, 1, 0, 1])
spoon = Object("spoon", ObjectType.SPOON, "../../resources/spoon.stl", pose=Pose([2.4, 2.2, 0.85]),
               color=[0, 0, 1, 0.95])
bowl = Object("bowl", ObjectType.BOWL, "../../resources/bowl.stl", pose=Pose([2.5, 2.2, 1.02]), color=[1, 1, 0, 0.95])
human_female = Object("human_female", ObjectType.HUMAN, "../../resources/female_standing.stl", pose=Pose([3, 3.8, 0]),
                      color=[1, 1, 0, 1.05])
world.simulate(seconds=1)
RobotStateUpdater("/tf", "/giskard_joint_states")
KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

# world.simulate()

@giskardpy.init_giskard_interface
def move_to_table(pose1):
    with semi_real_robot:
        NavigateAction([Pose([2, 2, 0])]).resolve().perform()


def test_move():
    pose1 = Pose([3, 3, 0])
    move_to_table(pose1)


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
    print(kitchen.links)
    sem = SemanticCostmap(kitchen_desig.resolve().world_object, "dinner_table:dinner_table:table_center")
    sem.visualize()


def test_pick_up():
    milk_desig = ObjectDesignatorDescription(names=["milk"])
    tf_link = kitchen.get_link_tf_frame("dinner_table:dinner_table:table_center")
    pick_up(milk_desig.resolve(),tf_link )


# pick_up(milk, "hand_palm_link")
def pick_up(obj_desig, tf_link):
    """
    :param obj_desig:
    :param tf_link:  environment_raw.get_link_tf_frame(link) -> use this to deliver the link
    :return:
    """
    lt = LocalTransformer()

    obj_pose = obj_desig.pose
    # transforms the object pose to the link frame
    print(obj_pose)
    oTb = lt.transform_pose(obj_pose, tf_link)
    object_name = obj_desig.name

    grasp_set = None
    # if to far behind the front face were robots look on the table
    # this is x since we pick up from X of the tf_link so we can do advanged on this by checking the x value
    # so basicly if the object sis to far rotated we grasp from top since hsr is will not be able to grasp it
    # from front
    if oTb.pose.position.x >= 0.20:
        grasp_set = Grasp.TOP

    angle = Pose.quaternion_to_angle(oTb.pose.orientation)
    obj_desig.world_object.get_object_dimensions()
    #print(obj_desig.get_object_dimensions())
    object_dim = obj_desig.world_object.get_object_dimensions()
    print(f"obj dim von {object_name} {object_dim}")


    if grasp_set:
        grasp = Grasp.TOP
    else:
        if object_dim[2] < 0.055:
            rospy.logwarn(f"{object_name} grasp is set to top, angle: {angle}")
            rospy.logwarn(f"{object_name} and height {object_dim[2]}")
            rospy.logwarn(f"{object_name} and width {object_dim[0]}")
            grasp = Grasp.TOP
        elif object_dim[2] < 0.065 or angle > 40 and (object_dim[0] > 0.075 and object_dim[1] > 0.075):
            rospy.logwarn(f"{object_name} grasp is set to top, angle: {angle}")
            rospy.logwarn(f"{object_name} and height {object_dim[2]}")
            rospy.logwarn(f"{object_name} and width {object_dim[0]}")
            grasp = Grasp.FRONT
        else:
            rospy.logwarn(f"{object_name} grasp is set to front, angle: {angle}")
            rospy.logwarn(f"{object_name} and height {object_dim[2]}")
            rospy.logwarn(f"{object_name} and width {object_dim[0]}")
            grasp = Grasp.FRONT

        if grasp == Grasp.TOP:
            print("pose adjusted with z")
            oTb.pose.position.z += (object_dim[2] / 10)
            if object_dim[2] < 0.02:
                rospy.logwarn(f"I am not able to grasp the object: {object_name} please help me!")
                oTb.pose.position.z = 0.011
        else:
            oTb.pose.position.x += 0.03

        grasp_rotation = RobotDescription.current_robot_description.grasps[grasp]
        if grasp == Grasp.TOP:
            grasp_q = Quaternion(grasp_rotation[0], grasp_rotation[1], grasp_rotation[2], grasp_rotation[3])
            oTb.multiply_quaternions(grasp_rotation)
        else:
            oTb.orientation = grasp_rotation

        oTmG = lt.transform_pose(oTb, "map")
        after_pose = oTmG.copy()
        after_pose.pose.position.z += 0.02

        World.current_world.add_vis_axis(oTmG)

        #this if for preposes for the robot in this case the HSR, so giskard has no troubles
        if grasp == "front":
            config_for_placing = {'arm_lift_joint': -1, 'arm_flex_joint': -0.16, 'arm_roll_joint': -0.0145,
                                  'wrist_flex_joint': -1.417, 'wrist_roll_joint': 0.0}
        else:
            config_for_placing = {'arm_flex_joint': -1.1, 'arm_lift_joint': 1.15, 'arm_roll_joint': 0,
                                  'wrist_flex_joint': -1.6, 'wrist_roll_joint': 0, }

        MoveArmJointsMotion(left_arm_poses=config_for_placing)
        MoveGripperMotion(gripper=Arms.LEFT, motion=GripperState.OPEN, allow_gripper_collision=False).perform()
        TalkingMotion(f"Pick Up now! {object_name.split('_')[0]} from: {grasp}").perform()
        MoveTCPMotion(arm=Arms.LEFT, target=oTmG, allow_gripper_collision=False).perform()
        giskard_return = giskardpy.achieve_sequence_pick_up(oTmG)
        while not giskard_return:
            rospy.sleep(0.1)

        giskardpy.achieve_attached(object)
        tip_link = 'hand_gripper_tool_frame'
        robot.attach(object=object, link=tip_link)
        BulletWorld.robot.attach(object=object, link=tip_link)
        MoveGripperMotion("close").perform()
        giskardpy.avoid_all_collisions()
        park = pakerino()
        while not park:
            print("waiting for park")
            rospy.sleep(0.1)
        grasped_bool = None
        if grasp_listener.check_grasp():
            talk.pub_now("Grasped a object")
            grasped_bool = True
        else:
            talk.pub_now("I was not able to grasped a object")
            grasped_bool = False

        return grasped_bool, grasp, found_object
