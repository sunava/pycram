import ipywidgets as widgets
import rospy
from IPython.display import display
from ipywidgets import HBox
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
import ipywidgets as widgets
from ipywidgets import HBox, Button, Output
from IPython.display import display, clear_output, HTML
import rospkg
from pycram.context_knowledge import ContextConfig, generate_context


def cutting_simple(obj="cucumber.stl", technique="slicing"):
    world = BulletWorld()
    #kitchen = Object("environment", ObjectType.ENVIRONMENT, "apartment-small.urdf")
    #kitchen.set_color([0.5, 0.5, 0.5, 0.8])
    #    name = "kitchen-small.urdf"

    #   package_path = rospack.get_path('pycram') + '/resources/' + name
    #  urdf_string = helper.urdf_to_string(package_path)

    robot = Object("pr2", "robot", "../../resources/" + robot_description.name + ".urdf")
    robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()

    # rospy.set_param('envi_description', kitchen.urdf_object)
    robot.set_joint_state(robot_description.torso_joint, 0.24)
    kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

    VizMarkerPublisher(interval=0.05)

    # Initialize a ROS package object

    # #name = "apartment-small.urdf"
    # rospack = rospkg.RosPack()
    # package_path = rospack.get_path('pycram') + '/resources/' + name
    # urdf_string = helper.urdf_to_string(package_path)
    # rospy.set_param('kitchen_description', urdf_string)

    # broadcaster = TFBroadcaster(interval=0.0002)
    # viz = VizMarkerPublisher()
    spawning_poses = {# 'bigknife': Pose([-0.95, 1.2, 1.3], [1, -1, 1, -1]),
        'bigknife': Pose([0.9, 0.6, 0.8], [0, 0, 0, -1]), # 'bread': Pose([-0.85, 0.9, 0.90], [0, 0, -1, 1])
        'bread': Pose([-0.85, 0.9, 0.90], [0, 0, -1, -1]), 'board': Pose([-0.85, 0.9, 0.85], [0, 0, -1, -1]),
        'cucumber': Pose([-0.85, 0.9, 0.87], [0, 0, -1, -1])}
    bigknife = Object("bigknife", "bigknife", "big-knife.stl", spawning_poses["bigknife"])
    cucumber = Object("cucumber", "cucumber", obj, spawning_poses["cucumber"])
    perceived_cucumber = ObjectDesignatorDescription.Object(cucumber.name, cucumber.type, cucumber)
    board = Object("board", "board", "board.stl", spawning_poses["board"])
    cucumber.set_color([0, 1, 0.04, 1])
    board.set_color([0.4, 0.2, 0.06, 1])
    bigknife.set_color([0.5, 0.5, 0.5, 1])
    bigknife_BO = BelieveObject(names=["bigknife"])
    bread_BO = BelieveObject(names=["bread"])
    cucumber_BO = BelieveObject(names=["cucumber"])
    rospy.loginfo("quering the KB")
    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([0.33]).resolve().perform()
        grasp = robot_description.grasps.get_orientation_for_grasp("top")
        arm = "left"
        #pickup_pose_knife = CostmapLocation(target=bigknife_BO.resolve(), reachable_for=robot_desig).resolve()
        #pickup_arm = pickup_pose_knife.reachable_arms[0]
        #NavigateAction(target_locations=[pickup_pose_knife.pose]).resolve().perform()
        #PickUpAction(object_designator_description=bigknife_BO, arms=["left"], grasps=["top"]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()
        original_quaternion = (0, 0, 0, 1)
        rotation_axis = (0, 0, 1)
        rotation_quaternion = helper.axis_angle_to_quaternion(rotation_axis, 180)
        x = rotation_quaternion[0]
        y = rotation_quaternion[1]
        z = rotation_quaternion[2]
        w = rotation_quaternion[3]

        q = [x, y, z, w]
        resulting_quaternion = helper.multiply_quaternions(original_quaternion, rotation_quaternion)
        nav_pose = Pose([-0.3, 0.9, 0.0], resulting_quaternion)
        NavigateAction(target_locations=[nav_pose]).resolve().perform()
        LookAtAction(targets=[cucumber_BO.resolve().pose]).resolve().perform()

        CuttingAction(perceived_cucumber, bigknife_BO, ["left"], technique).resolve().perform()
        rospy.loginfo("Done with the task")
cutting_simple()