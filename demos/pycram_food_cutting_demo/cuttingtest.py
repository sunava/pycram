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

# all available parameters
tasks = [('Select',None),('Cutting Action', "Cutting Action"), ('Sawing', "Sawing"), ('Paring', "Paring"), ('Cutting', "Cutting"),
         ('Carving', "Carving"), ('Slicing', "Slicing"), ('Snipping', "Snipping"), ('Slivering', "Slivering"),
         ('Halving', "Halving"), ('Quartering (Unavailable)', None), ('Julienning (Unavailable)', None),
         ('Dicing (Unavailable)', None), ('Mincing (Unavailable)', None), ('Cubing (Unavailable)', None),
         ('Cubing (Unavailable)', None)]

objects = [('Select', None), ('cucumber', "cucumber.stl"), ('strawberry', "strawberry.stl"), ('pumpkin', "pumpkin.stl"),
    ('banana', "banana.stl"), ('apple', "apple.stl"), ('citron', "citron.stl"),
    ('orange', "orange.stl"),('coconut (Unavailable)', None), ('pineapple (Unavailable)', None),
    ('pepper (Unavailable)', None), ('squash (Unavailable)', None), ('lime (Unavailable)', None),
    ('cherry (Unavailable)', None), ('bean (Unavailable)', None), ('lemon (Unavailable)', None),
    ('kumquat (Unavailable)', None), ('olive (Unavailable)', None)]

task = ""
obj = ""

selected_task = None
selected_obj = None


def update_globals(task=None, obj=None):
    global selected_task, selected_obj
    if task is not None:
        selected_task = task
    if obj is not None:
        selected_obj = obj


def robot_execute():
    global selected_task, selected_obj
    with output:
        output.clear_output()
        cutting_simple(selected_obj, selected_task)

    output.clear_output()


def setup_task_object_widgets():
    context_dropdown = widgets.Dropdown(options=tasks, description='task:')
    environment_dropdown = widgets.Dropdown(options=objects, description='object:')

    context_dropdown.observe(lambda change: update_globals(task=change['new']), names='value')
    environment_dropdown.observe(lambda change: update_globals(obj=change['new']), names='value')

    display(HBox([context_dropdown, environment_dropdown]))


def start_demo():
    global output
    output = Output()
    setup_task_object_widgets()
    execute_button = Button(description="Execute Task")
    # Use a lambda function to defer the call to `robot_execute`
    # In this lambda function, lambda x: robot_execute(func),
    # x represents the button click event (which we don't use here),
    # and robot_execute(func) is the function call you want to happen when the button is clicked.
    # This way, robot_execute will only be executed when the button is clicked, not when start_demo is called.
    execute_button.on_click(lambda x: robot_execute())
    display(execute_button, output)


def cutting_simple(obj="cucumber.stl", technique="slicing", color=[0, 1, 0.04, 1]):
    world = BulletWorld()
    kitchen = Object("environment", ObjectType.ENVIRONMENT, "kitchen-small.urdf")
    kitchen.set_color([0.5, 0.5, 0.5, 0.8])
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
    cucumber.set_color(color)
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
        pickup_pose_knife = CostmapLocation(target=bigknife_BO.resolve(), reachable_for=robot_desig).resolve()
        pickup_arm = pickup_pose_knife.reachable_arms[0]
        NavigateAction(target_locations=[pickup_pose_knife.pose]).resolve().perform()
        PickUpAction(object_designator_description=bigknife_BO, arms=["left"], grasps=["top"]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()
        original_quaternion = (0, 0, 0, 1)
        rotation_axis = (0, 0, 1)
        rotation_quaternion = helper.axis_angle_to_quaternion(rotation_axis, 180)
        resulting_quaternion = helper.multiply_quaternions(original_quaternion, rotation_quaternion)
        nav_pose = Pose([-0.3, 0.9, 0.0], resulting_quaternion)
        NavigateAction(target_locations=[nav_pose]).resolve().perform()
        LookAtAction(targets=[cucumber_BO.resolve().pose]).resolve().perform()

        CuttingAction(perceived_cucumber, bigknife_BO.resolve(), ["left"], technique).resolve().perform()
        rospy.loginfo("Done with the task")

colors = {
    "Crimson": (0.86, 0.08, 0.24, 1),
    "Cerulean": (0, 0.48, 0.65, 1),
    "Amber": (1, 0.75, 0, 1),
    "Teal": (0, 0.5, 0.5, 1),
    "Lavender": (0.9, 0.9, 0.98, 1),
    "Saffron": (0.96, 0.77, 0.19, 1),
    "Charcoal": (0.21, 0.27, 0.31, 1),
    "Coral": (1, 0.5, 0.31, 1),
    "Turquoise": (0.25, 0.88, 0.82, 1),
    "Mauve": (0.88, 0.69, 1, 1)
}

cutting_simple("/objects/tomato.stl", colors["Amber"])