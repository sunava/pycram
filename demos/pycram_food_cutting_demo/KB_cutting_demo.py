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



objects = [('Select', None), ('cucumber', "cucumber.stl"), ('strawberry', "strawberry.stl"), ('pumpkin', "pumpkin.stl"),
    ('banana', "banana.stl"), ('apple', "apple.stl"), ('citron', "citron.stl"),
    ('orange', "orange.stl"),('coconut (Unavailable)', None), ('pineapple (Unavailable)', None),
    ('pepper (Unavailable)', None), ('squash (Unavailable)', None), ('lime (Unavailable)', None),
    ('cherry (Unavailable)', None), ('bean (Unavailable)', None), ('lemon (Unavailable)', None),
    ('kumquat (Unavailable)', None), ('olive (Unavailable)', None)]

# all available parameters
tasks = [('Select',None),
         ('Cutting',"cut:CuttingAction"),
        ('Quartering', "cut:Quartering"),
        ('Julienning (Unavailable)',"cut:Julienning"),
        ('Halving',"cut:Halving"),
        ('Dicing (Unavailable)', None)
        ('Cutting',"soma:Cutting"),
        ('Slicing',"soma:Slicing"),
        ('Snipping',"cut:Snipping"),
        ('Slivering',"cut:Slivering"),
        ('Sawing',"cut:Sawing"),
        ('Paring',"cut:Paring"),
        ('Carving',"cut:Carving"),
        ('Mincing (Unavailable)',None),
        ('Cubing (Unavailable)',None),
        ('Chopping (Unavailable)',None)]
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
        cutting(selected_obj, selected_task)

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


def cutting(obj="cucumber.stl", technique="slicing"):
    world = BulletWorld("DIRECT")
    VizMarkerPublisher(interval=0.1)

    current_context = generate_context("cutting-init", "apartment-small.urdf")
    cutting_tool = current_context.get_cutting_tool()

    cutting_obj = current_context.get_cutting_objects()
    name = "apartment-small.urdf"
    robot_desig = BulletWorld.current_bullet_world.robot

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        MoveTorsoAction([0.33]).resolve().perform()
        TransportAction(current_context=current_context, hold=True, target_object=cutting_tool.name).resolve().perform()
        location_pose = Pose([1.7, 2, 0])
        looking_pose = Pose([2.5, 2, 0.97])
        NavigateAction([location_pose]).resolve().perform()

        LookAtAction([looking_pose]).resolve().perform()
        status, object_dict = DetectAction(technique='specific', object_type="object_to_be_cut").resolve().perform()
        if status:
            for key, value in object_dict.items():
                detected_object = object_dict[key]
                bigknife_BO = BelieveObject(names=["bigknife"]).resolve()
                CuttingAction(detected_object, bigknife_BO, ["right"], "slicing").resolve().perform()