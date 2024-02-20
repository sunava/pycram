import ipywidgets as widgets
from IPython.display import display
from ipywidgets import HBox
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper
from pycram.ros.viz_marker_publisher import VizMarkerPublisher

# all available parameters
tasks = [('Cutting Action',"cut:CuttingAction"),
        ('Quartering', "cut:Quartering"),
        ('Julienning',"cut:Julienning"),
        ('Halving',"cut:Halving"),
        ('Dicing',"soma:Dicing"),
        ('Cutting',"soma:Cutting"),
        ('Slicing',"soma:Slicing"),
        ('Snipping',"cut:Snipping"),
        ('Slivering',"cut:Slivering"),
        ('Sawing',"cut:Sawing"),
        ('Paring',"cut:Paring"),
        ('Carving',"cut:Carving"),
        ('Mincing',"cut:Mincing"),
        ('Cubing',"cut:Cubing"),
        ('Chopping',"cut:Chopping")]

objects=[('almond', "obo:FOODON_00003523"),
      ('cucumber', "obo:FOODON_00003415"),
      ('strawberry', "obo:FOODON_00003443"),
      ('coconut', "obo:FOODON_00003449"),
      ('pineapple', "obo:FOODON_00003459"),
      ('pumpkin', "obo:FOODON_00003486"),
      ('pepper', "obo:FOODON_00003520"),
      ('squash', "obo:FOODON_00003539"),
      ('lime', "obo:FOODON_00003661"),
      ('banana', "obo:FOODON_00004183"),
      ('cherry', "obo:FOODON_03301240"),
      ('bean', "obo:FOODON_03301403"),
      ('lemon', "obo:FOODON_03301441"),
      ('apple', "obo:FOODON_03301710"),
      ('citron', "obo:FOODON_03306596"),
      ('kumquat', "obo:FOODON_03306597"),
      ('orange', "obo:FOODON_03309832"),
      ('tomato', "obo:FOODON_03309927"),
      ('olive', "obo:FOODON_03317509")]

task=""
tobject=""

def PR2_execute():
    with output:
        output.clear_output()
        print(f"PR2 performing, Task: {selected_task}, on Object: {selected_object}")
        cutting_simple()
def handle_action_change(change):
    if change['new'] == 'PR2':
        PR2_execute()
    # elif change['new'] == 'Pick Up':
    #     pick_up()
    # elif change['new'] == 'Cut':
    #     cut()

def setup_task_object_widgets():
    task_dropdown = widgets.Dropdown(options=tasks, description='Task:')
    object_dropdown = widgets.Dropdown(options=objects, description='Object:')

    def update_selected_task(change):
        global selected_task
        selected_task = change['new']

    def update_selected_object(change):
        global selected_object
        selected_object = change['new']

    task_dropdown.observe(update_selected_task, names='value')
    object_dropdown.observe(update_selected_object, names='value')

    display(HBox([task_dropdown, object_dropdown]))

def setup_widgets():
    global output
    output = widgets.Output()

    action_dropdown = widgets.Dropdown(
        options=['PR2'],
        value=None,
        description='Robot Agent:',
        disabled=False,
    )


    action_dropdown.observe(handle_action_change, names='value')

    setup_task_object_widgets()

    display(action_dropdown, output)

def cutting_simple():
    world = BulletWorld("DIRECT")
    viz = VizMarkerPublisher()
    world.set_gravity([0, 0, -9.8])
    robot = Object("pr2", "robot", "../../resources/" + robot_description.name + ".urdf")
    robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
    kitchen = Object("kitchen", "environment", "kitchen-small.urdf")
    robot.set_joint_state(robot_description.torso_joint, 0.24)
    kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
    spawning_poses = {
        # 'bigknife': Pose([-0.95, 1.2, 1.3], [1, -1, 1, -1]),
        'bigknife': Pose([0.9, 0.6, 0.8], [0, 0, 0, -1]),
        # 'bread': Pose([-0.85, 0.9, 0.90], [0, 0, -1, 1])
        'bread': Pose([-0.85, 0.9, 0.90], [0, 0, -1, -1]),
        'board': Pose([-0.85, 0.9, 0.85], [0, 0, -1, -1]),
        'cucumber': Pose([-0.85, 0.9, 0.87], [0, 0, -1, -1])
    }
    bigknife = Object("bigknife", "bigknife", "big-knife.stl", spawning_poses["bigknife"])
    cucumber = Object("cucumber", "cucumber", "cocumber.stl", spawning_poses["cucumber"])
    board = Object("board", "board", "board.stl", spawning_poses["board"])
    cucumber.set_color([0, 1, 0.04, 1])
    board.set_color([0.4, 0.2, 0.06, 1])
    bigknife_BO = BelieveObject(names=["bigknife"])
    bread_BO = BelieveObject(names=["bread"])
    cucumber_BO = BelieveObject(names=["cucumber"])

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([0.33]).resolve().perform()
        grasp = robot_description.grasps.get_orientation_for_grasp("top")
        arm = "left"
        pickup_pose_knife = CostmapLocation(target=bigknife_BO.resolve(), reachable_for=robot_desig).resolve()
        pickup_arm = pickup_pose_knife.reachable_arms[0]
        NavigateAction(target_locations=[pickup_pose_knife.pose]).resolve().perform()
        PickUpAction(object_designator_description=bigknife_BO,
                     arms=["left"],
                     grasps=["top"]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()
        original_quaternion = (0, 0, 0, 1)
        rotation_axis = (0, 0, 1)
        rotation_quaternion = helper.axis_angle_to_quaternion(rotation_axis, 180)
        resulting_quaternion = helper.multiply_quaternions(original_quaternion, rotation_quaternion)
        nav_pose = Pose([-0.3, 0.9, 0.0], resulting_quaternion)
        NavigateAction(target_locations=[nav_pose]).resolve().perform()
        LookAtAction(targets=[cucumber_BO.resolve().pose]).resolve().perform()

        object_desig = DetectAction(technique='all').resolve().perform()
        object_dict = object_desig[1]
        for key, value in object_dict.items():
            # detected_desig = DetectAction(cucumber_BO).resolve().perform()
            if object_dict[key].type == "cucumber":
                CuttingAction(cucumber_BO, bigknife_BO, ["left"], "slicing").resolve().perform()

        # CuttingActionSPARQL(object_designator_description=bread_BO,
        #              arms=["left"],
        #              grasps=["top"]).resolve().perform()
