import ipywidgets as widgets
from ipywidgets import HBox, Button, Output
from IPython.display import display

from demos.pycram_virtual_building_demos.cutting_actions.cutting_demos.cutting_task import start_cutting


objects=[
('apple', "obo:FOODON_03301710"),
('avocado', "obo:FOODON_00003600"),
('banana', "obo:FOODON_00004183"),
('citron', "obo:FOODON_03306596"),
('cucumber', "obo:FOODON_00003415"),
('kiwi', "obo:FOODON_00004387"),
('lemon', "obo:FOODON_03301441"),
('lime', "obo:FOODON_00003661"),
('orange', "obo:FOODON_03309832"),
('peach', "obo:FOODON_03315502"),
('tomato', "obo:FOODON_03309927")]

# all available parameters
tasks = [
         ('Cutting',"cut:CuttingAction"),
        ('Quartering', "cut:Quartering"),
        ('Halving',"cut:Halving"),
        ('Cutting',"soma:Cutting"),
        ('Slicing',"soma:Slicing"),
        ('Snipping',"cut:Snipping"),
        ('Slivering',"cut:Slivering"),
        ('Sawing',"cut:Sawing"),
        ('Paring',"cut:Paring"),
        ('Carving',"cut:Carving")]
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
        start_cutting(selected_obj, selected_task)

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
    execute_button = Button(description="Start Demo")
    # Use a lambda function to defer the call to `robot_execute`
    # In this lambda function, lambda x: robot_execute(func),
    # x represents the button click event (which we don't use here),
    # and robot_execute(func) is the function call you want to happen when the button is clicked.
    # This way, robot_execute will only be executed when the button is clicked, not when start_demo is called.
    execute_button.on_click(lambda x: robot_execute())
    display(execute_button, output)

