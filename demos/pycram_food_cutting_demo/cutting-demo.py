from pycram.context_knowledge import generate_context
from pycram.process_module import simulated_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
from pycram.ros.viz_marker_publisher import VizMarkerPublisher


def cutting_objects(object_to_be_cut="garlic.stl", color=[0.2, 0.8, 0.2, 1], technique="halving",
                    environment="apartment-small.urdf", pose=Pose([2.4, 2, 0.95], [0, 0, -1, -1]),
                    context="cutting-big"):
    world = BulletWorld()
    VizMarkerPublisher(interval=0.8)

    current_context = generate_context(context, environment)
    # cutting_tool = Object("cutting_tool", "cutting_tool", tool, Pose([2.55, 2.15, 0.85])
    cutting_tool = current_context.get_cutting_tool()

    board = Object("board", "board", "board.stl", pose)
    board.set_color([0.4, 0.2, 0.06, 1])

    obj_to_cut = Object("object_to_be_cut", "object_to_be_cut", object_to_be_cut, pose)
    obj_to_cut.set_color(color)
    length, width, height = obj_to_cut.get_object_dimensions()
    obj_to_cut.set_pose(Pose([pose.position.x, pose.position.y, pose.position.z + height / 1.5], pose.orientation))

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
                bigknife_BO = BelieveObject(names=["knife"]).resolve()
                CuttingAction(detected_object, bigknife_BO, ["right"], technique).resolve().perform()
        ParkArmsAction([Arms.BOTH]).resolve().perform()


drawer_island_surface = Pose([2.4, 2, 0.95], [0, 0, -1, -1])
right_to_sink_surface = Pose([2.4, 2.3, 0.95], [0, 0, -1, -1])
colors = {
    "orange": (1, 0.75, 0, 1),
    "cucumber": (0, 1, 0, 1),
    "banana": (1, 1, 0, 1),
    "lemon": (1, 1, 0, 1),
    "lime": (0.75, 1.0, 0.0, 1),
    "apple": (1, 0, 0, 1),
    "tomato": (1, 0, 0, 1),
    "peach": (1.0, 0.8, 0.64, 1),
    "kiwi": (0.76, 0.88, 0.52, 1),
    "avocado": (0.34, 0.51, 0.01, 1),
}

cutting_objects("/objects/avocado.stl", colors["Saffron"], "slicing", "apartment-small.urdf",
                Pose([2.38, 2, 0.95], [0, 0, 1, 1]), context="cutting-small")
