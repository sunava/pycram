from pycram.context_knowledge import generate_context
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper
from pycram.ros.viz_marker_publisher import VizMarkerPublisher

world = BulletWorld("DIRECT")
VizMarkerPublisher(interval=0.6)
current_context = generate_context("cutting-init", "apartment-small.urdf")
cutting_tool = current_context.get_cutting_tool()
with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    MoveTorsoAction([0.33]).resolve().perform()
    TransportAction(current_context=current_context, hold=True, target_object=cutting_tool.name).resolve().perform()
    #     grasp = robot_description.grasps.get_orientation_for_grasp("top")
#     arm = "left"
#     pickup_pose_knife = CostmapLocation(target=bigknife_BO.resolve(), reachable_for=robot_desig).resolve()
#     pickup_arm = pickup_pose_knife.reachable_arms[0]
#     NavigateAction(target_locations=[pickup_pose_knife.pose]).resolve().perform()
#     PickUpAction(object_designator_description=bigknife_BO,
#                  arms=["left"],
#                  grasps=["top"]).resolve().perform()
#
#     ParkArmsAction([Arms.BOTH]).resolve().perform()
#     original_quaternion = (0, 0, 0, 1)
#     rotation_axis = (0, 0, 1)
#     rotation_quaternion = helper.axis_angle_to_quaternion(rotation_axis, 180)
#     resulting_quaternion = helper.multiply_quaternions(original_quaternion, rotation_quaternion)
#     nav_pose = Pose([-0.3, 0.9, 0.0], resulting_quaternion)
#     NavigateAction(target_locations=[nav_pose]).resolve().perform()
#     LookAtAction(targets=[cucumber_BO.resolve().pose]).resolve().perform()
#
#     object_desig = DetectAction(technique='all').resolve().perform()
#     object_dict = object_desig[1]
#     for key, value in object_dict.items():
#     #detected_desig = DetectAction(cucumber_BO).resolve().perform()
#         if object_dict[key].type == "cucumber":
#             CuttingAction(cucumber_BO, bigknife_BO, ["left"], "slicing").resolve().perform()
#
#     # CuttingActionSPARQL(object_designator_description=bread_BO,
#     #              arms=["left"],
#     #              grasps=["top"]).resolve().perform()
