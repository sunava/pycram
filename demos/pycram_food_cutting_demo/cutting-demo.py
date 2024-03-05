from pycram.context_knowledge import generate_context
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.ros.tf_broadcaster import TFBroadcaster
import os
import rospkg



world = BulletWorld()
VizMarkerPublisher(interval=0.8)



# Initialize a ROS package object
rospack = rospkg.RosPack()

# Get the path to the specific ROS package
# Replace 'your_package_name' with the name of your ROS package



current_context = generate_context("cutting-init", "apartment-small.urdf")
cutting_tool = current_context.get_cutting_tool()

cutting_obj = current_context.get_cutting_objects()
name = "apartment-small.urdf"

package_path = rospack.get_path('pycram') + '/resources/' + name
urdf_string = helper.urdf_to_string(package_path)
rospy.set_param('kitchen_description', urdf_string)
broadcaster = TFBroadcaster(interval=0.0002)
robot_desig = BulletWorld.current_bullet_world.robot


with simulated_robot:
    # ParkArmsAction([Arms.BOTH]).resolve().perform()
    # MoveTorsoAction([0.33]).resolve().perform()
    # TransportAction(current_context=current_context, hold=True, target_object=cutting_tool.name).resolve().perform()
    location_pose = Pose([1.7, 2, 0])
    looking_pose = Pose([2.5, 2, 0.97])
    NavigateAction([location_pose]).resolve().perform()

    LookAtAction([looking_pose]).resolve().perform()
    status, object_dict = DetectAction(technique='specific', object_type="object_to_be_cut").resolve().perform()
    if status:
        for key, value in object_dict.items():
            detected_object = object_dict[key]
            bigknife_BO = BelieveObject(names=["bigknife"])
            CuttingAction(detected_object, bigknife_BO, ["right"], "slicing").resolve().perform()

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
