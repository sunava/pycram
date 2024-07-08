import rospy

import pycram.external_interfaces.giskard_new as giskardpy
from pycram.bullet_world import BulletWorld, Object

from pycram.local_transformer import LocalTransformer
from pycram.ros.robot_state_updater import RobotStateUpdater

from pycram.utilities.robocup_utils import pakerino

giskardpy.init_giskard_interface()
giskardpy.clear()
config_for_placing = {
    'arm_flex_joint': 0,
    'arm_lift_joint': 0.15,
    'arm_roll_joint': 0,
    # 'head_pan_joint': 0.1016423434842566,
    # 'head_tilt_joint': 0.0255536193297764,
    'wrist_flex_joint': -1.7,
    'wrist_roll_joint': 0,
}
world = BulletWorld()
robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")

pakerino(config=config_for_placing)

lt = LocalTransformer()
rospy.sleep(2)
ps = robot.get_link_pose("hand_gripper_tool_frame")
tps = lt.transform_pose(ps, robot.get_link_tf_frame("base_link"))
print(tps)