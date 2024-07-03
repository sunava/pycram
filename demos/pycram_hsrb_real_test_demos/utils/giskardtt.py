import rospy

from giskard_msgs.msg import GiskardError
from giskardpy.python_interface.old_python_interface import OldGiskardWrapper
from giskardpy.python_interface.python_interface import GiskardWrapper
from giskardpy.suturo_types import ForceTorqueThresholds, ObjectTypes

rospy.init_node('asdf')
giskard = GiskardWrapper()

# js1 = {
# # 'arm_flex_joint': -0.05,
# 'arm_lift_joint': 0.05,
# 'arm_roll_joint': -1.5,
# # 'head_pan_joint': 0.1016423434842566,
# # 'head_tilt_joint': 0.0255536193297764,
# 'wrist_flex_joint': -1.3,
# 'wrist_roll_joint': 1.7,
# }

js2 = {
'arm_flex_joint': -1,
'arm_lift_joint': 0.25,
'arm_roll_joint': 0,
# 'head_pan_joint': 0.1016423434842566,
# 'head_tilt_joint': 0.0255536193297764,
'wrist_flex_joint': 0,
'wrist_roll_joint': 1.7,
}

js1 = {
'arm_flex_joint': 0,
'arm_lift_joint': 0.15,
'arm_roll_joint': -1.2,
# 'head_pan_joint': 0.1016423434842566,
# 'head_tilt_joint': 0.0255536193297764,
'wrist_flex_joint': -1.5,
'wrist_roll_joint': 0,
}

giskard.motion_goals.add_joint_position(js1)
# js_reached = giskard.monitors.add_joint_position(js1, threshold=0.03)
# giskard.monitors.add_end_motion(start_condition=js_reached)
giskard.add_default_end_motion_conditions()
giskard.execute()
print('done')
#giskard.motion_goals.add_joint_position(js2)
#giskard.add_default_end_motion_conditions()
#giskard.execute()
print('done')