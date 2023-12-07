import rospy
import pycram.external_interfaces.giskard as giskardpy
from pycram.designators.action_designator import *
from pycram.process_module import real_robot

rospy.loginfo("init_interface")
giskardpy.init_giskard_interface()

with real_robot:
    MoveGripperMotion("open", "left", allow_gripper_collision=True).resolve().perform()
    time.sleep(2)
    MoveGripperMotion("neutral", "left", allow_gripper_collision=True).resolve().perform()
    time.sleep(2)
    MoveGripperMotion("close", "left", allow_gripper_collision=True).resolve().perform()
