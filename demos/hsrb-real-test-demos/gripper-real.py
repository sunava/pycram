from tmc_control_msgs.msg import GripperApplyEffortActionGoal

from pycram.designators.action_designator import *

def open_gripper():
    """ Opens the gripper of the HSR """
    pub_gripper = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                  queue_size=10)
    rate = rospy.Rate(10)
    rospy.sleep(2)
    msg = GripperApplyEffortActionGoal()  # sprechen joint gripper_controll_manager an, indem wir goal publishen type den giskard fürs greifen erwartet
    msg.goal.effort = 0.8
    pub_gripper.publish(msg)


def close_gripper():
    """ Closes the gripper of the HSR """
    pub_gripper = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                  queue_size=10)
    rate = rospy.Rate(10)
    rospy.sleep(2)
    msg = GripperApplyEffortActionGoal()
    msg.goal.effort = -0.8
    pub_gripper.publish(msg)