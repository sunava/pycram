
import rospy
from robokudo_msgs.msg import QueryActionGoal

from pycram.external_interfaces import giskard


from pycram.process_module import simulated_robot, with_simulated_robot, real_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper

world = BulletWorld()
world.set_gravity([0, 0, -9.8])
robot = Object("pr2", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
kitchen = Object("kitchen", "environment", "kitchen.urdf")
robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
spawning_poses = {
    # 'bigknife': Pose([-0.95, 1.2, 1.3], [1, -1, 1, -1]),
    'bigknife': Pose([0.9, 0.6, 0.8], [0, 0, 0, -1]),
    # 'bread': Pose([-0.85, 0.9, 0.90], [0, 0, -1, 1])
    'bread': Pose([-0.85, 0.9, 0.90], [0, 0, -1, -1]),
    'board': Pose([-0.85, 0.9, 0.85], [0, 0, -1, -1]),
    'cocumber': Pose([-0.85, 0.9, 0.87], [0, 0, -1, -1])
}
bigknife = Object("bigknife", "bigknife", "big-knife.stl", spawning_poses["bigknife"])
cocumber = Object("cocumber", "cocumber", "cocumber.stl", spawning_poses["cocumber"])
board = Object("board", "board", "board.stl", spawning_poses["board"])
cocumber.set_color([0, 1, 0.04, 1])
board.set_color([0.4, 0.2, 0.06, 1])
bigknife_BO = BelieveObject(names=["bigknife"])
bread_BO = BelieveObject(names=["bread"])
cocumber_BO = BelieveObject(names=["cocumber"])

giskard.init_giskard_interface()


with real_robot:
    rospy.loginfo("human detected")
    pub_nlp = rospy.Publisher('/robokudovanessa/query/goal', QueryActionGoal, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    msgs = QueryActionGoal()
    #talk_test.talker("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?")
    rospy.sleep(2)
    pub_nlp.publish(msgs)

    giskard.move_head_to_human()

    #LookAtAction(targets=[cocumber_BO.resolve().pose]).resolve().perform()
