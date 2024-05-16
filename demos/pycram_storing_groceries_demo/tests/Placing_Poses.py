import rospy

import pycram.external_interfaces.giskard as giskardpy
from demos.pycram_storing_groceries_demo.utils.misc import *
from pycram.designators.action_designator import *
from pycram.process_module import real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from docutils.nodes import math
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.designators.object_designator import *



world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "couch-whole_kitchen2.urdf")
giskardpy.init_giskard_interface()
giskardpy.sync_worlds()
RobotStateUpdater("/tf", "/giskard_joint_states")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
wished_sorted_obj_list = ["Metalbowl", "Cerealbox", "Milkpack", "Spoon", "Chips"]



def pre_place():
    MoveJointsMotion(["arm_lift_joint"], [0.2]).resolve().perform()
    MoveJointsMotion(["wrist_roll_joint"], [0.12]).resolve().perform()
    MoveJointsMotion(["arm_roll_joint"], [0.0]).resolve().perform()
    MoveJointsMotion(["wrist_flex_joint"], [-1.0]).resolve().perform()
    MoveJointsMotion(["arm_flex_joint"], [-0.3]).resolve().perform()

with real_robot:

    pre_place()
    # TalkingMotion("starting test").resolve().perform()

    # rospy.sleep(1.5)
    # pose2 = Pose([2.23, 1.96, 0.0], [0, 0, 0.68, 0.731])
    # NavigateAction([pose2]).resolve().perform()

    print("placing bottom shelf")
    # TODO: Move Torso up/down
    MoveJointsMotion(["arm_lift_joint"], [0.09]).resolve().perform()
    #
    # # TODO: Wrist Up/Down
    MoveJointsMotion(["wrist_flex_joint"], [0.12]).resolve().perform()

    print(robot.get_complete_joint_state())
    #
    # # TODO: Arm heben/senken
    MoveJointsMotion(["arm_flex_joint"], [-2.18]).resolve().perform()

    # TODO: Wrist left/right side turn
    # MoveJointsMotion(["wrist_roll_joint"], [-1.6]).resolve().perform()

    MoveJointsMotion(["wrist_flex_joint"], [0.5]).resolve().perform()

    print("placing middle shelf")

    MoveJointsMotion(["arm_lift_joint"], [0.42]).resolve().perform()

    MoveJointsMotion(["wrist_flex_joint"], [0.5]).resolve().perform()

    # TODO: Collision deactivate
    # pose2 = Pose([2.33, 2.6, 0.0], [0, 0, 0.68, 0.731])
    # NavigateAction([pose2]).resolve().perform()



