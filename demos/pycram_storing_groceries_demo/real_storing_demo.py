from demos.pycram_storing_groceries_demo.utils.misc import *
from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.designators.action_designator import *
from pycram.enums import ObjectType
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
import pycram.external_interfaces.robokudo as robokudo
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, StartSignalWaiter

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

text_to_speech_publisher = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()
start_signal_waiter = StartSignalWaiter()
move = PoseNavigator()

robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])

RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

giskardpy.init_giskard_interface()
#robokudo.init_robokudo_interface()
#rospy.sleep(2)
# giskardpy.spawn_kitchen()

# Shelf Variables
shelf_pose = Pose([4.417880842356951,4.913135736923778, 0] ,[0, 0, 0, 1])
lower_compartment = ShelfCompartmentDescription(height=0.09, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
middle_compartment = ShelfCompartmentDescription(height=0.37, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
upper_compartment = ShelfCompartmentDescription(height=0.71, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
shelves = [lower_compartment, middle_compartment, upper_compartment]


def demo(step):
    with real_robot:
        # Wait for the start signal
        if step <= 0:
            start_signal_waiter.wait_for_startsignal()

            # continue with the rest of the script
            rospy.loginfo("Start signal received, now proceeding with tasks.")

        # move to shelf
        if step <= 1:
            move.query_pose_nav(shelf_pose)

        # perceive shelf
        if step <= 2:
            # TalkingMotion("start").resolve().perform()
            shelf_obj = DetectAction(technique='all').resolve().perform()

            res = sort_new(shelf_obj, robot)
            # object sorted by distance to robot
            sorted_objects = res[0]
            print("sorted obj: " + str(sorted_objects))

            # dict with object poses
            object_list_poses = res[1]
            print("pose list: " + str(object_list_poses))

            # update perceived info to shelves
            order_items_to_shelves(object_list_poses, shelves)

            # TODO: next steps

            # navigate to table and perceive

            # while objects on table:

                 # pick up nearest object

                 # navigate to shelf

                 # place object depending on shelf areas

            # end?







demo(2)