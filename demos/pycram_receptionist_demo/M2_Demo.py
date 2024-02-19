import rospy
from pycram.designators.action_designator import DetectAction, NavigateAction
from pycram.designators.motion_designator import TalkingMotion
from pycram.process_module import real_robot
from demos.pycram_receptionist_demo.utils.misc import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.external_interfaces.knowrob import instances_of, get_guest_info
from std_msgs.msg import String, Bool
from pycram.helper import axis_angle_to_quaternion

world = BulletWorld("DIRECT")
# /pycram/viz_marker topic bei Marker Array
v = VizMarkerPublisher()

world.set_gravity([0, 0, -9.8])
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])
kitchen = Object("kitchen", "environment", "kitchen.urdf")
robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
milk = Object("Milkpack", "milk", "milk.stl", pose=Pose([-2.7, 2.3, 0.43]), color=[1, 0, 0, 1])

giskardpy.init_giskard_interface()


# giskardpy.sync_worlds()
# RobotStateUpdater("/tf", "/joint_states")


with real_robot:

    # Variables
    robot_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
    pose_kitchen_to_couch = Pose([4.2, 3, 0], robot_orientation)
    robot_orientation_couch = axis_angle_to_quaternion([0, 0, 1], 0)
    pose_couch = Pose([3, 5, 0], robot_orientation_couch)
    host = HumanDescription("Bob", fav_drink="Coffee")
    guest1 = HumanDescription("x")
    pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)

    # Perception, detect first guest
    DetectAction(technique='human', state='start').resolve().perform()
    while not guest1.human_pose:
        TalkingMotion("Please step in front of me").resolve.perform()
        rospy.sleep(5)

    rospy.loginfo("human detected")

    # look at guest and introduction
    giskardpy.move_head_to_human()
    TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()

    # reicht sleep 1?
    rospy.sleep(1)

    # signal to start listening
    pub_nlp.publish("start listening")
    rospy.sleep(10)

    # get data from Knowledge
    # TODO: test on real HSR
    guest_data = get_guest_info(1)
    while guest_data == "No name saved under this ID!":
        talk_error("no name")
        guest_data = get_guest_info(1)
        rospy.sleep(3)

    guest1.set_name(guest_data[0])
    guest1.set_drink(guest_data[1])
    talk_request(guest_data)

    # lead guest to living room
    giskardpy.stop_looking()
    NavigateAction([pose_kitchen_to_couch]).resolve().perform()
    NavigateAction([pose_couch]).resolve().perform()

    # search for host in living room
    if DetectAction(technique='human', state='start').resolve().perform():
        # look at host
        giskardpy.move_head_to_human()

        # introduce guest and host
        introduce(host.name, host.fav_drink, guest1.name, guest1.fav_drink)
        giskardpy.stop_looking()










