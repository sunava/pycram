from pycram.designators.action_designator import DetectAction, NavigateAction
from pycram.designators.motion_designator import TalkingMotion
from pycram.helper import axis_angle_to_quaternion
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String
import pycram.external_interfaces.navigate as moveBase

world = BulletWorld("DIRECT")
# /pycram/viz_marker topic bei Marker Array
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

# carefull that u spawn the correct kitchen
kitchen = Object("kitchen", "environment", "../../../resources/kitchen.urdf")
giskardpy.init_giskard_interface()
RobotStateUpdater("/tf", "/giskard_joint_states")

robot_orientation_couch = axis_angle_to_quaternion([0, 0, 1], 0)
pose_couch = Pose([3, 5, 0], robot_orientation_couch)

robot_orientation_from_couch = axis_angle_to_quaternion([0, 0, 1], -90)
pose_from_couch = Pose([4.2, 3.8, 0], robot_orientation_from_couch)

robot_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
pose_kitchen_to_couch = Pose([4.2, 3, 0], robot_orientation)

pose_home = Pose([3, 1.7, 0], robot_orientation)

pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)


class HumanDescription:

    def __init__(self, name, fav_drink):
        # TODO: coordinate with Perception on what is easy to implement
        # characteristics to consider: height, hair color, and age.
        self.human_pose = Fluent()
        self.name = name
        self.fav_drink = fav_drink  # self.shirt_color = shirt_color  # self.gender = gender

        self.human_pose_sub = rospy.Subscriber("/human_pose", String, self.human_pose_cb)

    def human_pose_cb(self, HumanPoseMsg):
        self.human_pose.set_value(HumanPoseMsg.data)



def demo_test(area):
    with real_robot:
        # Guest arrives:
        TalkingMotion("Hello, i am Toya let me open my eyes for you").resolve().perform()
        # Perception, detect first guest -> First detect guest, then listen
        DetectAction(technique='default', state='start').resolve().perform()

        host = HumanDescription("Bob", fav_drink="Coffee")

        # While loop, human is detected
        while host.human_pose:
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
        # TODO: How to erst weiter machen, wenn Knowledge Daten geschickt hat

        TalkingMotion("Hey i will stop looking now").resolve().perform()

        rospy.loginfo("stop looking now")
        giskardpy.stop_looking()
        rospy.loginfo("Navigating now")
        TalkingMotion("navigating to couch area now, pls step away").resolve().perform()

        if area == 'to_couch':
            NavigateAction([pose_kitchen_to_couch]).resolve().perform()
            NavigateAction([pose_couch]).resolve().perform()
        elif area == 'from_couch':
            NavigateAction([pose_from_couch]).resolve().perform()
            NavigateAction([pose_home]).resolve().perform()

        # failure handling
        # rospy.Subscriber("nlp_feedback", Bool, talk_error)

        # receives name and drink via topic
        # rospy.Subscriber("nlp_out", String, talk_request)


def nav_test():
    with real_robot:
        robot_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
        test_pose1 = Pose([4.2, 3, 0], robot_orientation)
        test_pose = Pose([3, 5, 0], [0, 0, 0, 1])
        moveBase.queryPoseNav(test_pose1)
        moveBase.queryPoseNav(test_pose)

# demo_test('from_couch')
# demo_test('to_couch')

# receives name and drink via topic
# rospy.Subscriber("nlp_out", String, talk_request)


# 1. rasa run --enable-api -> start Rasa Server
# 2. python3 activate_language_processing.py -> NLP
# 3. roslaunch suturo_bringup suturo_bringup.launch -> Map
# 4. roslaunch_hsr_velocity_controller unloas_my_controller.launch
# 5. roslaunch giskardpy giskardpy_hsr_real_vel.launch -> Giskard
# starten
# 6. rosrun robokudo main.py _ae=humandetection_demo_ros_pkg=milestone1 -> Perception
# 7. run demo in Pycharm -> Planning
