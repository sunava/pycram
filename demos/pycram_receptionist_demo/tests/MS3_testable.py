import rospy

from pycram.designators.action_designator import DetectAction, NavigateAction, LookAtAction
from pycram.designators.motion_designator import TalkingMotion
import demos.pycram_receptionist_demo.utils.misc as misc
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

kitchen = Object("kitchen", "environment", "kitchen.urdf")
giskardpy.init_giskard_interface()
RobotStateUpdater("/tf", "/giskard_joint_states")

pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)


def demo_tst():
    with real_robot:

        # wait for doorbell
        bell_subscriber = rospy.Subscriber("doorbell", Bool, misc.doorbell_cb)
        while not misc.doorbell:
            # TODO: spin or sleep better?
            # TODO: Failure Handling, when no bell is heard for a longer period of time
            rospy.spin()

        # subscriber not needed anymore
        bell_subscriber.unregister()

        # NavigateAction([misc.pose_door]).resolve().perform()
        # giskardpy.opendoor()

        TalkingMotion("Welcome, please come in").resolve().perform()

        # look for human
        # TODO: test new technique
        DetectAction(technique='human', state='start').resolve().perform()
        rospy.loginfo("human detected")

        # look at guest and introduce
        giskardpy.move_head_to_human()
        TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()
        # rospy.sleep(1)

        # signal to start listening
        pub_nlp.publish("start listening")

        # receives name and drink via topic
        rospy.Subscriber("nlp_out", String, misc.talk_request_nlp)

        # failure handling
        rospy.Subscriber("nlp_feedback", Bool, misc.talk_error)

        while misc.guest1.name == "guest1":
            rospy.sleep(1)

        TalkingMotion("it is so noisy here, please confirm if i got your name right").resolve().perform()
        rospy.sleep(1)
        TalkingMotion("is your name " + misc.guest1.name + "?").resolve().perform()
        pub_nlp.publish("start")

        rospy.Subscriber("nlp_confirmation", Bool, misc.name_cb)

        while not misc.understood_name:
            rospy.sleep(1)

        # TODO: implement HRI for fav drink
        TalkingMotion("is your favorite drink " + misc.guest1.fav_drink + "?").resolve().perform()
        pub_nlp.publish("start")

        while not misc.understood_drink:
            rospy.sleep(1)

        # stop looking
        # TalkingMotion("i will show you the living room now").resolve().perform()
        # rospy.sleep(1)
        # TalkingMotion("please step out of the way and follow me").resolve().perform()
        # rospy.loginfo("stop looking now")
        # TODO: look in direction of navigation maybe?
        # giskardpy.stop_looking()

        # stop perceiving human
        # rospy.loginfo("stop detecting")
        DetectAction(technique='human', state='start').resolve().perform()
        rospy.sleep(5)
        DetectAction(technique='human', state='stop').resolve().perform()
        # lead human to living room
        # TODO: check if rospy.sleep is needed and how long
        # rospy.sleep(2)
        # NavigateAction([misc.pose_kitchen_to_couch]).resolve().perform()
        # NavigateAction([misc.pose_couch]).resolve().perform()
        # TalkingMotion("Welcome to the living room").resolve().perform()
        # rospy.sleep(1)

        # TalkingMotion("please take a seat next to your host").resolve().perform()
        rospy.sleep(2)

        # point to free place
        # giskardpy.point_to_seat

        pose_host = PoseStamped()
        pose_host.header.frame_id = "/map"
        pose_host.pose.position.x = 1
        # pose_host.pose.position.y = 5.9
        pose_host.pose.position.y = 2
        pose_host.pose.position.z = 1

        pose_guest = PoseStamped()
        pose_guest.header.frame_id = "/map"
        pose_guest.pose.position.x = 1
        # pose_guest.pose.position.y = 4.7
        pose_guest.pose.position.y = -0.4
        pose_guest.pose.position.x = 1

        misc.host.set_pose(pose_host)
        misc.host.set_drink("beer")
        misc.guest1.set_pose(pose_guest)

        # introduce humans and look at them
        giskardpy.move_head_to_human()
        print("täsr" + misc.host.fav_drink)
        misc.introduce(pose_b=pose_host, pose_a=pose_guest)


def demo_tst2():
    with real_robot:
        # wait for doorbell
        bell_subscriber = rospy.Subscriber("doorbell", Bool, misc.doorbell_cb)
        while not misc.doorbell:
            # TODO: spin or sleep better?
            # TODO: Failure Handling, when no bell is heard for a longer period of time
            rospy.spin()

        # subscriber not needed anymore
        bell_subscriber.unregister()

        # NavigateAction([misc.pose_door]).resolve().perform()
        # giskardpy.opendoor()

        TalkingMotion("Welcome, please come in").resolve().perform()

        # look for human
        # TODO: test new technique
        #DetectAction(technique='human', state='start').resolve().perform()
        rospy.loginfo("human detected")
        #giskardpy.move_head_to_human()
        #rospy.sleep(7)
        #DetectAction(technique='human', state='stop').resolve().perform()

        pose_host = PoseStamped()
        pose_host.header.frame_id = 'map'
        pose_host.pose.position.x = 1.0
        pose_host.pose.position.y = 5.9
        pose_host.pose.position.z = 0.9

        pose_guest = PoseStamped()
        pose_guest.header.frame_id = 'map'
        pose_guest.pose.position.x = 1.0
        pose_guest.pose.position.y = 4.7
        pose_guest.pose.position.z = 1.0

        misc.host.set_pose(pose_host)
        misc.host.set_drink("beer")

        misc.guest1.set_pose(pose_guest)

        # introduce humans and look at them
        giskardpy.move_head_to_human()
        rospy.sleep(3)
        misc.introduce(pose_b=pose_host, pose_a=pose_guest)

        rospy.sleep(2)
        TalkingMotion("Introducing again").resolve().perform()
        rospy.sleep(2)

        misc.introduce(pose_b=pose_host, pose_a=pose_guest)


demo_tst2()
