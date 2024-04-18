import rospy

from pycram.designators.action_designator import DetectAction, NavigateAction
from demos.pycram_receptionist_demo.utils.new_misc import *
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

# variables for communcation with nlp
pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)
response = ""
callback = False

# Declare variables for humans
host = HumanDescription("Lukas", fav_drink="Coffee")
guest1 = HumanDescription("Jessica", fav_drink="Water")
guest2 = HumanDescription("guest2")
seat_number = 2

def data_cb(data):
    global response
    global callback

    response = data.data.split(",")
    response.append("None")
    callback = True

def demo_tst():
    """
    testing HRI and introduction and navigating
    """
    with real_robot:
        global callback
        global response
        test_all = False

        rospy.Subscriber("nlp_out", String, data_cb)
        DetectAction(technique='human', state='start').resolve().perform()
        rospy.loginfo("human detected")

        giskardpy.move_head_to_human()
        TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()
        rospy.sleep(0.9)

        # signal to start listening
        pub_nlp.publish("start listening")

        while not callback:
            rospy.sleep(1)
        callback = False

        if response[0] == "<GUEST>":
            if response[1] != "<None>":
                TalkingMotion("it is so noisy here, please confirm if i got your name right").resolve().perform()
                guest1.set_drink(response[2])
                rospy.sleep(1)
                guest1.set_name(name_confirm(response[1]))

            else:
                # save heard drink
                guest1.set_drink(response[2])

                # ask for name again once
                guest1.set_name(name_repeat())

            # confirm favorite drink
            guest1.set_drink(drink_confirm(guest1.fav_drink))

        else:
            # two chances to get name and drink
            i = 0
            while i < 2:
                TalkingMotion("please repeat your name and drink loud and clear").resolve().perform()
                pub_nlp.publish("start")

                while not callback:
                    rospy.sleep(1)
                callback = False

                if response[0] == "<GUEST>":
                    guest1.set_name(response[1])
                    guest1.set_drink(response[2])
                    break
                else:
                    i += 1


        # stop looking
        TalkingMotion("i will show you the living room now").resolve().perform()
        rospy.sleep(1)
        TalkingMotion("please step out of the way and follow me").resolve().perform()
        giskardpy.stop_looking()

        # stop perceiving human
        DetectAction(technique='human', state='stop').resolve().perform()

        if test_all:
            # lead human to living room
            NavigateAction([pose_kitchen_to_couch]).resolve().perform()
            NavigateAction([pose_couch]).resolve().perform()
            TalkingMotion("Welcome to the living room").resolve().perform()
            rospy.sleep(1)

            TalkingMotion("please take a seat next to your host").resolve().perform()
            rospy.sleep(2)

            # hard coded poses for seat1 and seat2
            pose_host = PoseStamped()
            pose_host.header.frame_id = "/map"
            pose_host.pose.position.x = 1
            pose_host.pose.position.y = 5.9
            pose_host.pose.position.z = 1

            pose_guest = PoseStamped()
            pose_guest.header.frame_id = "/map"
            pose_guest.pose.position.x = 1
            pose_guest.pose.position.y = 4.7
            pose_guest.pose.position.z = 1

            host.set_pose(pose_host)
            guest1.set_pose(pose_guest)

            # introduce humans and look at them
            giskardpy.move_head_to_human()

        rospy.sleep(1)
        introduce(host, guest1)


def demo_tst2():
    """
    just testing the gazing between humans -> introduce function
    """
    with real_robot:

        TalkingMotion("Welcome, please come in").resolve().perform()

        giskardpy.move_head_to_human()

        TalkingMotion("Hello, i will alternaze gaze now").resolve().perform()
        rospy.sleep(1)


        pose_host = PoseStamped()
        pose_host.header.frame_id = '/map'
        pose_host.pose.position.x = 1.0
        pose_host.pose.position.y = 5.9
        pose_host.pose.position.z = 0.9

        pose_guest = PoseStamped()
        pose_guest.header.frame_id = '/map'
        pose_guest.pose.position.x = 1.0
        pose_guest.pose.position.y = 4.7
        pose_guest.pose.position.z = 1.0

        host.set_pose(pose_host)
        guest1.set_pose(pose_guest)

        # introduce humans and look at them
        giskardpy.move_head_to_human()
        rospy.sleep(3)
        introduce(host, guest1)

        rospy.sleep(2)
        TalkingMotion("Introducing again").resolve().perform()
        rospy.sleep(2)
        introduce(host, guest1)



demo_tst2()


