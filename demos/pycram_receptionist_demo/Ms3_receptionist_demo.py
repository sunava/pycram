from pycram.designators.action_designator import DetectAction, NavigateAction, OpenAction
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
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

# variables for communcation with nlp
pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)
response = ""
callback = False
doorbell = True

# Declare variables for humans
host = HumanDescription("Bob", fav_drink="coffee")
guest1 = HumanDescription("guest1")
guest2 = HumanDescription("guest2")
seat_number = 2


def data_cb(data):
    global response
    global callback

    response = data.data.split(",")
    response.append("None")
    callback = True


with real_robot:
    
    # receive data from nlp via topic
    rospy.Subscriber("nlp_out", String, data_cb)

    while not doorbell:
        # TODO: spin or sleep better?
        rospy.spin()

    # TODO: find name of door handle
    # link in rviz: iai_kitchen:arena:door_handle_inside
    # door_handle_desig = ObjectPart(names=["door_handle_inside"], part_of=kitchen_desig.resolve())
    # OpenAction(object_designator_description=door_handle_desig, arms=["left"]).resolve().perform()
    # NavigateAction([pose_door]).resolve().perform()

    TalkingMotion("Welcome, please step in").resolve().perform()

    # look for human
    attr_list = DetectAction(technique='attributes', state='start').resolve().perform()
    rospy.loginfo("human detected")

    guest1.set_attributes(attr_list)
    print(attr_list)

    DetectAction(technique='human').resolve().perform()

    # look at guest and introduce
    giskardpy.move_head_to_human()
    TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()
    rospy.sleep(1)

    # signal to start listening
    pub_nlp.publish("start listening")

    while not callback:
        rospy.sleep(1)
    callback = False

    if response[0] == "<GUEST>":
        # success a name and intent was understood
        if response[1] != "<None>":
            TalkingMotion("please confirm if i got your name right").resolve().perform()
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
    rospy.loginfo("stop looking now")
    # TODO: look in direction of navigation maybe?
    giskardpy.stop_looking()

    # stop perceiving human
    rospy.loginfo("stop detecting")
    DetectAction(technique='human', state='stop').resolve().perform()

    # lead human to living room
    # TODO: check if rospy.sleep is needed and how long
    rospy.sleep(2)
    NavigateAction([pose_kitchen_to_couch]).resolve().perform()
    NavigateAction([pose_couch]).resolve().perform()
    TalkingMotion("Welcome to the living room").resolve().perform()
    rospy.sleep(1)
    TalkingMotion("please take a seat next to your host").resolve().perform()

    # search for free place to sit and host
    # TODO: Failure Handling: scan room if no human detected on couch


    # TODO: is it ok to seat guest bevore introducing??

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
    introduce(host, guest1)
    describe(guest1)

