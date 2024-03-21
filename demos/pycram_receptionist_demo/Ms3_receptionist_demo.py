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
    DetectAction(technique='human', state='start').resolve().perform()
    rospy.loginfo("human detected")

    # look at guest and introduce
    giskardpy.move_head_to_human()
    TalkingMotion("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?").resolve().perform()
    rospy.sleep(1)

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

    while not misc.understood:
        rospy.sleep(1)


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
    #NavigateAction([misc.pose_kitchen_to_couch]).resolve().perform()
    #NavigateAction([misc.pose_couch]).resolve().perform()
    TalkingMotion("Welcome to the living room").resolve().perform()
    rospy.sleep(1)

    # search for free place to sit and host
    # TODO: get pose of host that sits in living room
    # TODO: Failure Handling: scan room if no human detected on couch
    human_pose = DetectAction(technique='human', state='start').resolve().perform()
    print(human_pose)
    misc.host.set_pose(human_pose)
    # TODO: HSR looks to his right??
    misc.guest1.set_pose()

    # point to free place
    # giskardpy.point_to_seat

    # introduce humans and look at them
    misc.introduce()
