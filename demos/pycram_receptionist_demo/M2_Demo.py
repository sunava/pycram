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
from demos.pycram_receptionist_demo.deprecated import talk_actions

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
    host = HumanDescription("Bob", fav_drink="Coffee")
    pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)

    # Perception, detect first guest
    perceived_object_dict = DetectAction(BelieveObject(types=[milk.type]), technique='human').resolve().perform()
    while perceived_object_dict[0] is None:
        rospy.sleep(5)
        TalkingMotion("Please step in front of me")
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

    # TODO: knowledge interface erweitern
    # Idee: jeder Mensch braucht ID, diese kann ich dann hier abfragen
    # Host schon in Liste!!, daher < 2 und nicht < 1
    while len(instances_of("Customer")) < 2:

        # TODO: funktioniert das noch mit Subscriber?
        # TODO: NLP muss variabel ein/ausgeschaltet werden können
        # failure handling if receptionist intend was not understood
        rospy.Subscriber("nlp_feedback", Bool, talk_error)
        #TalkingMotion("please repeat")
        rospy.sleep(10)

    #save received data from guest
    guest_data = get_guest_info(1)
    name01 = guest_data[0]
    drink01 = guest_data[1]
    guest1 = HumanDescription(name=name01, fav_drink=drink01)



    #lead guest to living room
    giskardpy.stop_looking()
    NavigateAction([Pose([3, 5, 0], [0, 0, 1, 1])]).resolve().perform()
    #NavigateAction(target_locations=[Pose([3, 5, 0])]).resolve().perform() ??

    # search for host in living room
    if DetectAction(BelieveObject(types=[milk.type]), technique='human').resolve().perform():
        # look at host
        giskardpy.move_head_to_human()

        # introduce guest and host
        introduce(host.name, host.fav_drink, guest1.name, guest1.fav_drink)
        giskardpy.stop_looking()









