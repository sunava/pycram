import rospy
from robokudo_msgs.msg import QueryActionGoal

from pycram.external_interfaces import robokudo
from pycram.process_module import simulated_robot, with_simulated_robot, real_robot, with_real_robot, semi_real_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper
import pycram.external_interfaces.giskard as giskardpy
from pycram.fluent_language_misc import failure_handling
import threading
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.plan_failures import TorsoFailure
from pycram.language import macros, par
import sys
#from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.enums import ObjectType
from std_msgs.msg import String
from pycram.external_interfaces import giskard
import actionlib
from tmc_msgs.msg import TalkRequestActionGoal
import talk_actions


from pycram.ros.robot_state_updater import RobotStateUpdater
#from pycram.ros.joint_state_publisher import JointStatePublisher

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

world.set_gravity([0, 0, -9.8])
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])
kitchen = Object("kitchen", "environment", "kitchen.urdf")
robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])


#giskardpy.init_giskard_interface()
#giskardpy.sync_worlds()
#RobotStateUpdater("/tf", "/giskard_joint_states")
#/pycram/viz_marker topic bei Marker Array


class HumanDescription:
    def __init__(self, name, fav_drink, shirt_color, gender):
        self.name = name
        self.fav_drink = fav_drink
        self.shirt_color = shirt_color
        self.gender = gender
        # characteristics to consider: height, hair color, and age. TODO: coordinate with Perception on what is easy to implement


def talk_request(data):

    """
    callback function that takes the data from nlp (name and drink) and lets the robot talk
    :param data: String "name drink"
    """
    print(data)
    #
    # name_drink = data.split(" ")
    # talk_test.name_drink_talker(name_drink)
    # rospy.loginfo("nlp data:" + name_drink[0] + " " + name_drink[1])




with real_robot:
    #Query to perception starts the whole demo
    #when a human is perceived the object_pose variable is filled with content
    #alternative
    #object_desig_human = ObjectDesignatorDescription(types=["ObjectType.HUMAN"])
    #object_pose = robokudo.query(object_desig_human) #kriege ich hier erst Daten, wenn Perception einen Mensch erkennt?
    #TODO: rictigen Topic Namen für Perception Node herausfinden
    pub_robokudo = rospy.Publisher('/robokudovanessa/query/goal', QueryActionGoal, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    msgs = QueryActionGoal()
    # talk_test.talker("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?")
    rospy.sleep(2)
    pub_robokudo.publish(msgs)

    #if len(object_pose) > 0:

    #send signal to NLP to start listening
    rospy.loginfo("human detected")
    pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    talk_actions.talker("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?")
    rospy.sleep(2)
    pub_nlp.publish("start listening")

        #Giskard Funktion hinzugefügt
        #bin aber noch auf Simons Branch, die Funktion die in der giskady.py aufgerufen sollte nicht vorhanden sein
        #giskardpy.move_head_to_human()

        #Nlp Interface, gets data in the form of "name drink"
    rospy.Subscriber("nlp_out", String, talk_request)