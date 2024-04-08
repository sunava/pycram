import rospy

from pycram.designators.action_designator import DetectAction, NavigateAction
from pycram.designators.motion_designator import TalkingMotion
from pycram.fluent import Fluent
from pycram.helper import axis_angle_to_quaternion
from pycram.process_module import semi_real_robot, real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String
from demos.pycram_receptionist_demo.deprecated import talk_actions
import pycram.external_interfaces.navigate as moveBase

world = BulletWorld("DIRECT")
# /pycram/viz_marker topic bei Marker Array
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

# careful that u spawn the correct kitchen
kitchen = Object("kitchen", "environment", "kitchen.urdf")
giskardpy.init_giskard_interface()



def p():
    with real_robot:
        seat = True
        attributes = False

        if attributes:
            # to signal the start of demo
            # TalkingMotion("Hello, i am ready for the test").resolve().perform()

            # does the code work with the manipulation feature?
            giskardpy.move_head_to_human()

            # new Query to detect attributes of a human
            TalkingMotion("detecting attributes now").resolve().perform()
            desig = DetectAction(technique='attributes').resolve().perform()
            rospy.loginfo("Attributes: " + str(desig))
            print("#####################")
            print(desig[1].res[0].attribute)
            gender = desig[1].res[0].attribute[0][13:19]
            clothes = desig[1].res[0].attribute[2][20:]
            brightness_clothes = desig[1].res[0].attribute[1]
            hat = desig[1].res[0].attribute[3][20:]

            print("#####################")
            print(gender)
            print("#####################")
            print(clothes)
            print("#####################")
            print(brightness_clothes)
            print("#####################")
            print(hat)

            # rospy.sleep(5)

        if seat:
            # new Query for free seat
            TalkingMotion("detecting free seat now").resolve().perform()
            seat = DetectAction(technique='location', state='seat2').resolve().perform()
            rospy.loginfo("seat bool: " + str(seat))
            rospy.sleep(1)

            TalkingMotion("detecting free seat number 2 now").resolve().perform()
            seat = DetectAction(technique='location', state='seat1').resolve().perform()
            rospy.loginfo("seat bool: " + str(seat))
            rospy.sleep(3)


        print("end")


if __name__ == '__main__':
    p()
