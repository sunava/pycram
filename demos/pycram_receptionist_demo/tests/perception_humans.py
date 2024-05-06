import rospy

from pycram.designators.action_designator import DetectAction, NavigateAction, HeadFollowAction
from pycram.designators.motion_designator import PointingMotion
from pycram.process_module import semi_real_robot, real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from demos.pycram_receptionist_demo.utils.new_misc import *

world = BulletWorld("DIRECT")
# /pycram/viz_marker topic bei Marker Array
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

# careful that u spawn the correct kitchen
kitchen = Object("kitchen", "environment", "suturo_lab_version_2.urdf")
giskardpy.init_giskard_interface()
guest1 = HumanDescription("guest1")

#for obj in world.current_bullet_world.objects:
#   print(obj)

#kitchen.set_joint_state("iai_kitchen:arena:door_origin_revolute_joint", 1)

def p():
    with real_robot:
        seat = False
        attributes = True

        if attributes:
            # to signal the start of demo
            # TalkingMotion("Hello, i am ready for the test").resolve().perform()


            TalkingMotion("detecting attributes now").resolve().perform()
            rospy.sleep(2)

            HeadFollowAction('start').resolve().perform()
            desig = DetectAction(technique='human', state='start').resolve().perform()
            # desig = DetectAction(technique='attributes').resolve().perform()
            print("msgs from PPP: " + str(desig))
            #if desig != "False":
             #   guest1.set_attributes(desig)

             #   describe(guest1)
               # rospy.sleep(2)

            TalkingMotion("attributes over").resolve().perform()

        if seat:
            # new Query for free seat
            #TalkingMotion("detecting free seat on whole couch now").resolve().perform()
            seat = DetectAction(technique='location', state='sofa').resolve().perform()
            rospy.loginfo(seat[1])
            #rospy.sleep(2)

            print("########################")
            print(seat[1][1][1])
            if seat[1][1][1] == 'occupied:false':
                print(seat[1][1][2][2:])
                PointingMotion(float(seat[1][1][2][2:]), float(seat[1][1][3][2:]), float(seat[1][1][4][2:])).resolve().perform()

            print("start")


        print("end")


if __name__ == '__main__':
     p()
