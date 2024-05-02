import rospy

from pycram.designators.action_designator import DetectAction, NavigateAction
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
            giskardpy.move_head_to_human()


            desig = DetectAction(technique='attributes').resolve().perform()
            print("msgs from PPP: " + str(desig))
            # guest1.set_attributes(desig)




            TalkingMotion("demo over").resolve().perform()

        if seat:
            # new Query for free seat
            #TalkingMotion("detecting free seat on whole couch now").resolve().perform()
            #seat = DetectAction(technique='location', state='sofa').resolve().perform()
            #rospy.loginfo("seat bool: " + str(seat))
            print("start")
            TalkingMotion("detecting").resolve().perform()
            seat1 = DetectAction(technique='location', state='seat1').resolve().perform()
            seat1 = seat1[1]
            if seat1.strip() == "false":
                TalkingMotion("the seat is free").resolve().perform()
            else:
                TalkingMotion("the seat is not free").resolve().perform()
                rospy.sleep(2)
                TalkingMotion("detecting seat number 2 now").resolve().perform()
                seat = DetectAction(technique='location', state='seat2').resolve().perform()
                seat = seat[1]
                if seat.strip() == "false":
                    TalkingMotion("the seat is free").resolve().perform()
            # rospy.loginfo("seat bool: " + str(seat1))

            # print(seat1[1][0])
            # print("##################")
            rospy.sleep(1)

            # TalkingMotion("detecting free seat number 2 now").resolve().perform()
            #seat = DetectAction(technique='location', state='seat1').resolve().perform()
            #rospy.loginfo("seat bool: " + str(seat))
            #rospy.sleep(3)

        print("end")


if __name__ == '__main__':
     p()
