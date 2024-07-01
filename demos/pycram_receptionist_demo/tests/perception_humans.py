import rospy
from demos.pycram_receptionist_demo.utils.new_misc import *
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.external_interfaces.robokudo import faces_queryHuman
from pycram.process_module import semi_real_robot, real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from demos.pycram_receptionist_demo.utils.new_misc import *
from pycram.helper import axis_angle_to_quaternion



world = BulletWorld("DIRECT")
# /pycram/viz_marker topic bei Marker Array
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

# careful that u spawn the correct kitchen
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
giskardpy.init_giskard_interface()
host = HumanDescription("Lukas", fav_drink="water")
guest1 = HumanDescription("Jule", fav_drink="tea")

#for obj in world.current_bullet_world.objects:
#    print(obj)


# kitchen.set_joint_state("iai_kitchen:arena:door_origin_revolute_joint", 1)

def p():
    with real_robot:
        seat = False
        attributes = True

        if attributes:

            # to signal the start of demo
            # TalkingMotion("Hello, i am ready for the pp.py").resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()

            TalkingMotion("Welcome, please step in").resolve().perform()
            MoveTorsoAction([0.1]).resolve().perform()

            TalkingMotion("detecting human now").resolve().perform()
            rospy.sleep(2)
            desig = DetectAction(technique='human', state='start').resolve().perform()
            HeadFollowAction('start').resolve().perform()
            pub_nlp.publish("start listening")
            # desig = DetectAction(technique='attributes').resolve().perform()




            #TalkingMotion("Test").resolve().perform()
            #rospy.sleep(1)

            try:
                # remember face
                keys = DetectAction(technique='human', state='face').resolve().perform()[1]
                new_id = keys["keys"][0]
                guest1.set_id(new_id)


                # get clothes and gender
                attr_list = DetectAction(technique='attributes', state='start').resolve().perform()
                guest1.set_attributes(attr_list)
                rospy.loginfo(attr_list)

            except KeyError:
                print("error")

            HeadFollowAction('start').resolve().perform()
            pub_pose.publish(keys[new_id])

            rospy.sleep(2)
            TalkingMotion("end").resolve().perform()


        if seat:
            # new Query for free seat
            # TalkingMotion("detecting free seat on whole couch now").resolve().perform()
            seat = DetectAction(technique='location', state="sofa").resolve().perform()
            rospy.loginfo(seat[1])
            # rospy.sleep(2)
            for place in seat[1]:
                print(place)
                print(place[0])
                if place[0] == 'False':
                    PointingMotion(float(place[1]), float(place[2]), float(place[3])).resolve().perform()
                    print("free")

            # print("########################")
            # print(seat[1][1][1])
            # if seat[1][0][0] == 'False':
            # print(seat[1][0][1])
            # PointingMotion(float(seat[1][1][1]), float(seat[1][1][2]), float(seat[1][1][3])).resolve().perform()


def ms3_perception():
    with real_robot:
        TalkingMotion("start").resolve().perform()


        HeadFollowAction('start').resolve().perform()
        host_pose = DetectAction(technique='human').resolve().perform()
        print(host_pose)

        rospy.sleep(10)

        TalkingMotion("end").resolve().perform()


if __name__ == '__main__':
    p()
