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
move = PoseNavigator()

host = HumanDescription("James", fav_drink="water")

world = BulletWorld("DIRECT")
# /pycram/viz_marker topic bei Marker Array
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

# careful that u spawn the correct kitchen
kitchen = Object("kitchen", "environment", "suturo_lab_version_15.urdf")
giskardpy.init_giskard_interface()
guest1 = HumanDescription("guest1")

for obj in world.current_bullet_world.objects:
    print(obj)


# kitchen.set_joint_state("iai_kitchen:arena:door_origin_revolute_joint", 1)

def p():
    with real_robot:
        seat = False
        attributes = True
        # pose = PoseStamped
        pose = toPoseStamped(2.4,2.1,0)
        move.query_pose_nav(pose)

        if attributes:
            # get attributes

            try:
                attr_list = DetectAction(technique='attributes', state='start').resolve().perform()
                guest1.set_attributes(attr_list)
                print(attr_list)
                id = DetectAction(technique='human', state='face').resolve().perform()[1][0]
                print(id)

                rospy.loginfo(attr_list)
            except PerceptionObjectNotFound:
                TalkingMotion("please step in front of me").resolve().perform()
                rospy.sleep(3)
                attr_list = DetectAction(technique='attributes', state='start').resolve().perform()
                guest1.set_attributes(attr_list)
                print(attr_list)
                id = DetectAction(technique='human', state='face').resolve().perform()[1][0]
                print(id)

            # TODO: face recognition
            # TODO face and attr at same time??
            TalkingMotion("end").resolve().perform()
            rospy.sleep(2)

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
        rospy.sleep(2)

        # lead human to living room
        # NavigateAction([door_to_couch]).resolve().perform()

        TalkingMotion("Welcome to the living room").resolve().perform()
        host_pose = DetectAction(technique='human').resolve().perform()
        host.set_pose(host_pose[1])
        host_pose = DetectAction(technique='human', state='stop').resolve().perform()
        seat = DetectAction(technique='location', state="sofa").resolve().perform()
        for place in seat[1]:
            if place[0] == 'False':
                PointingMotion(float(place[1]), float(place[2]), float(place[3])).resolve().perform()
                pose_guest1 = PoseStamped()
                pose_guest1.header.frame_id = "/map"
                pose_guest1.pose.position.x = float(place[1])
                pose_guest1.pose.position.y = float(place[2])
                pose_guest1.pose.position.z = float(place[3])
                guest1.set_pose(pose_guest1)
                break

        HeadFollowAction('start').resolve().perform()
        pub_pose.publish(guest1.pose)
        TalkingMotion("please take a seat next to your host").resolve().perform()

        # introduce humans and look at them

        introduce(host, guest1)
        describe(guest1)
        HeadFollowAction('stop').resolve().perform()

        TalkingMotion("end").resolve().perform()


if __name__ == '__main__':
    p()
