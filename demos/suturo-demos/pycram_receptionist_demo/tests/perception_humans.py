from pycram.designators.action_designator import DetectAction
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object

world = BulletWorld("DIRECT")
# /pycram/viz_marker topic bei Marker Array
v = VizMarkerPublisher()

robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

# carefull that u spawn the correct kitchen
kitchen = Object("kitchen", "environment", "../../../../resources/kitchen.urdf")
giskardpy.init_giskard_interface()
#RobotStateUpdater("/tf", "/giskard_joint_states")

def test():
    with real_robot:
        guest1 = HumanDescription("x")
       # print("...................... nothing" )
        guest1.human_pose.set_value(False)
        if not guest1.human_pose.get_value():
            print("wrong? " + str(guest1.human_pose))

        if guest1.human_pose.get_value():
            print("right? " + str(guest1.human_pose))

        print("----------------------------------")
        DetectAction(technique='human', state='start').resolve().perform()

        if not guest1.human_pose.get_value():
            print("wrong? " + str(guest1.human_pose))

        if guest1.human_pose.get_value():
            print("right? " + str(guest1.human_pose))



        rospy.loginfo("human detected")

        print("---------- " + "start")

        rospy.loginfo("sleeping now")

        rospy.sleep(5)

        rospy.loginfo("sleep done stopping now")

        DetectAction(technique='human', state='stop').resolve().perform()

        print("------------------------------- stop")

        print("end")










if __name__ == '__main__':
    test()