import rospy

from pycram.fluent import Fluent
from pycram.process_module import real_robot
from std_msgs.msg import String


class HumanDescription:

    def __init__(self, name, fav_drink):
        # TODO: coordinate with Perception on what is easy to implement
        # characteristics to consider: height, hair color, and age.
        self.human_pose = Fluent()
        self.name = name
        self.fav_drink = fav_drink  # self.shirt_color = shirt_color  # self.gender = gender

        self.human_pose_sub = rospy.Subscriber("/human_pose", String, self.human_pose_cb)

    def human_pose_cb(self, HumanPoseMsg):
        self.human_pose.set_value(HumanPoseMsg.data)


with real_robot:
    host = HumanDescription("Bob", fav_drink="Coffee")
    #detect
    # Perception, detectfirst guest
    # If detect true then

    host.human_pose.set_value("True")

    # While loop, human is detected
    while host.human_pose.get_value():
        print(host.human_pose.get_value())

    rospy.loginfo("human not detected anymore")