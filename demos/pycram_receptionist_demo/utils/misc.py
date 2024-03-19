from pycram.designators.motion_designator import TalkingMotion
from pycram.designators.object_designator import *
from std_msgs.msg import String
from pycram.helper import axis_angle_to_quaternion


pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)


# pose variables

# pose in front of the couch, hsr looks in direction of couch
pose_couch = Pose([3, 5, 0], [0, 0, 1, 0])

# pose in the passage between kitchen and living room
robot_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
pose_kitchen_to_couch = Pose([4.2, 3, 0], robot_orientation)


# talk functions
def talk_request(data: list):
    """
    function that takes the data from nlp (name and drink) and lets the robot talk
    :param data: list ["name" "drink"]
    """

    name, drink = data
    toyas_text = f"Hey {name}, your favorite drink is {drink}"
    TalkingMotion(toyas_text).resolve().perform()
    rospy.sleep(1)
    TalkingMotion("Nice to meet you").resolve().perform()


def talk_error(data):
    """
    callback function if no name/drink was heard
    """

    error_msgs = "i could not hear you, please repeat"
    TalkingMotion(error_msgs).resolve().perform()
    rospy.sleep(2)
    pub_nlp.publish("start listening")


def introduce(name1, drink1, host_name, host_drink):
    """
    Text for robot to introduce two people to each other
    """

    first = "Hey " + str(host_name) + " This is " + str(name1) + " and the favorite drink of your guest is " + str(drink1)
    second = str(name1) + " This is " + str(host_name) + " his favorite drink is " + str(host_drink)
    TalkingMotion(first).resolve().perform()
    rospy.sleep(3)
    TalkingMotion(second).resolve().perform()


