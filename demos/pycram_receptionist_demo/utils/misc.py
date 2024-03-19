from pycram.designators.motion_designator import TalkingMotion
from pycram.designators.object_designator import *
from pycram.helper import axis_angle_to_quaternion

# Publisher for NLP
pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)
understood = False
doorbell = False

# Declare variables for humans
host = HumanDescription("Yannis", fav_drink="ice tea")
guest1 = HumanDescription("guest1")
guest2 = HumanDescription("guest2")

# Pose variables
# Pose in front of the couch, HSR looks in direction of couch
pose_couch = Pose([3, 5, 0], [0, 0, 1, 0])

# Pose in the passage between kitchen and living room
robot_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
pose_kitchen_to_couch = Pose([4.2, 3, 0], robot_orientation)


def talk_request(data: list):
    """
    function that takes the data from nlp (name and drink) and lets the robot talk
    :param data: list ["name" "drink"]
    """
    global understood
    name, drink = data
    guest1.set_name(name)
    guest1.set_drink(drink)
    toyas_text = f"Hey {name}, your favorite drink is {drink}"

    TalkingMotion(toyas_text).resolve().perform()
    rospy.sleep(1)
    TalkingMotion("Nice to meet you").resolve().perform()

    understood = True


def talk_request_nlp(data: String):
    """
    function that takes the data from nlp (name and drink) and lets the robot talk
    :param data: String "name, drink"
    """
    global understood
    data_list = data.data.split(",")
    name, drink = data_list
    toyas_text = f"Hey {name}, your favorite drink is {drink}"

    TalkingMotion(toyas_text).resolve().perform()
    rospy.sleep(2)
    TalkingMotion("Nice to meet you").resolve().perform()

    understood = True


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


def doorbell_cb(data):
    """
    callback function for a subscriber to NLP script.
    is called when doorbell sound was detected
    """
    global doorbell
    doorbell = True

def name_cb(data):
    """
    callback function for a subscriber to NLP script.
    is called when name was correctly understood
    """
    global name
    name = data



