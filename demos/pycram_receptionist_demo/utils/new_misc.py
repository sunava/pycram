import rospy
from pycram.designators.object_designator import Pose, PoseStamped, HumanDescription
from pycram.designators.motion_designator import TalkingMotion
from pycram.helper import axis_angle_to_quaternion
from std_msgs.msg import String


# Publisher for NLP
pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)
pub_pose = rospy.Publisher('/human_pose', PoseStamped, queue_size=10)
response = ""
callback = False

# TODO: set to False when NLP has implemented that feature
doorbell = True

# Pose variables
# Pose in front of the couch, HSR looks in direction of couch
pose_couch = Pose([2.7, 5, 0], [0, 0, 1, 0])
pose_door = Pose([1.4, 0.25, 0], [0, 0, 1, 0])

# Pose in the passage between kitchen and living room
robot_orientation = axis_angle_to_quaternion([0, 0, 1], 90)
pose_kitchen_to_couch = Pose([4.35, 3, 0], robot_orientation)


def data_cb(data):
    global response
    global callback

    response = data.data.split(",")
    callback = True


def name_confirm(name):
    """
    HRI-function to confirm name. in case the name was not right the guest will be asked to repeat.
    if the name was not understood the second time the old name will be returned due to
    time limitations.
    :param name: person name that has to be confirmed
    """

    global callback
    global response
    rospy.Subscriber("nlp_out", String, data_cb)

    TalkingMotion("is your name " + str(name) + "?").resolve().perform()
    pub_nlp.publish("start now")

    while not callback:
        rospy.sleep(1)

    callback = False

    if response[1].strip() == "True":
        TalkingMotion("alright").resolve().perform()
        return name
    else:
        TalkingMotion("please repeat your name").resolve().perform()
        pub_nlp.publish("start")

        while not callback:
            rospy.sleep(1)
        callback = False

        if response[0] == "<GUEST>" and response[1].strip() != "None":
            return response[1]

        else:
            TalkingMotion("i have to continue with wrong name, im sorry").resolve().perform()
            return name


def name_repeat():
    """
    HRI-function to ask for name again once.
    """

    global callback
    global response
    got_name = False
    rospy.Subscriber("nlp_out", String, data_cb)

    while not got_name:
        TalkingMotion("i am sorry, please repeat your name").resolve().perform()
        pub_nlp.publish("start")

        while not callback:
            rospy.sleep(1)
        callback = False

        if response[0] == "<GUEST>" and response[1].strip() != "None":
            got_name = True
            return response[1]




def drink_confirm(drink):
    """
    HRI-function to confirm favorite drink. in case the drink was not right,
    the guest will be asked to repeat.
    if the drink was not understood the second time the drink "water" will be returned
    :param drink: drink that has to be confirmed
    """

    global callback
    global response
    rospy.Subscriber("nlp_out", String, data_cb)

    TalkingMotion("is your favorite drink " + str(drink) + "?").resolve().perform()
    rospy.sleep(1)
    pub_nlp.publish("start")

    while not callback:
        rospy.sleep(1)

    callback = False

    if response[1].strip() == "True":
        TalkingMotion("great, nice to meet you").resolve().perform()
        return drink
    else:
        TalkingMotion("i am sorry, please repeat the drink").resolve().perform()
        pub_nlp.publish("start")

        while not callback:
            rospy.sleep(1)
        callback = False

        if response[0] == "<GUEST>" and response[2].strip() != "None":
            return response[2]

        else:
            return "water"


def introduce(human1: HumanDescription, human2: HumanDescription):
    """
    Text for robot to introduce two people to each other and alternate gaze
    :param human1: the first human the robot talks to
    :param human2: the second human the robot talks to
    """

    if human1.pose:
        pub_pose.publish(human1.pose)
        rospy.sleep(1.5)
    TalkingMotion(f"Hey, {human1.name}").resolve().perform()
    rospy.sleep(1.5)

    if human2.pose:
        pub_pose.publish(human2.pose)
        rospy.sleep(1)
    TalkingMotion(f" This is {human2.name} and their favorite drink is {human2.fav_drink}").resolve().perform()
    rospy.sleep(2)
    TalkingMotion(f"Hey, {human2.name}").resolve().perform()
    rospy.sleep(1.5)

    if human1.pose:
        pub_pose.publish(human1.pose)
        rospy.sleep(1.5)
    TalkingMotion(f" This is {human1.name} and their favorite drink is {human1.fav_drink}").resolve().perform()

    rospy.sleep(1)

    def describe(human: HumanDescription):
        """
        HRI-function for describing a human more detailed.
        the following will be stated: gender, headgear, clothing, brightness of clothes
        :param human: human to be described
        """

        TalkingMotion(f"I will describe {human.name} further now").resolve().perform()
        rospy.sleep(1)

        # gender
        TalkingMotion(f"i think your gender is {human.attributes[0]}").resolve().perform()
        rospy.sleep(1)

        # headgear or not
        TalkingMotion(f"you are wearing {human.attributes[1]}").resolve().perform()
        rospy.sleep(1)

        # kind of clothes
        TalkingMotion(f"you are wearing {human.attributes[2]}").resolve().perform()
        rospy.sleep(1)

        # brightness of clothes
        TalkingMotion(f"your clothes are {human.attributes[3]}").resolve().perform()
        rospy.sleep(1)
