import time
import rospy
from geometry_msgs.msg import PointStamped

from pycram.designators.object_designator import Pose, PoseStamped, HumanDescription
from pycram.enums import ImageEnum
from pycram.helper import axis_angle_to_quaternion
from std_msgs.msg import String, UInt16

from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher

# Publisher for NLP
pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)
pub_pose = rospy.Publisher('/human_pose', PointStamped, queue_size=10)
# /hsrb/serial_node
pub_color = rospy.Publisher('/hsrb/command_status_led', UInt16, queue_size=5, latch=True)
response = ""
callback = False
timeout = 5

talk = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()


# TODO: set to False when NLP has implemented that feature


# Pose variables
# Pose in front of the couch, HSR looks in direction of couch
pose_couch = Pose([2.7, 5, 0], [0, 0, 1, 0])
pose_door = Pose([1.4, 0.25, 0], [0, 0, 1, 0])
pose_red_seat = [1.1, 4.6, 1]
pose_blue_seat = [1.1, 5.9, 1]
wall_seat_left = [1.8, 6.1, 1]
wall_seat_right = [2.6, 6.1, 1]

# Pose in the passage between kitchen and living room
robot_orientation = axis_angle_to_quaternion([0, 0, 1], 270)
door_to_couch = Pose([3.65, 3.0, 0], robot_orientation)


def data_cb(data):
    global response
    global callback

    image_switch_publisher.pub_now(ImageEnum.HI.value)
    response = data.data.split(",")
    response.append("None")
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
    callback = False
    rospy.Subscriber("nlp_out", String, data_cb)

    talk.pub_now("is your name " + str(name) + "?")
    rospy.sleep(0.8)
    pub_nlp.publish("start now")

    #sound/picture
    rospy.sleep(3.5)
    image_switch_publisher.pub_now(ImageEnum.TALK.value)


    start_time = time.time()
    while not callback:
        # signal repeat to human
        if time.time() - start_time == timeout:
            print("guest needs to repeat")
            image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

    image_switch_publisher.pub_now(ImageEnum.HI.value)
    callback = False

    if response[1].strip() == "True":
        talk.pub_now("alright")
        return name
    else:
        talk.pub_now("please repeat your name")
        rospy.sleep(0.6)
        pub_nlp.publish("start")

        # sound/picture
        rospy.sleep(3.5)
        image_switch_publisher.pub_now(ImageEnum.TALK.value)


        start_time = time.time()
        while not callback:
            # signal repeat to human
            if time.time() - start_time == timeout:
                print("guest needs to repeat")
                image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

        image_switch_publisher.pub_now(ImageEnum.HI.value)
        callback = False

        if response[0] == "<GUEST>" and response[1].strip() != "None":
            return response[1]

        else:
            talk.pub_now("i have to continue with wrong name, im sorry")
            return name


def name_repeat():
    """
    HRI-function to ask for name again once.
    """

    global callback
    global response
    callback = False
    got_name = False
    rospy.Subscriber("nlp_out", String, data_cb)

    while not got_name:
        talk.pub_now("i am sorry, please repeat your name")
        rospy.sleep(0.7)
        pub_nlp.publish("start")

        # sound/picture
        rospy.sleep(3.5)
        image_switch_publisher.pub_now(ImageEnum.TALK.value)

        start_time = time.time()
        while not callback:
            # signal repeat to human
            if time.time() - start_time == timeout:
                print("guest needs to repeat")
                image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

        image_switch_publisher.pub_now(ImageEnum.HI.value)
        callback = False

        if response[0] == "<GUEST>" and response[1].strip() != "None":
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
    callback = False
    rospy.Subscriber("nlp_out", String, data_cb)

    talk.pub_now("is your favorite drink " + str(drink) + "?")
    rospy.sleep(0.6)
    pub_nlp.publish("start")

    # sound/picture
    rospy.sleep(3.5)
    image_switch_publisher.pub_now(ImageEnum.TALK.value)

    start_time = time.time()
    while not callback:
        # signal repeat to human
        if time.time() - start_time == timeout:
            print("guest needs to repeat")
            image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

    image_switch_publisher.pub_now(ImageEnum.HI.value)
    callback = False

    if response[1].strip() == "True":
        talk.pub_now("great, nice to meet you")
        return drink
    else:
        talk.pub_now("i am sorry, please repeat the drink")
        rospy.sleep(0.6)
        pub_nlp.publish("start")

        # sound/picture
        rospy.sleep(3.5)
        image_switch_publisher.pub_now(ImageEnum.TALK.value)

        start_time = time.time()
        while not callback:
            # signal repeat to human
            if time.time() - start_time == timeout:
                print("guest needs to repeat")
                image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)

        image_switch_publisher.pub_now(ImageEnum.HI.value)
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
        rospy.sleep(1.0)
    talk.pub_now(f"Hey, {human1.name}")
    rospy.sleep(2.0)

    if human2.pose:
        pub_pose.publish(human2.pose)
        rospy.sleep(1)
    talk.pub_now(f" This is {human2.name} and their favorite drink is {human2.fav_drink}")
    rospy.sleep(2)
    talk.pub_now(f"Hey, {human2.name}")
    rospy.sleep(1.5)

    if human1.pose:
        pub_pose.publish(human1.pose)
        rospy.sleep(1.5)
    talk.pub_now(f" This is {human1.name} and their favorite drink is {human1.fav_drink}")

    rospy.sleep(1)


def describe(human: HumanDescription):
    """
    HRI-function for describing a human more detailed.
    the following will be stated: gender, headgear, clothing, brightness of clothes
    :param human: human to be described
    """
    if human.attributes:

        if human.pose:
            pub_pose.publish(human.pose)

        talk.pub_now(f"I will describe {human.name} further now")
        rospy.sleep(1)

        # gender
        talk.pub_now(f"i think your gender is {human.attributes[0]}")
        rospy.sleep(1)

        # headgear or not
        talk.pub_now(f"you are {human.attributes[1]}")
        rospy.sleep(1)

        # kind of clothes
        talk.pub_now(f"you are  {human.attributes[2]}")
        rospy.sleep(1)

        # brightness of clothes
        talk.pub_now(f"you are wearing {human.attributes[3]}")
        rospy.sleep(1)


def toPoseStamped(x_cord: float, y_cord: float, z_cord: float) -> PoseStamped:
    """
    function to transform floats to PoseStamped in '/map' frame
    """
    pose = PoseStamped()
    pose.header.frame_id = "/map"
    pose.pose.position.x = x_cord
    pose.pose.position.y = y_cord
    pose.pose.position.z = z_cord

    return pose
