import rospy
from sensor_msgs.msg import LaserScan
from sound_play.msg import SoundRequestActionGoal, SoundRequest
from std_msgs.msg import Int32
from tmc_msgs.msg import Voice
from pycram.designators.object_designator import *


class SoundRequestPublisher:
    """
    A class to publish sound requests in a ROS environment.
    """

    current_subscriber: rospy.Subscriber = None
    """
    Reference to the current subscriber instance.
    """

    def __init__(self, topic='/sound_play/goal', queue_size=10, latch=True):
        """
        Initializes the SoundRequestPublisher with a ROS publisher.

        :param topic: The ROS topic to publish sound requests to. Default is '/sound_play/goal'.
        :param queue_size: The size of the message queue for the publisher. Default is 10.
        :param latch: Whether the publisher should latch messages. Default is True.
        """
        self.pub = rospy.Publisher(topic, SoundRequestActionGoal, queue_size=queue_size, latch=latch)
        self.msg = SoundRequestActionGoal()
        self.msg.goal.sound_request.sound = 1
        self.msg.goal.sound_request.command = 1
        self.msg.goal.sound_request.volume = 2.0

    def publish_sound_request(self):
        """
        Publish the sound request message.
        """
        rospy.loginfo("Publishing sound request")
        rospy.loginfo("Waiting for subscribers to connect...")
        while self.pub.get_num_connections() == 0:
            rospy.sleep(0.1)  # Sleep for 100ms and check again
        self.pub.publish(self.msg)
        rospy.loginfo("Sound request published")


class StartSignalWaiter:
    """
    A class to wait for a start signal based on laser scan data in a ROS environment.
    """

    current_subscriber: rospy.Subscriber = None
    """
    Reference to the current subscriber instance.
    """

    def __init__(self):
        """
        Initializes the StartSignalWaiter with a Fluent object.
        """
        self.fluent = Fluent()
        self.current_subscriber = None

    def wait_for_startsignal(self):
        """
        Subscribe to the laser scan topic and wait for the door to open before continuing.
        """

        def laser_scan_callback(msg):
            ranges = list(msg.ranges)
            if len(ranges) > 481 and ranges[481] > 1.0:
                self.fluent.set_value(True)
                rospy.loginfo("Door is open, unsubscribing from topic")
                self.current_subscriber.unregister()

        rospy.loginfo("Waiting for starting signal.")
        self.current_subscriber = rospy.Subscriber("hsrb/base_scan", LaserScan, laser_scan_callback)

        rospy.loginfo("Waiting for door to open")
        self.fluent.wait_for()

        rospy.loginfo("Start signal received.")


class TextToSpeechPublisher:
    """
    A class to publish text-to-speech requests in a ROS environment.
    """

    def __init__(self, topic='/talk_request', queue_size=10):
        """
        Initializes the TextToSpeechPublisher with a ROS publisher.

        :param topic: The ROS topic to publish text-to-speech requests to. Default is '/talk_request'.
        :param queue_size: The size of the message queue for the publisher. Default is 10.
        """
        self.pub = rospy.Publisher(topic, Voice, queue_size=queue_size)
        self.talking_sentence = ''
        rospy.Subscriber('/talking_sentence', String, self.talking_sentence_callback)

    def talking_sentence_callback(self, msg, talk: bool = True):
        """
        Callback function to update the talking_sentence attribute.

        :param msg: The ROS message received from the /talking_sentence topic.
        """
        if talk:
            self.talking_sentence = msg.data

    def pub_now(self, text, language=1):
        """
        Publishes a text-to-speech request with the given text and language.

        :param text: The text to be spoken.
        :param language: The language of the text. 1 for English, 0 for Japanese. Default is 1 (English).
        """
        rospy.loginfo("talkstring:" + text)
        # while self.talking_sentence != '':
        #     rospy.sleep(0.1)  # Sleep for 100ms and check again
        text_to_speech = Voice()
        text_to_speech.language = language
        text_to_speech.sentence = text
        while self.pub.get_num_connections() == 0:
            rospy.sleep(0.1)  # Sleep for 100ms and check again
        self.pub.publish(text_to_speech)


class ImageSwitchPublisher:
    """
    A class to publish image switch requests in a ROS environment.
    """

    def __init__(self, topic='/media_switch_topic', queue_size=10, latch=True):
        """
        Initializes the ImageSwitchPublisher with a ROS publisher.

        :param topic: The ROS topic to publish image switch requests to. Default is '/image_switch_topic'.
        :param queue_size: The size of the message queue for the publisher. Default is 10.
        :param latch: Whether the publisher should latch messages. Default is True.
        """
        self.pub = rospy.Publisher(topic, Int32, queue_size=queue_size, latch=latch)
        self.msg = Int32()

    def pub_now(self, image_id):
        """
        Publish the image switch request message.

        :param image_id: The ID of the image to switch to.
        """
        self.msg.data = image_id
        rospy.loginfo(f"Publishing image switch request with ID {image_id}")
        rospy.loginfo("Waiting for subscribers to connect...")
        while self.pub.get_num_connections() == 0:
            rospy.sleep(0.1)  # Sleep for 100ms and check again
        self.pub.publish(self.msg)
        rospy.loginfo("Image switch request published")

# Hints: List for image view (mit Zahlen ändert man das Bild)
# "hi.png" -> 0
# "talk.png" -> 1
# "dish.png" -> 2
# "done.png" -> 3
# "drop.png" -> 4
# "handover.png" -> 5
# "order.png" -> 6
# "picking.png" -> 7
# "placing.png" -> 8
# "repeat.png" -> 9
# "search.png" -> 10
# "waving.mp4" -> 11
# "following" -> 12


# Example usage:
# image_switch_publisher = ImageSwitchPublisher()
# image_switch_publisher.publish_image_switch(12)
# Publishing and latching message. Press ctrl-C to terminate.
