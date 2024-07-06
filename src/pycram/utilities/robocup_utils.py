import actionlib
import rospy
from sensor_msgs.msg import LaserScan
from sound_play.msg import SoundRequestActionGoal, SoundRequest
from std_msgs.msg import Int32
from tmc_control_msgs.msg import GripperApplyEffortActionGoal
from tmc_msgs.msg import Voice, TalkRequestAction, TalkRequestActionGoal
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

    def something_in_the_way(self):
        """
        Subscribe to the laser scan topic and wait for an obstacle to appear within a specified distance
        before continuing.
        """

        def laser_scan_callback(msg):
            ranges = list(msg.ranges)
            rospy.loginfo(f"Received laser scan with {len(ranges)} ranges")
            # Ensure there are enough elements in ranges
            if len(ranges) > 501:
                obstacle_detected = False
                # Check if any value in the range 461 to 501 is smaller than 1.0
                for i in range(460, 500):
                    #print(ranges[i])
                    if ranges[i] < 0.98:
                        obstacle_detected = True
                        rospy.loginfo(f"Obstacle detected at index {i} with range {ranges[i]}")
                        break
                if obstacle_detected:
                    self.fluent.set_value(True)
                    rospy.loginfo("Obstacle detected within 1.0 meter, unsubscribing from topic")
                    self.current_subscriber.unregister()
                    return True
                else:
                    self.fluent.set_value(True)
                    rospy.loginfo("No Obstacle detected within 1.0 meter, unsubscribing from topic")
                    self.current_subscriber.unregister()
                    return False


        rospy.loginfo("Waiting for obstacle detection signal.")
        self.current_subscriber = rospy.Subscriber("hsrb/base_scan", LaserScan, laser_scan_callback)

        rospy.loginfo("Waiting for obstacle to be detected")
        self.fluent.wait_for()

        rospy.loginfo("Obstacle detection signal received.")


class TextToSpeechPublisher:
    """
    A class to publish text-to-speech requests in a ROS environment.
    """
    #
    # def __init__(self):  #
    #     rospy.loginfo("talk init")
    #     self.client = actionlib.SimpleActionClient('talk_request_action', TalkRequestAction)
    #     rospy.loginfo("talk done")
    # def init(self):
    #     if self.client.wait_for_server():
    #         rospy.loginfo("Waiting for talk ActionServer")
    #     else:
    #         rospy.loginfo("done waiting for talk action server")
    #
    # def interrupt(self):
    #     self.client.cancel_all_goals()
    #
    # def pub_now(self, sentence, talk=True):
    #     if talk:
    #         msg = TalkRequestActionGoal()
    #         msg.goal.data.language = 1
    #         msg.goal.data.sentence = sentence
    #         self.client.send_goal(msg)
    #         wait = self.client.wait_for_result()
    #         if not wait:
    #             rospy.logerr("Action server not available!")
    #
    def __init__(self, topic='/talk_request', queue_size=100):
        """
        Initializes the TextToSpeechPublisher with a ROS publisher.

        :param topic: The ROS topic to publish text-to-speech requests to. Default is '/talk_request'.
        :param queue_size: The size of the message queue for the publisher. Default is 100.
        """
        self.pub = rospy.Publisher(topic, Voice, queue_size=queue_size)
        self.talking_sentence = ''
        rospy.Subscriber('/talking_sentence', String, self.talking_sentence_callback)

    def talking_sentence_callback(self, msg):
        """
        Callback function to update the talking_sentence attribute.

        :param msg: The ROS message received from the /talking_sentence topic.
        """
        #rospy.loginfo("Received message on /talking_sentence")
        # self.talking_sentence = msg.data

    def pub_now(self, text, talk: bool = True):
        """
        Publishes a text-to-speech request with the given text.

        :param text: The text to be spoken.
        :param talk: Boolean flag to determine whether to publish the message. Default is True.
        """
        rospy.loginfo("Preparing to publish text: " + text)
        if talk:
            # rate = rospy.Rate(10)  # 10 Hz
            # while self.talking_sentence != '':
            #     rospy.loginfo("Waiting for the current talking sentence to finish...")
            #     rate.sleep()

            text_to_speech = Voice()
            text_to_speech.language = 1
            text_to_speech.sentence = text
            rospy.loginfo("Waiting for subscribers...")
            while self.pub.get_num_connections() == 0:
                rospy.sleep(0.2)

            rospy.loginfo("Publishing text-to-speech request")
            self.pub.publish(text_to_speech)
            rospy.loginfo("Published: " + text)


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


class HSRBMoveGripperReal():
    def __init__(self, latch=True):
        self.pub = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                   queue_size=10, latch=latch)
        self.msg = GripperApplyEffortActionGoal()

    def pub_now(self, motion: str):
        if motion == "open":
            self.msg.goal.effort = 0.8
            self.pub.publish(self.msg)
        elif motion == "close":
            self.msg.goal.effort = -0.8
            self.pub.publish(self.msg)

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
