#!/usr/bin/env python
import rospy
import cv2

from threading import Thread
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class HeadDisplayPublisher:

    def __init__(self, topic="/head_display/receiver"):
        self.bridge = CvBridge()

        self.image_path = None
        self.frequency = None

        # Create a publisher for the image
        self.publisher = rospy.Publisher(topic, Image, queue_size=10)
        self.thread = Thread(target=self._pub)
        print("publisher started")

    def publish_image(self, path, publish_constantly=True, frequency=0.3):
        self.image_path = path
        self.frequency = frequency

        if publish_constantly:
            self.thread.start()
        else:
            self._pub()

    def stop_publishing(self):
        if self.thread.is_alive():
            self.thread.join()
        else:
            rospy.logwarn("Thread is not running")

    def _pub(self):
        image = cv2.imread(self.image_path)

        if image is not None:
            # Convert the OpenCV image to a ROS image message
            ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")

            # Publish the ROS image message
            self.publisher.publish(ros_image)

            rospy.logdebug("published image for head display")
            rospy.sleep(self.frequency)
        else:
            rospy.logerr("Failed to load image: %s", self.image_path)
