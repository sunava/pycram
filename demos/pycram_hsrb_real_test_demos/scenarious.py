import rospy

from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher

text_to_speech_publisher = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()
sound_publisher = SoundRequestPublisher()

text_to_speech_publisher.publish_text("Hey! I am Toya.")
image_switch_publisher.publish_image_switch(0)
rospy.sleep(2)
text_to_speech_publisher.publish_text("Talk to me.")
image_switch_publisher.publish_image_switch(1)
#sound_publisher.publish_sound_request()
rospy.sleep(2)
text_to_speech_publisher.publish_text("Opening the dishwasher.")
image_switch_publisher.publish_image_switch(2)
rospy.sleep(2)
text_to_speech_publisher.publish_text("I dropped the object.")
image_switch_publisher.publish_image_switch(4)
rospy.sleep(2)
text_to_speech_publisher.publish_text("Please Handover the object")
image_switch_publisher.publish_image_switch(5)
rospy.sleep(2)
text_to_speech_publisher.publish_text("What will you odering?")
image_switch_publisher.publish_image_switch(6)
rospy.sleep(2)
text_to_speech_publisher.publish_text("I am picking up the object.")
image_switch_publisher.publish_image_switch(7)
rospy.sleep(2)
text_to_speech_publisher.publish_text("I am placing the object.")
image_switch_publisher.publish_image_switch(8)
rospy.sleep(2)
text_to_speech_publisher.publish_text("Can you repeat that?")
image_switch_publisher.publish_image_switch(9)
rospy.sleep(2)
text_to_speech_publisher.publish_text("I am searching for the object.")
image_switch_publisher.publish_image_switch(10)
rospy.sleep(2)
text_to_speech_publisher.publish_text("I am done.")
image_switch_publisher.publish_image_switch(3)

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
