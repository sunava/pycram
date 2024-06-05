from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher

text_to_speech_publisher = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()


text_to_speech_publisher.publish_text("Hey! I am Toya.")
image_switch_publisher.publish_image_switch(0)
text_to_speech_publisher.publish_text("Talk to me.")
image_switch_publisher.publish_image_switch(1)
text_to_speech_publisher.publish_text("Opening the dishwasher.")
image_switch_publisher.publish_image_switch(2)
text_to_speech_publisher.publish_text("I am done.")
image_switch_publisher.publish_image_switch(3)
text_to_speech_publisher.publish_text("I dropped the object.")
image_switch_publisher.publish_image_switch(4)
text_to_speech_publisher.publish_text("Please Handover the object")
image_switch_publisher.publish_image_switch(5)
text_to_speech_publisher.publish_text("What will you odering?")
image_switch_publisher.publish_image_switch(6)
text_to_speech_publisher.publish_text("I am picking up the object.")
image_switch_publisher.publish_image_switch(7)
text_to_speech_publisher.publish_text("I am placing the object.")
image_switch_publisher.publish_image_switch(8)
text_to_speech_publisher.publish_text("Can you repeat that?")
image_switch_publisher.publish_image_switch(9)
text_to_speech_publisher.publish_text("I am searching for the object.")
image_switch_publisher.publish_image_switch(10)

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
