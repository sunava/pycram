from pycram.utilities.robocup_utils import TextToSpeechPublisher

# Create an instance of the TextToSpeechPublisher
text_to_speech_publisher = TextToSpeechPublisher()

# Publish a text-to-speech request
text_to_speech_publisher.publish_text("Hello, world!")
