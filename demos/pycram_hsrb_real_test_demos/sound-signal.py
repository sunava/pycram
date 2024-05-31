# Create an instance of the SoundRequestPublisher
from pycram.utilities.robocup_utils import SoundRequestPublisher

sound_publisher = SoundRequestPublisher()

# Publish a sound request
sound_publisher.publish_sound_request()
