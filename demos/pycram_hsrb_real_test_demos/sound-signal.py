from pycram.designators.action_designator import *
from demos.pycram_receptionist_demo.utils.new_misc import *
from pycram.enums import ObjectType
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool
from sound_play.msg import SoundRequestActionGoal, SoundRequest, SoundRequestAction, SoundRequestGoal
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID

# variables for communcation with nlp
pub = rospy.Publisher('/sound_play/goal', SoundRequestActionGoal, queue_size=10, latch=True)
msg = SoundRequestActionGoal()
msg.goal.sound_request.sound = 1
msg.goal.sound_request.command = 1
msg.goal.sound_request.volume = 4.0

print(msg)
# Publish the message
rospy.loginfo("Publishing sound request")
# Wait for subscribers to connect
rospy.loginfo("Waiting for subscribers to connect...")
while pub.get_num_connections() == 0:
    rospy.sleep(0.1)  # Sleep for 100ms and check again

pub.publish(msg)

# Keep the node running for a while to ensure the message is sent

