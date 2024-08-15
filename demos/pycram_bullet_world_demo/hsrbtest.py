from pycram.datastructures.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.datastructures.enums import ObjectType

from pycram.ros.viz_marker_publisher import VizMarkerPublisher

world = BulletWorld()
v = VizMarkerPublisher()
robot = Object(name="hsrb", type=ObjectType.ROBOT, path="hsrb.urdf", pose=Pose([1, 2, 0]))

kitchen = Object(name="kitchen", type=ObjectType.ENVIRONMENT, path="isr-testbed.urdf")

