from enum import Enum

import rospy

from pycram.process_module import real_robot, semi_real_robot, simulated_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from demos.pycram_serve_breakfast_demo.utils.misc import *
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np
import cv2
#ärospy.init_node('pybullet_simulation_node', anonymous=True)
world = BulletWorld("DIRECT")
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1, 2, 0]))

# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "isr-testbed.urdf")
