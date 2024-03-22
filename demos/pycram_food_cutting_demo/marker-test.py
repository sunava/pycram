from datetime import time

import ipywidgets as widgets
import rospy
from IPython.display import display
from ipywidgets import HBox
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.ros.viz_marker_manual_publisher import VizMarkerManualPublisher
import ipywidgets as widgets
from ipywidgets import HBox, Button, Output
from IPython.display import display, clear_output, HTML
import rospkg
from pycram.context_knowledge import ContextConfig, generate_context

#robot = Object("hsrb", ObjectType.ROBOT, "hsrb.urdf", pose=Pose([1, 2, 0]))
#apartment = Object("apartment", ObjectType.ENVIRONMENT, "apartment.urdf")
#robot = Object("hsrb", ObjectType.ROBOT, "hsrb.urdf", pose=Pose([1, 2, 0]))
#apartment = Object("apartment", "environment", "apartment.urdf")


def cutting(obj="cucumber.stl", technique="slicing"):
    location_pose = Pose([1.7, 2, 1])
    v = VizMarkerManualPublisher()
    print("frst")
    v.publish(location_pose)
    rospy.sleep(10)
    print("second")
    v.publish(location_pose)
    location_pose1 = Pose([1.7, 2, 0.5])
    v.publish(location_pose1, name="location_pose1")
    print("third")
    rospy.sleep(10)
    v.publish(location_pose1, name="location_pose1")

    current_context = generate_context("cutting-init", "apartment-small.urdf")
    cutting_tool = current_context.get_cutting_tool()

cutting()
