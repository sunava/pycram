import rospy
from sensor_msgs.msg import LaserScan
import threading

from pycram.fluent import Fluent

current_subscriber = None

def wait_for_startsignal(body_func):
    def laser_scan_callback(msg):
        global current_subscriber
        ranges = list(msg.ranges)
        if len(ranges) > 481 and ranges[481] > 1.0:
            fl.set_value(True)
            rospy.loginfo("Door is open, unsubscribing from topic")
            current_subscriber.unregister()

    rospy.loginfo("Waiting for starting signal.")
    fl = Fluent()
    global current_subscriber
    current_subscriber = rospy.Subscriber("hsrb/base_scan", LaserScan, laser_scan_callback)

    rospy.loginfo("Waiting for door to open")
    fl.wait_for()

    body_func()

def body():
    rospy.loginfo("Executing body function after start signal received.")

if __name__ == "__main__":
    #rospy.init_node('wait_for_startsignal_node')
    wait_for_startsignal(body)
