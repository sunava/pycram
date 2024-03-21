import rospy
from std_msgs.msg import String, Bool
import demos.pycram_receptionist_demo.utils.misc as misc


def doorbell_fct():

    rospy.Subscriber("doorbell", Bool, misc.doorbell_cb)
    while not misc.doorbell:
        rospy.sleep(1)

    print("doorbell sound was heard!")



doorbell_fct()