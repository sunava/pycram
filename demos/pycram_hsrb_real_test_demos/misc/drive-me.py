from pycram.datastructures.pose import Pose
import pycram.external_interfaces.navigate as navi

# # Initialize the necessary components
# world, v, text_to_speech_publisher, image_switch_publisher, move, robot = startup()
# giskardpy.init_giskard_interface()
# robokudo.init_robokudo_interface()
# marker = ManualMarkerPublisher()
# listener = tf.TransformListener()


import rospy

if __name__ == '__main__':
    navPose = Pose([0,0,0])
    move = navi.PoseNavigator()
    rospy.sleep(1)
    rospy.loginfo("navi")
    move.pub_now(navPose)


