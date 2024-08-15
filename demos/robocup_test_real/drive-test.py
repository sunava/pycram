from pycram.datastructures.pose import Pose
import pycram.external_interfaces.navigate as navi

# # Initialize the necessary components
# world, v, text_to_speech_publisher, image_switch_publisher, move, robot = startup()
# giskardpy.init_giskard_interface()
# robokudo.init_robokudo_interface()
# marker = ManualMarkerPublisher()
# listener = tf.TransformListener()


import rospy

start_pose = Pose([1.5, 0.5, 0])

door_pose = Pose([3, 1.0, 0],[0, 0, 1, 0])
convo_pose = Pose([4.2, 0.15, 0],[0, 0, 1, 0])
convo_pose_to_couch = Pose([4.2, 0.15, 0],[0, 0, 0, 1])
couch_pose = Pose([8.5, 0, 0],[0, 0, 0, 1])
couch_pose_to_door = Pose([8.6, 0, 0],[0, 0, 1, 0])
diswasher = Pose([9, 3, 0],[0, 0, 0.7, 0.7])  # ori
schreibtisch = Pose([8.6, 0, 0],[0, 0, 1, 0])
schreibtisch_to_eingang = Pose([4.8, 1.1, 0],[0, 0, -0.7, 0.7])

long_table_1 = Pose([6.65, 4.6, 0],[0, 0, 0, 1])
long_table_1_rotated = Pose([6.65, 4.6, 0],[0, 0, 1, 0])
shelf_1 = Pose([6, 6, 0],[0, 0, 1, 0])
shelf_1_rotated = Pose([6, 6, 0],[0, 0, 0, 1])
if __name__ == '__main__':
    move = navi.PoseNavigator()
    rospy.sleep(1)
    rospy.loginfo("navi")
    #move.pub_now(convo_pose_to_couch)
    #move.pub_now(door_pose, interrupt_bool=False)
    #move.pub_now(convo_pose)
    #move.pub_now(convo_pose_to_couch, interrupt_bool=False)
    #
    move.pub_now(couch_pose)
    # move.pub_now(couch_pose_to_door)
    # move.pub_now(couch_pose)
    #
    # move.pub_now(diswasher)
    # move.pub_now(schreibtisch)
    # move.pub_now(schreibtisch_to_eingang)
