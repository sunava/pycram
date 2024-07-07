import math

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from pycram.fluent import Fluent

move_client = None


def interrupt():
    global move_client
    move_client.cancel_all_goals()

class PoseNavigator():
    def __init__(self):#
        rospy.loginfo("move_base init")
        global move_client
        self.client = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)
        move_client = self.client
        rospy.loginfo("move_base init construct done")
    def init(self):
        rospy.loginfo("Waiting for move_base ActionServer")
        if self.client.wait_for_server():
            rospy.loginfo("Done")
        else:
            rospy.loginfo("done waiting for move_base")


    def interrupt(self):
        self.client.cancel_all_goals()

    def pub_now(self, navpose):
        rospy.logerr("New implementation!")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose = navpose

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")


# class PoseNavigator():
#     def __init__(self, latch=True):
#         self.pub = rospy.Publisher('goal', PoseStamped,
#                                    queue_size=10, latch=latch)
#         self.toya_pose = Fluent()
#         self.human_pose = None
#         self.toya_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.toya_pose_cb)
#         self.goal_pose = None
#
#     def toya_pose_cb(self, msg):
#         # print("updating")
#
#         self.toya_pose.set_value(msg.pose.pose.position)
#         rospy.sleep(0.1)
#
#     def pub_now(self, nav_pose: PoseStamped):
#         self.goal_pose = nav_pose
#         ps = PoseStamped()
#         ps.pose = nav_pose.pose
#         ps.header = nav_pose.header
#
#         rospy.loginfo(f"Publishing navigation pose")
#         rospy.loginfo("Waiting for subscribers to connect...")
#         while self.pub.get_num_connections() == 0:
#             rospy.sleep(0.1)  # Sleep for 100ms and check again
#         self.pub.publish(ps)
#
#         near_goal = False
#         rospy.loginfo("Pose was published")
#         while not near_goal:
#             dis = math.sqrt((self.goal_pose.pose.position.x - self.toya_pose.get_value().x) ** 2 +
#                             (self.goal_pose.pose.position.y - self.toya_pose.get_value().y) ** 2)
#             print("dis: " + str(dis))
#             if dis < 0.3:
#                 near_goal = True
#                 rospy.logwarn("Near Pose")
#                 break
#             else:
#                 rospy.logwarn("Waiting for Toya to drive to pose")


