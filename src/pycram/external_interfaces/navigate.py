import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

move_client = None


def interrupt():
    global move_client
    move_client.cancel_all_goals()

class PoseNavigator():
    def __init__(self):
        global move_client

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        move_client = self.client

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
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()
