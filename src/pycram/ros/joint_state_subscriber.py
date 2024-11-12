import rospy
from sensor_msgs.msg import JointState

class JointAngleReader:
    def __init__(self):
        # Dictionary to store the latest joint angles
        self.joint_angles = {}

        # Subscriber to the /joint_states topic
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        # Update the joint angles dictionary with the latest positions
        for joint_name, joint_position in zip(msg.name, msg.position):
            self.joint_angles[joint_name] = joint_position

    def get_joint_angle(self, joint_name):
        # Return the angle of the specified joint if it exists, otherwise None
        return self.joint_angles.get(joint_name, None)
