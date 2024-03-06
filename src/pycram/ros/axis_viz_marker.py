import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class AxisVisualization:
    def __init__(self, topic='/axis_visualization', marker_id=0, frame_id='map'):
        self.topic = topic
        self.marker_id = marker_id
        self.frame_id = frame_id
        self.marker_pub = rospy.Publisher(self.topic, MarkerArray, queue_size=10)

    def publish(self, pose, axis):
        marker_array = MarkerArray()

        length = 3

        # Create a line marker for the axis
        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = 'axis_visualization'
        line_marker.id = self.marker_id
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.pose = pose
        line_marker.scale.x = 0.05  # Line width
        line_marker.color.a = 1.0
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0

        # Create two points for the line (start and end)
        start_point = Point()
        start_point.x = pose.position.x
        start_point.y = pose.position.y
        start_point.z = pose.position.z

        end_point = Point()
        end_point.x = pose.position.x + (axis[0] * length)
        end_point.y = pose.position.y + (axis[1] * length)
        end_point.z = pose.position.z + (axis[2] * length)

        line_marker.points.append(start_point)
        line_marker.points.append(end_point)

        # Add the line marker to the MarkerArray
        marker_array.markers.append(line_marker)

        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)
