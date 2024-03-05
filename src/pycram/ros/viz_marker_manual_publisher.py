import logging

import rospy
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

from pycram.pose import Pose


class ManualPublisher:
    def __init__(self, topic_name='/pycram/manual_marker'):
        """
        Class to manually add and remove marker of objects and poses.

        :param topic_name: Name of the marker topic
        """
        self.marker_array_pub = rospy.Publisher(topic_name, MarkerArray, queue_size=10)

        self.marker_array = MarkerArray()
        self.marker_overview = {}
        self.current_id = 0

    def publish(self, pose: Pose, color=None, bulletworld_object=None, name=None):
        """
        Publish a pose or an object into the MarkerArray.
        Priorities to add an object if possible

        :param pose: Pose of the marker
        :param color: Color of the marker if no object is given
        :param bulletworld_object: Object to add as a marker
        :param name: Name of the marker
        """
        if bulletworld_object is not None:
            self._publish_object(name=name, pose=pose, bw_object=bulletworld_object)
            return

        if color is None:
            color = 'pink'

        self._publish_pose(name=name, pose=pose, color=color)

    def _publish_pose(self, name, pose, color):
        """
        Publish a Pose as a marker

        :param name: Name of the marker
        :param pose: Pose of the marker
        :param color: Color of the marker
        """

        if name is None:
            name = 'pose_marker'

        if name in self.marker_overview.keys():
            self._update_marker(self.marker_overview[name], new_pose=pose)
            return

        color_rgba = self.get_color(color)

        self.create_marker(name=name, marker_type=Marker.ARROW, marker_pose=pose,
                           marker_scales=(0.1, 0.01, 0.01), color_rgba=color_rgba)
        self.marker_array_pub.publish(self.marker_array)

    def _publish_object(self, name, pose, bw_object):
        """
        Publish an Object as a marker

        :param name: Name of the marker
        :param pose: Pose of the marker
        :param bw_object: Bulletworld object for the marker
        """

        bw_real = bw_object.resolve()

        if name is None:
            name = bw_real.name

        if name in self.marker_overview.keys():
            self._update_marker(self.marker_overview[name], new_pose=pose)
            return

        path = bw_real.bullet_world_object.urdf_object.links[0].visual.geometry.filename

        # Create and populate your marker array here
        self.create_marker(name=name, marker_type=Marker.MESH_RESOURCE, marker_pose=pose,
                           path_to_resource=path)

        # Publish the marker array
        self.marker_array_pub.publish(self.marker_array)
        rospy.loginfo("Marker array published")

    def create_marker(self, name, marker_type, marker_pose: Pose, marker_scales=(1.0, 1.0, 1.0),
                      color_rgba=ColorRGBA(1.0, 1.0, 1.0, 1.0), path_to_resource=None):
        """
        Create a Marker and add it to the MarkerArray

        :param name: Name of the Marker
        :param marker_type: Type of the marker to create
        :param marker_pose: Pose of the marker
        :param marker_scales: individual scaling of the markers axes
        :param color_rgba: Color of the marker as RGBA
        :param path_to_resource: Path to the resource of a Bulletworld object
        """

        frame_id = marker_pose.header.frame_id
        if frame_id == 'map':
            frame_id = 'simulated/map'

        # Example Marker
        example_marker = Marker()
        example_marker.id = self.current_id
        example_marker.header.frame_id = frame_id
        example_marker.type = marker_type
        example_marker.action = Marker.ADD
        example_marker.pose.position.x = marker_pose.pose.position.x
        example_marker.pose.position.y = marker_pose.pose.position.y
        example_marker.pose.position.z = marker_pose.pose.position.z
        example_marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        example_marker.scale.x = marker_scales[0]
        example_marker.scale.y = marker_scales[1]
        example_marker.scale.z = marker_scales[2]
        example_marker.color.a = color_rgba.a
        example_marker.color.r = color_rgba.r
        example_marker.color.g = color_rgba.g
        example_marker.color.b = color_rgba.b

        if path_to_resource is not None:
            example_marker.mesh_resource = 'file://' + path_to_resource

        # Append the example marker to the array
        self.marker_array.markers.append(example_marker)
        self.marker_overview[name] = example_marker.id
        self.current_id += 1

    def _update_marker(self, marker_id, new_pose):
        """
        Update an existing marker to a new pose

        :param marker_id: id of the marker that should be updated
        :param new_pose: Pose where the updated marker is set
        """

        # Find the marker with the specified ID
        for marker in self.marker_array.markers:
            if marker.id == marker_id:
                # Update successful
                marker.pose = new_pose
                rospy.loginfo(f"Marker {marker_id} updated")
                self.marker_array_pub.publish(self.marker_array)
                return True

        # Update was not successful
        rospy.logwarn(f"Marker {marker_id} not found for update")
        return False

    def remove_marker(self, bw_object=None, name=None):
        """
        Remove a marker by object or name

        :param bw_object: Object which marker should be removed
        :param name: Name of object that should be removed
        """

        if bw_object is not None:
            bw_real = bw_object.resolve()
            name = bw_real.name

        if name is None:
            logging.error('No name for object')
            return

        marker_id = self.marker_overview.pop(name)

        for marker in self.marker_array.markers:
            if marker.id == marker_id:
                marker.action = Marker.DELETE

        self.marker_array_pub.publish(self.marker_array)
        self.marker_array.markers.pop(marker_id)
        self.marker_array_pub.publish(self.marker_array)

    def clear_all_marker(self):
        """
        Clear all existing markers
        """
        for marker in self.marker_array.markers:
            marker.action = Marker.DELETE

        self.marker_overview = {}
        self.marker_array_pub.publish(self.marker_array)

    def get_color(self, color):
        """
        Retrieve a color based on name

        :param color: Name of a color
        """

        converted_color = ColorRGBA()
        value = lambda x: x / 255

        if color == 'pink':
            return ColorRGBA(r=value(255), g=value(0), b=value(255), a=1.0)
        elif color == 'black':
            return ColorRGBA(r=value(0), g=value(0), b=value(0), a=1.0)
        elif color == 'white':
            return ColorRGBA(r=value(255), g=value(255), b=value(255), a=1.0)
        elif color == 'red':
            return ColorRGBA(r=value(255), g=value(0), b=value(0), a=1.0)
        elif color == 'green':
            return ColorRGBA(r=value(0), g=value(128), b=value(0), a=1.0)
        elif color == 'yellow':
            return ColorRGBA(r=value(255), g=value(255), b=value(0), a=1.0)
        elif color == 'cyan':
            return ColorRGBA(r=value(0), g=value(255), b=value(255), a=1.0)

        return converted_color
