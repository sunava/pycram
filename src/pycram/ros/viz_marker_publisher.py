import atexit
import threading
import time
from enum import Enum
from typing import List, Optional, Tuple

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ..datastructures.dataclasses import BoxVisualShape, CylinderVisualShape, MeshVisualShape, SphereVisualShape
from ..datastructures.pose import Pose, Transform
from ..designator import ObjectDesignatorDescription
from ..datastructures.world import World
from visualization_msgs.msg import MarkerArray, Marker
import rospy
from ..datastructures.pose import Transform, Pose

class VizMarkerPublisher:
    """
    Publishes an Array of visualization marker which represent the situation in the World
    """

    def __init__(self, topic_name="/pycram/viz_marker", interval=0.1):
        """
        The Publisher creates an Array of Visualization marker with a Marker for each link of each Object in the
        World. This Array is published with a rate of interval.

        :param topic_name: The name of the topic to which the Visualization Marker should be published.
        :param interval: The interval at which the visualization marker should be published, in seconds.
        """
        self.topic_name = topic_name
        self.interval = interval

        self.pub = rospy.Publisher(self.topic_name, MarkerArray, queue_size=10)

        self.thread = threading.Thread(target=self._publish)
        self.kill_event = threading.Event()
        self.main_world = World.current_world if not World.current_world.is_prospection_world else World.current_world.world_sync.world

        self.thread.start()
        atexit.register(self._stop_publishing)

    def _publish(self) -> None:
        """
        Constantly publishes the Marker Array. To the given topic name at a fixed rate.
        """
        while not self.kill_event.is_set():
            marker_array = self._make_marker_array()

            self.pub.publish(marker_array)
            time.sleep(self.interval)

    def _make_marker_array(self) -> MarkerArray:
        """
        Creates the Marker Array to be published. There is one Marker for link for each object in the Array, each Object
        creates a name space in the visualization Marker. The type of Visualization Marker is decided by the collision
        tag of the URDF.

        :return: An Array of Visualization Marker
        """
        marker_array = MarkerArray()
        for obj in self.main_world.objects:
            if obj.name == "floor":
                continue
            #TODOme make this as enums
            if not obj.name in ["environment", "pr2"]:
                objnew = True
            for link in obj.link_name_to_id.keys():
                geom = obj.get_link_geometry(link)
                if not geom:
                    continue
                msg = Marker()
                msg.header.frame_id = "map"
                msg.ns = obj.name
                msg.id = obj.link_name_to_id[link]
                msg.type = Marker.MESH_RESOURCE
                msg.action = Marker.ADD
                link_pose = obj.get_link_transform(link)
                if obj.get_link_origin(link) is not None:
                    link_origin = obj.get_link_origin_transform(link)
                else:
                    link_origin = Transform()
                link_pose_with_origin = link_pose * link_origin
                msg.pose = link_pose_with_origin.to_pose().pose
                color = [1, 1, 1, 1]
                # if obj.name == "board":
                #     color = [0.4, 0.2, 0.06, 1]
                # elif obj.obj_type == "object_to_be_cut":
                #     colors = {
                #         "orange": (1, 0.75, 0, 1),
                #         "cucumber": (0, 1, 0, 1),
                #         "banana": (1, 1, 0, 1),
                #         "lemon": (1, 1, 0, 1),
                #         "citron": (1, 1, 0, 1),
                #         "lime": (0.75, 1.0, 0.0, 1),
                #         "apple": (1, 0, 0, 1),
                #         "tomato": (1, 0, 0, 1),
                #         "peach": (1.0, 0.8, 0.64, 1),
                #         "kiwi": (0.76, 0.88, 0.52, 1),
                #         "avocado": (0.0, 0.5, 0.0, 1),
                #     }
                #     color = colors[obj.name]
                # else:
                #     color = [1, 1, 1, 1] if obj.link_name_to_id[link] == -1 else obj.get_link_color(link).get_rgba()
                color = [1, 1, 1, 1] if obj.link_name_to_id[link] == -1 else obj.get_link_color(link).get_rgba()
                msg.color = ColorRGBA(*color)
                msg.lifetime = rospy.Duration(1)

                if isinstance(geom, MeshVisualShape):
                    msg.type = Marker.MESH_RESOURCE
                    msg.mesh_resource = "file://" + geom.file_name
                    msg.scale = Vector3(1, 1, 1)
                    msg.mesh_use_embedded_materials = True
                elif isinstance(geom, CylinderVisualShape):
                    msg.type = Marker.CYLINDER
                    msg.scale = Vector3(geom.radius * 2, geom.radius * 2, geom.length)
                elif isinstance(geom, BoxVisualShape):
                    msg.type = Marker.CUBE
                    msg.scale = Vector3(*geom.size)
                elif isinstance(geom, SphereVisualShape):
                    msg.type = Marker.SPHERE
                    msg.scale = Vector3(geom.radius * 2, geom.radius * 2, geom.radius * 2)
                elif obj.customGeom:
                    msg.type = Marker.CUBE
                    x = geom["size"][0]
                    y = geom["size"][1]
                    z = geom["size"][2]
                    msg.scale = Vector3(x, y, z)
                if objnew:
                    #color = obj.get_color()
                    #msg.color = ColorRGBA(*color)
                    objnew = False
                marker_array.markers.append(msg)
        return marker_array

    def _stop_publishing(self) -> None:
        """
        Stops the publishing of the Visualization Marker update by setting the kill event and collecting the thread.
        """
        self.kill_event.set()
        self.thread.join()


class ManualMarkerPublisher:
    """
    Class to manually add and remove marker of objects and poses.
    """

    def __init__(self, topic_name: str = '/pycram/manual_marker', interval: float = 0.1):
        """
        The Publisher creates an Array of Visualization marker with a marker for a pose or object.
        This Array is published with a rate of interval.

        :param topic_name: Name of the marker topic
        :param interval: Interval at which the marker should be published
        """
        self.start_time = None
        self.marker_array_pub = rospy.Publisher(topic_name, MarkerArray, queue_size=10)

        self.marker_array = MarkerArray()
        self.marker_overview = {}
        self.current_id = 0

        self.interval = interval
        self.log_message = None

    def publish(self, pose: Pose, color: Optional[List] = None, bw_object: Optional[ObjectDesignatorDescription] = None,
                name: Optional[str] = None):
        """
        Publish a pose or an object into the MarkerArray.
        Priorities to add an object if possible

        :param pose: Pose of the marker
        :param color: Color of the marker if no object is given
        :param bw_object: Object to add as a marker
        :param name: Name of the marker
        """

        if color is None:
            color = [1, 0, 1, 1]

        self.start_time = time.time()
        thread = threading.Thread(target=self._publish, args=(pose, bw_object, name, color))
        thread.start()
        rospy.loginfo(self.log_message)
        thread.join()

    def _publish(self, pose: Pose, bw_object: Optional[ObjectDesignatorDescription] = None, name: Optional[str] = None,
                 color: Optional[List] = None):
        """
        Publish the marker into the MarkerArray
        """
        stop_thread = False
        duration = 2

        while not stop_thread:
            if time.time() - self.start_time > duration:
                stop_thread = True
            if bw_object is None:
                self._publish_pose(name=name, pose=pose, color=color)
            else:
                self._publish_object(name=name, pose=pose, bw_object=bw_object)

            rospy.sleep(self.interval)

    def _publish_pose(self, name: str, pose: Pose, color: Optional[List] = None):
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

        color_rgba = ColorRGBA(*color)
        self._make_marker_array(name=name, marker_type=Marker.ARROW, marker_pose=pose,
                                marker_scales=(0.05, 0.05, 0.05), color_rgba=color_rgba)
        self.marker_array_pub.publish(self.marker_array)
        self.log_message = f"Pose '{name}' published"

    def _publish_object(self, name: Optional[str], pose: Pose, bw_object: ObjectDesignatorDescription):
        """
        Publish an Object as a marker

        :param name: Name of the marker
        :param pose: Pose of the marker
        :param bw_object: ObjectDesignatorDescription for the marker
        """

        bw_real = bw_object.resolve()

        if name is None:
            name = bw_real.name

        if name in self.marker_overview.keys():
            self._update_marker(self.marker_overview[name], new_pose=pose)
            return

        path = bw_real.world_object.root_link.geometry.file_name

        self._make_marker_array(name=name, marker_type=Marker.MESH_RESOURCE, marker_pose=pose,
                                path_to_resource=path)

        self.marker_array_pub.publish(self.marker_array)
        self.log_message = f"Object '{name}' published"

    def _make_marker_array(self, name, marker_type: int, marker_pose: Pose, marker_scales: Tuple = (1.0, 1.0, 1.0),
                           color_rgba: ColorRGBA = ColorRGBA(*[1.0, 1.0, 1.0, 1.0]),
                           path_to_resource: Optional[str] = None):
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
        new_marker = Marker()
        new_marker.id = self.current_id
        new_marker.header.frame_id = frame_id
        new_marker.ns = name
        new_marker.header.stamp = rospy.Time.now()
        new_marker.type = marker_type
        new_marker.action = Marker.ADD
        new_marker.pose = marker_pose.pose
        new_marker.scale.x = marker_scales[0]
        new_marker.scale.y = marker_scales[1]
        new_marker.scale.z = marker_scales[2]
        new_marker.color.a = color_rgba.a
        new_marker.color.r = color_rgba.r
        new_marker.color.g = color_rgba.g
        new_marker.color.b = color_rgba.b

        if path_to_resource is not None:
            new_marker.mesh_resource = 'file://' + path_to_resource

        self.marker_array.markers.append(new_marker)
        self.marker_overview[name] = new_marker.id
        self.current_id += 1

    def _update_marker(self, marker_id: int, new_pose: Pose) -> bool:
        """
        Update an existing marker to a new pose

        :param marker_id: id of the marker that should be updated
        :param new_pose: Pose where the updated marker is set

        :return: True if update was successful, False otherwise
        """

        # Find the marker with the specified ID
        for marker in self.marker_array.markers:
            if marker.id == marker_id:
                # Update successful
                marker.pose = new_pose
                self.log_message = f"Marker '{marker.ns}' updated"
                self.marker_array_pub.publish(self.marker_array)
                return True

        # Update was not successful
        rospy.logwarn(f"Marker {marker_id} not found for update")
        return False

    def remove_marker(self, bw_object: Optional[ObjectDesignatorDescription] = None, name: Optional[str] = None):
        """
        Remove a marker by object or name

        :param bw_object: Object which marker should be removed
        :param name: Name of object that should be removed
        """

        if bw_object is not None:
            bw_real = bw_object.resolve()
            name = bw_real.name

        if name is None:
            rospy.logerr('No name for object given, cannot remove marker')
            return

        marker_id = self.marker_overview.pop(name)

        for marker in self.marker_array.markers:
            if marker.id == marker_id:
                marker.action = Marker.DELETE

        self.marker_array_pub.publish(self.marker_array)
        self.marker_array.markers.pop(marker_id)

        rospy.loginfo(f"Removed Marker '{name}'")

    def clear_all_marker(self):
        """
        Clear all existing markers
        """
        for marker in self.marker_array.markers:
            marker.action = Marker.DELETE

        self.marker_overview = {}
        self.marker_array_pub.publish(self.marker_array)
        rospy.loginfo('Removed all markers')


class AxisMarkerPublisher:
    def __init__(self, topic='/pycram/axis_marker', marker_id=0, frame_id='map'):
        self.topic = topic
        self.marker_id = marker_id
        self.frame_id = frame_id
        self.marker_pub = rospy.Publisher(self.topic, MarkerArray, queue_size=10)
        self.marker_array = MarkerArray()
        self.marker_pub.publish(self.marker_array)
        self.thread = threading.Thread(target=self._publish)

        self.length = None
        self.duration = None
        self.pose = None
        self.axis = None
        self.color = None

    def publish(self, pose, axis, duration=5.0, length=2.0, color=None):
        """
        Publish a MarkerArray with given pose and axis.
        Duration, length and color of the line are optional.

        :param pose: Starting position of the Line
        :param axis: Orientation for the Line
        :param duration: Duration of the marker
        :param length: Length of the line
        :param color: Color of the line if it should be personalized
        """

        if isinstance(axis, AxisIdentifier):
            axis = axis.value

        if color is None:
            color = self._get_color(axis)
        else:
            color = Colors.get_color(color)

        self.color = color
        self.axis = axis
        self.pose = pose
        self.duration = duration
        self.length = length

        self.thread.start()
        rospy.loginfo("Publishing axis visualization")
        self.thread.join()
        rospy.logdebug("Stopped Axis visualization")

    def _publish(self):
        self._create_line(self.pose, self.axis, self.duration, self.length, self.color)

        stop_thread = False
        duration = 1
        frequency = 0.2
        start_time = time.time()

        while not stop_thread:
            if time.time() - start_time > duration:
                stop_thread = True

            # Publish the MarkerArray
            self.marker_pub.publish(self.marker_array)

            rospy.sleep(frequency)
    def _get_color(self, axis):
        """
        Get the color of the given Axis

        :param axis: Used axis
        """
        if axis == AxisIdentifier.X.value:
            axis_color = Colors.get_color('red')
        elif axis == AxisIdentifier.Y.value:
            axis_color = Colors.get_color('green')
        elif axis == AxisIdentifier.Z.value:
            axis_color = Colors.get_color('blue')
        else:
            logging.error(f'Axis {str(axis)} is not valid')
            axis_color = Colors.get_color('red')

        return axis_color

    def _create_line(self, pose, axis, duration, length, color):
        """
        Create a line marker to add to the marker array.

        :param pose: Starting pose of the line
        :param axis: Axis along which the line is set
        :param duration: Duration of the line marker
        :param length: Length of the line
        :param color: Optional color for the Line
        """

        # Create a line marker for the axis
        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = 'axis_visualization'
        line_marker.id = self.marker_id
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.pose = pose
        line_marker.scale.x = 0.01  # Line width
        line_marker.color = color
        line_marker.lifetime = rospy.Duration(duration)

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
        self.marker_array.markers.append(line_marker)


class AxisIdentifier(Enum):
    X = (1, 0, 0)
    Y = (0, 1, 0)
    Z = (0, 0, 1)


class Colors:
    @staticmethod
    def get_color(color):
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
            return ColorRGBA(r=value(0), g=value(255), b=value(0), a=1.0)
        elif color == 'blue':
            return ColorRGBA(r=value(0), g=value(0), b=value(255), a=1.0)
        elif color == 'yellow':
            return ColorRGBA(r=value(255), g=value(255), b=value(0), a=1.0)
        elif color == 'cyan':
            return ColorRGBA(r=value(0), g=value(255), b=value(255), a=1.0)

        return converted_color
