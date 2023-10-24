#!/usr/bin/python3

import os
import rospy
import tf
import tf2_ros
import utm

from enum import Enum
from pykml import parser

from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray


class PlacemarkType(Enum):
    Contour = 1
    Lane = 2
    Obstacle = 3


class MapComponent:
    numInstances = 0

    def __init__(
        self,
        type: PlacemarkType = PlacemarkType.Contour,
        list_coordinates: list = [],
    ):
        self.id = MapComponent.numInstances
        MapComponent.numInstances += 1
        self.type = type
        self.list_coordinates = list_coordinates


class MapPublisher:
    def __init__(self):

        rospy.init_node("map_publisher", anonymous=True)

        # Publisher
        self.map_marker_pub = rospy.Publisher(
            "map_viz", MarkerArray, queue_size=10, latch=True
        )

        # Parameters
        self.map_file_name = rospy.get_param("~map_file_name", "MapSanMateoP1.kml")

    def load_map(self):
        """
        Function to parse a kml file into a list of MapComponent, each defining a portion of the map
        Returns:
            - list_components: List of MapComponent objects
        """

        waypoint_file_folder_path = os.path.join(
            os.path.dirname(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            ),
            "utils/utm_map_generation/kml_files",
        )

        wp_file = os.path.join(waypoint_file_folder_path, self.map_file_name)

        list_components = []

        with open(wp_file) as f:

            root = parser.parse(f).getroot()

            for p in root.Document.Placemark:

                component = MapComponent()
                component.list_coordinates = []

                # Classify type of component using its name
                name = p.name.text

                if "obstacle" in name:
                    component.type = PlacemarkType.Obstacle

                elif "contour" in name:
                    component.type = PlacemarkType.Contour

                elif "line" in name:
                    component.type = PlacemarkType.Lane

                # Extract coordinates from component (is different depending on geometry of component)
                if hasattr(p, "Polygon"):
                    coordinates = p.Polygon.outerBoundaryIs.LinearRing.coordinates

                if hasattr(p, "LineString"):
                    coordinates = p.LineString.coordinates

                # Convert to string
                coordinates_string = str(coordinates)

                # Go through coordinates, convert them to utm and add them to component object
                for x_y in coordinates_string.split():

                    lon = float((x_y.split(","))[0])
                    lat = float((x_y.split(","))[1])

                    utm_values = utm.from_latlon(lat, lon)
                    x_utm, y_utm = utm_values[0], utm_values[1]

                    component.list_coordinates.append((x_utm, y_utm))

                # Add map component to list of map components
                list_components.append(component)

        return list_components

    def publish_map(self, map):
        """
        Function to publish a MarkerArray topic, containing Markers each representing a portion of the map
        """

        markerArray = MarkerArray()

        for count, component in enumerate(map):

            marker = Marker()
            marker.header.frame_id = "world"
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0

            marker.pose.orientation.w = 1.0
            marker.pose.position.x = 0
            marker.pose.position.y = 0
            marker.pose.position.z = 0

            if component.type == PlacemarkType.Obstacle:
                marker.color.r = 0.89
                marker.color.g = 0.05
                marker.color.b = 0.19

            elif component.type == PlacemarkType.Lane:
                marker.color.r = 0.08
                marker.color.g = 0.63
                marker.color.b = 0.09

            elif component.type == PlacemarkType.Contour:
                marker.color.r = 0.3
                marker.color.g = 0.3
                marker.color.b = 0.3

            marker.id = count

            for x_y in component.list_coordinates:
                p = Point()
                p.x = x_y[0]
                p.y = x_y[1]
                marker.points.append(p)

            markerArray.markers.append(marker)

        self.map_marker_pub.publish(markerArray)

    def publish_tf_world_map(self, map):
        """
        Function to publish the tf between world and map on the topic /tf_static
        """

        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transform = TransformStamped()

        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = "world"
        static_transform.child_frame_id = "map"

        static_transform.transform.translation.x = (map[0].list_coordinates)[0][0]
        static_transform.transform.translation.y = (map[0].list_coordinates)[0][1]
        static_transform.transform.translation.z = 0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)

        static_transform.transform.rotation.x = quaternion[0]
        static_transform.transform.rotation.y = quaternion[1]
        static_transform.transform.rotation.z = quaternion[2]
        static_transform.transform.rotation.w = quaternion[3]

        broadcaster.sendTransform(static_transform)


if __name__ == "__main__":
    try:
        map_pub = MapPublisher()

        # Load waypoints
        map = map_pub.load_map()

        # Publish map markers and tf
        map_pub.publish_map(map)
        map_pub.publish_tf_world_map(map)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
