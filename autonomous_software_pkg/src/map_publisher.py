#!/usr/bin/python3

import os
import rospy
import tf
import tf2_ros
from typing import List

from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker


class MapPublisher:
    def __init__(self):

        rospy.init_node("map_publisher", anonymous=True)

        """ Publishers """
        self.map_marker_pub = rospy.Publisher(
            "map_viz", Marker, queue_size=10, latch=True
        )

        """ Parameters """
        map_file_name = rospy.get_param("~map_file_name", "rex_manor_map.txt")

        """ Load waypoints fields into three lists """
        list_map_x, list_map_y = self.load_map(map_file_name)

        """ Publish """
        self.publish_map(list_map_x, list_map_y)
        self.publish_tf_world_map(list_map_x[0], list_map_y[0])

    def load_map(self, file_name: str):
        """
        Function to load the waypoints from a text file into two lists (x, y)
        """

        list_wp_x, list_wp_y = [], []

        """ Find file path """
        waypoint_file_folder_path = os.path.join(
            os.path.dirname(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            ),
            "utils/utm_map_generation/x_y_files",
        )

        wp_file = os.path.join(waypoint_file_folder_path, file_name)

        """ Open and read file """
        with open(wp_file) as waypoints_file:

            for line in waypoints_file.readlines():

                list_wp_x.append(float(line.split()[0]))
                list_wp_y.append(float(line.split()[1]))

        return list_wp_x, list_wp_y

    def publish_map(self, list_wp_x: List[float], list_wp_y: List[float]):
        """
        Function to publish the waypoints Markers
        """

        """ Marker message """
        marker_msg = Marker()

        marker_msg.header.frame_id = "world"
        marker_msg.ns = "map"
        marker_msg.type = 4
        marker_msg.action = 0
        marker_msg.id = 0

        # marker scale
        marker_msg.scale.x = 0.2
        marker_msg.scale.y = 0.2

        # marker color
        marker_msg.color.r = 0.5
        marker_msg.color.g = 0.5
        marker_msg.color.b = 0.5
        marker_msg.color.a = 1

        marker_msg.pose.orientation.w = 1.0

        marker_msg.points = []

        for x, y in zip(list_wp_x, list_wp_y):

            marker_msg.points.append(Point(x, y, 0))

        """ Publish """
        self.map_marker_pub.publish(marker_msg)

    def publish_tf_world_map(self, x_origin: float, y_origin: float):
        """
        Function to publish the tf between world and map on the topic /tf_static
        """

        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transform = TransformStamped()

        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = "world"
        static_transform.child_frame_id = "map"

        static_transform.transform.translation.x = x_origin
        static_transform.transform.translation.y = y_origin
        static_transform.transform.translation.z = 0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)

        static_transform.transform.rotation.x = quaternion[0]
        static_transform.transform.rotation.y = quaternion[1]
        static_transform.transform.rotation.z = quaternion[2]
        static_transform.transform.rotation.w = quaternion[3]

        broadcaster.sendTransform(static_transform)


if __name__ == "__main__":
    map_pub = MapPublisher()
    rospy.spin()
