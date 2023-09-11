#!/usr/bin/python3
"""
Notes:
- Currently removes first waypoint (because we use the first line to check the number of fields)
- In the interest of time, doesn't use the MarkerArray message type but Marker. To correct later.
"""

import os
import rospy

from geometry_msgs.msg import Point
from self_racing_car_msgs.msg import Waypoint, WaypointArray
from visualization_msgs.msg import Marker


class WaypointsPublisher:
    def __init__(self):

        rospy.init_node("waypoints_publisher", anonymous=True)

        """ Publishers """
        self.waypoint_pub = rospy.Publisher(
            "waypoints", WaypointArray, queue_size=10, latch=True
        )
        self.waypoint_marker_pub = rospy.Publisher(
            "waypoints_viz", Marker, queue_size=10, latch=True
        )

        """ Parameters """
        waypoints_file_name = rospy.get_param(
            "~waypoints_file", "rex_manor_parking_lot_waypoints.txt"
        )

        """ Load waypoints fields into three lists """
        list_wp_x, list_wp_y, list_wp_speed = self.load_waypoints(waypoints_file_name)

        """ Publish """
        self.publish_waypoints(list_wp_x, list_wp_y, list_wp_speed)

    def load_waypoints(self, file_name):
        """
        Function to load the waypoints from a text file into three lists (x, y, speed)
        """

        list_wp_x, list_wp_y, list_wp_speed = [], [], []

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

            nb_fields = len(waypoints_file.readline().split())
            has_speed = True if nb_fields > 2 else False

            for line in waypoints_file.readlines():

                list_wp_x.append(float(line.split()[0]))
                list_wp_y.append(float(line.split()[1]))

                if has_speed:
                    speed = float(line.split()[2])
                else:
                    speed = 0

                list_wp_speed.append(speed)

        return list_wp_x, list_wp_y, list_wp_speed

    def publish_waypoints(self, list_wp_x, list_wp_y, list_wp_speed):
        """
        Function to publish the waypoints as WaypointArray and Markers (for visualization)
        """

        """ Initialize messages """
        wp_array_msg = WaypointArray()
        # wp_marker_array_msg = MarkerArray()
        marker_msg = Marker()

        counter = 0

        """ Marker message """
        marker_msg.header.frame_id = "world"
        # marker.header.stamp = rospy.Time.now()
        marker_msg.ns = "waypoints"
        marker_msg.type = 4
        marker_msg.action = 0
        marker_msg.id = 0

        # marker scale
        marker_msg.scale.x = 0.2
        marker_msg.scale.y = 0.2

        # marker color
        marker_msg.color.r = 0.1
        marker_msg.color.g = 0.1
        marker_msg.color.b = 1
        marker_msg.color.a = 1

        marker_msg.pose.orientation.w = 1.0

        marker_msg.points = []

        for x, y, speed in zip(list_wp_x, list_wp_y, list_wp_speed):

            """Waypoint message"""
            wp_msg = Waypoint()

            wp_msg.id = counter
            wp_msg.pose.position.x = x
            wp_msg.pose.position.y = y
            wp_msg.speed_mps = speed

            counter += 1

            wp_array_msg.waypoints.append(wp_msg)

            """ Marker message """
            marker_msg.points.append(Point(x, y, 0))

            # wp_marker_array_msg.markers.append(marker_msg)

        """ Publish """
        self.waypoint_pub.publish(wp_array_msg)
        self.waypoint_marker_pub.publish(marker_msg)


if __name__ == "__main__":
    waypoint_pub = WaypointsPublisher()
    rospy.spin()
