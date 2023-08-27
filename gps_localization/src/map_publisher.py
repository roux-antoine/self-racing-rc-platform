#!/usr/bin/python3
"""
Script that reads a map (txt file with points in a 2D plane) and publishes them as markers so they can be 
visualized (in rviz, foxglove or other...)
"""

import rospkg
import rospy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class MapPublisher:
    def __init__(self):

        rospy.init_node("map_publisher", anonymous=True)

        # Publishers
        self.map_publisher = rospy.Publisher('map_markers', Marker, queue_size=10)

        # Parameters
        # Map name
        self.map_name = rospy.get_param('~map_name', 'mission_bay_1.txt')
        # Rate of publishing the markers
        self.rate = rospy.Rate(3)  

        rospy.logwarn("Map file requested: " + self.map_name)

    def create_markers_from_txt_file(self):

        marker = Marker()

        marker.header.frame_id = "world"
        # marker.header.stamp = rospy.Time.now()
        marker.ns = "points_map"
        marker.type = 4
        marker.action = 0
        marker.id = 0

        # marker scale
        marker.scale.x = 0.2
        marker.scale.y = 0.2

        # marker color
        marker.color.a = 1
        marker.color.b = 1

        # marker orientation
        marker.pose.orientation.w = 1.0

        # marker line points
        marker.points = []

        map_file = rospkg.RosPack().get_path('gps_localization') + '/maps/' + self.map_name

        with open(map_file) as f:

            lines = f.readlines()

            for line in lines:

                line = ''.join(line.splitlines())
                
                msg = line.split()

                pt = Point()
                pt.x = float(msg[0])
                pt.y = float(msg[1])
                pt.z = 0.0
                
                marker.points.append(pt)

        return marker
    

    def loop(self, marker):
        
        while not rospy.is_shutdown():

            self.map_publisher.publish(marker)

            self.rate.sleep()

if __name__ == "__main__":
    map_publisher = MapPublisher()
    marker = map_publisher.create_markers_from_txt_file()
    map_publisher.loop(marker)
    