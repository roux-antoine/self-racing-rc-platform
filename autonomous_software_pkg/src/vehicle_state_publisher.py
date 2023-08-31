#!/usr/bin/python3
"""
ROS node that subscribes to the NMEA sentence published by the gps serial reader node
and transforms it into two topics: gps_pose and gps_velocity
TODO: 
- Handle the situation where the location is exactly the same as in the previous message.
In this case, the best is probably to not publish
"""

import math
import rospy
import tf
import utm

from geometry_msgs.msg import PoseStamped, TwistStamped
from nmea_msgs.msg import Gprmc

class VehicleStatePublisher:
    def __init__(self):
        rospy.init_node("vehicle_state_publisher", anonymous=True)

        # Subscribers
        self.sub = rospy.Subscriber("gps_info", Gprmc, self.callback, queue_size=10)
        
        # Publishers
        self.pub_pose = rospy.Publisher("gps_pose", PoseStamped, queue_size=10)
        self.pub_velocity = rospy.Publisher("gps_velocity", TwistStamped, queue_size=10)

        self.rate = rospy.Rate(1000)

        self.last_msg_seq = None
        self.JUMPING_MESSAGE_FACTOR = 1

        rospy.logwarn("Finished init")

    def callback(self, rmc_msg):

        # small logic to skip messages if we want
        if self.last_msg_seq:
            if rmc_msg.header.seq - self.last_msg_seq < 0:
                rospy.logwarn("The header sequence id has jumped back")
                rospy.logwarn(
                    "Either you restarted the gps_publisher (or a rosbag play) or there is a problem"
                )
            elif rmc_msg.header.seq - self.last_msg_seq < self.JUMPING_MESSAGE_FACTOR:
                return

        self.last_msg_seq = rmc_msg.header.seq

        """ Read subscriber """
        latitude = rmc_msg.lat
        longitude = rmc_msg.lon
        speed_knots = rmc_msg.speed 
        track_angle_deg = rmc_msg.track

        """ Convert to utm coordinates"""
        utm_values = utm.from_latlon(latitude, longitude)
        x_utm, y_utm = utm_values[0], utm_values[1]

        """ Build the /gps_pose [PoseStamped] message """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = x_utm
        pose_msg.pose.position.y = y_utm
        pose_msg.pose.position.z = 0

        # yaw_rad = (math.pi / 2) - track_angle_deg * math.pi / 180 # Radians. Origin is horizontal axis

        # This is the angle we are using for now and for which the controller works fine. Not updating it 
        # right now, but later we might switch it to the version above, which has its origin on the horizontal axis, 
        # which should be a little more standard. The speed projection might not he good at the moment. 
        yaw_rad = -track_angle_deg * math.pi / 180

        # Convert to quaternions
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        """ Build the /gps_velocity [TwistStamped] message """
        speed_mps = speed_knots * 0.5144 # m/s

        vel_msg = TwistStamped()
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.header.frame_id = 'world'
        vel_msg.twist.linear.x = speed_mps * math.cos(yaw_rad)
        vel_msg.twist.linear.y = speed_mps * math.sin(yaw_rad)

        """ Publish /tf """
        # Publish the tf between 'world' and 'car' for visualization purposes.
        # It's easier to make the camera in foxglove follow a frame than another kind of object
        br = tf.TransformBroadcaster()
        br.sendTransform((x_utm, y_utm, 0),
                        quaternion,
                        rospy.Time.now(),
                        "car",
                        "world")    
    
        """ Publish topics """
        self.pub_pose.publish(pose_msg)
        self.pub_velocity.publish(vel_msg)


if __name__ == "__main__":
    vehicle_state_publisher = VehicleStatePublisher()
    rospy.spin()
