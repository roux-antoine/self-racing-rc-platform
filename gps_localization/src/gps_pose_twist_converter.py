#!/usr/bin/python3
"""
TODO
"""
import math
import rospy
import time
import tf
import utm

from geometry_msgs.msg import PoseStamped, TwistStamped
from self_racing_car_msgs.msg import RmcNmea

class GpsPoseTwistConverter:
    def __init__(self):

        rospy.init_node("gps_pose_twist_converter", anonymous=True)

        # Subscriber
        rospy.Subscriber("gps_info", RmcNmea, self.callback_gps)

        # Publishers
        self.pub_pose = rospy.Publisher("gps_pose", PoseStamped, queue_size=10)
        self.pub_vel  = rospy.Publisher("gps_velocity", TwistStamped, queue_size=10)

        # PARAMETERS 
        self.print_debug = False

        rospy.spin()

    def callback_gps(self, input_msg):

        """ Read subscriber """
        latitude = input_msg.latitude
        longitude = input_msg.longitude
        speed_knots = input_msg.speed_knots
        track_angle_deg = input_msg.track_angle_deg
        
        """ Convert to UTM coordinates """
        utm_values = utm.from_latlon(latitude, longitude)
        x = utm_values[0]
        y = utm_values[1]
        zone_number = utm_values[2]
        zone_letter = utm_values[3] 

        """ /gps_pose [PoseStamped] """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0

        yaw_rad = (math.pi / 2) - track_angle_deg * math.pi / 180 # Radians. Origin is horizontal axis

        # Convert to quaternions
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        self.pub_pose.publish(pose_msg)

        # Publish the tf between 'world' and 'car' for visualization purposes.
        # It's easier to make the camera in foxglove follow a frame than another kind of object
        br = tf.TransformBroadcaster()
        br.sendTransform((x, y, 0),
                        quaternion,
                        rospy.Time.now(),
                        "car",
                        "world")

        """ TWIST """
        speed_mps = speed_knots * 0.5144 # m/s

        vel_msg = TwistStamped()
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.header.frame_id = 'world'
        vel_msg.twist.linear.x = speed_mps * math.cos(yaw_rad)
        vel_msg.twist.linear.y = speed_mps * math.sin(yaw_rad)

        self.pub_vel.publish(vel_msg)

        """ DEBUGGING """
        if self.print_debug:
            print('latitude : ', latitude)
            print('longitude: ', longitude)
            print('x : ', x)
            print('y : ', y) 

            print('----------------------')

if __name__ == "__main__":
    gps_pose_publisher = GpsPoseTwistConverter()