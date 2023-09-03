#!/usr/bin/python3

import rospy
import tf
import math
import utm
from geometry_msgs.msg import PoseStamped, TwistStamped
from self_racing_car_msgs.msg import RmcNmea

class VsP:
    def __init__(self):
        rospy.init_node("v_s_p", anonymous=True)

        self.sub = rospy.Subscriber("gps_info", RmcNmea, self.callback,queue_size=10)

        self.pub_pose = rospy.Publisher("gps_pose", PoseStamped, queue_size=10)

        self.rate = rospy.Rate(10)

        rospy.logwarn("Finished init")

    def callback(self, rmc_msg):
        
        latitude = rmc_msg.latitude
        longitude = rmc_msg.longitude
        speed_knots = rmc_msg.speed_knots
        track_angle_deg = rmc_msg.track_angle_deg

        utm_values = utm.from_latlon(latitude, longitude)
        x_utm, y_utm = utm_values[0], utm_values[1]
        
        utm2 = utm.from_latlon(36.585825, -121.757031)
        x_orig_utm, y_orig_utm = utm2[0], utm2[1]

        print("latitude: " + str(latitude))
        print("longitude: " + str(longitude))
        
        print("x_utm: " + str(x_utm))
        print("y_utm: " + str(y_utm))
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = x_utm
        pose_msg.pose.position.y = y_utm
        pose_msg.pose.position.z = 0

        yaw_rad = -track_angle_deg * math.pi / 180

        utmx_fin = x_utm - x_orig_utm
        utmy_fin = y_utm - y_orig_utm
        print("finx: " + str(utmx_fin))
        print("finy: " + str(utmy_fin))
        # Convert to quaternions
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        br = tf.TransformBroadcaster()

        br.sendTransform((utmx_fin, utmy_fin, 0), quaternion, rospy.Time.now(), "car", "origin")

if __name__ == "__main__":
    vehicle_state_publisher = VsP()
    rospy.spin()