#!/usr/bin/env python3
import rospy
import tf
import math
import utm

from std_msgs.msg import String
from self_racing_car_msgs.msg import RmcNmea
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped


class bag_listener:
    # - Converts RmcNmea gps messages to NavStatFix messages which are the standard for Mapviz
    # - Broadcasts transform for Mapviz to follow target frame (Car)

    def __init__(self):

        #Initialize publisher that NavSatFix publisher and RmcNmea Subscriber
        self.pub = rospy.Publisher('gps_fix', NavSatFix, queue_size=10)
        self.sub = rospy.Subscriber("gps_info", RmcNmea, self.callback)
        self.pub_pose = rospy.Publisher("gps_pose", PoseStamped, queue_size=10)
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10)

        rospy.logwarn("Finished init")

    def callback(self, gps_data):

        #Extract RmcNmea message
        latitude = gps_data.latitude
        longitude = gps_data.longitude
        speed_knots = gps_data.speed_knots
        track_angle_deg = gps_data.track_angle_deg
        print("\n")
        print("Latitude: " + str(latitude))
        print("Longitude: " + str(longitude))
        print("Speed: " + str(speed_knots))
        print("Track Angle (Degrees): " + str(track_angle_deg))


        #MUST POPULATE THESE WITH KNOWN STARTING COORDINATES
        LAT_START = 36.585825
        LON_START = -121.757031

        #Convert from latitude/longitude to UTM
        utm_values = utm.from_latlon(latitude, longitude)
        x_utm, y_utm = utm_values[0], utm_values[1]

        utm2 = utm.from_latlon(LAT_START, LON_START)
        x_orig_utm, y_orig_utm = utm2[0], utm2[1]
        
        print("x_utm: " + str(x_utm))
        print("y_utm: " + str(y_utm))

        #Create Pose message and populate with utm position
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

        #Broadcast transform as "car" frame
        self.br.sendTransform((utmx_fin, utmy_fin, 0), quaternion, rospy.Time.now(), "car", "origin")



        #Create NavSatFix message and populate with gps_data
        navsat_msg = NavSatFix()
        navsat_msg.latitude = latitude
        navsat_msg.longitude = longitude

        self.pub.publish(navsat_msg)
        self.rate.sleep()


if __name__ == '__main__':

    rospy.init_node("bag_listener", anonymous=True)
    bl = bag_listener()
    rospy.spin()

