#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from self_racing_car_msgs.msg import RmcNmea
#from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix

def callback(gps_data):
    print("\n")
    print("Latitude: " + str(gps_data.latitude))
    print("Longitude: " + str(gps_data.longitude))
    print("Speed: " + str(gps_data.speed_knots))

    msg = NavSatFix()
    msg.latitude = gps_data.latitude
    msg.longitude = gps_data.longitude

    pub = rospy.Publisher('gps_fix', NavSatFix, queue_size=10)
    rate = rospy.Rate(10)
    pub.publish(msg)
    rate.sleep()

def listener():
    
    rospy.init_node('bag_listener', anonymous=True)
    rospy.Subscriber("gps_info", RmcNmea, callback)
    

    rospy.spin()

if __name__ == '__main__':
    
    listener()

