#!/usr/bin/python3

import time

import rospy

from self_racing_car_msgs.msg import VehicleCommand

pub = rospy.Publisher("vehicle_command", VehicleCommand, queue_size=10)
rospy.init_node("talker", anonymous=True)
rate = rospy.Rate(1000)  # 1kHz


last_steering_angle = 98
next_steering_angle = 63

while True:

    msg = VehicleCommand()
    msg.steering_angle = next_steering_angle
    msg.throttle_angle = 90
    pub.publish(msg)
    print("sent message", msg.steering_angle)
    foo = last_steering_angle
    last_steering_angle = next_steering_angle
    next_steering_angle = foo
    time.sleep(5)
