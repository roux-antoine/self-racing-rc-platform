#!/usr/bin/python3
"""
ROS node that subscribes to the NMEA sentence published by the gps serial reader node
and transforms it into two topics: current_pose and current_velocity
TODO:
- Handle the situation where the location is exactly the same as in the previous message.
In this case, the best is probably to not publish
"""

import math
import numpy as np
import rospy
import tf
import time
import utm

from geometry_msgs.msg import PoseStamped, TwistStamped
from nmea_msgs.msg import Gprmc
from self_racing_car_msgs.msg import ArduinoLogging


class VehicleStatePublisher:
    def __init__(self):
        rospy.init_node("vehicle_state_publisher", anonymous=True)

        # Subscribers
        self.gps_info_sub = rospy.Subscriber(
            "gps_info", Gprmc, self.callback_rmc_msg, queue_size=10
        )
        self.arduino_logging_sub = rospy.Subscriber(
            "arduino_logging",
            ArduinoLogging,
            self.callback_arduino_logging,
            queue_size=10,
        )

        # Publishers
        self.pub_pose = rospy.Publisher("current_pose", PoseStamped, queue_size=10)
        self.pub_velocity = rospy.Publisher(
            "current_velocity", TwistStamped, queue_size=10
        )
        self.future_pub_pose = rospy.Publisher(
            "future_pose", PoseStamped, queue_size=10
        )

        self.rate = rospy.Rate(1000)

        self.last_msg_seq = None
        self.JUMPING_MESSAGE_FACTOR = 1
        self._last_arduino_logging_msg = None
        self._last_t_arduino_logging_msg = time.time()

        rospy.logwarn("Finished init")

    def callback_arduino_logging(self, arduino_msg: ArduinoLogging):
        self._last_arduino_logging_msg = arduino_msg
        self._last_t_arduino_logging_msg = time.time()

    def callback_rmc_msg(self, rmc_msg: Gprmc):

        # small logic to skip messages if we want
        if self.last_msg_seq:
            if rmc_msg.header.seq - self.last_msg_seq < 0:
                rospy.logwarn("The header sequence id has jumped back")
                rospy.logwarn(
                    "Either you restarted the gps_publisher (or a rosbag play) or there is a problem"
                )
            elif rmc_msg.header.seq - self.last_msg_seq < self.JUMPING_MESSAGE_FACTOR:
                # This condition can be used to skip every N message if we ever want.
                # To do so, increase self.JUMPING_MESSAGE_FACTOR to more than 1
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

        """ Build the /current_pose [PoseStamped] message """
        current_pose_msg = PoseStamped()
        current_pose_msg.header.stamp = rospy.Time.now()
        current_pose_msg.header.frame_id = "world"
        current_pose_msg.pose.position.x = x_utm
        current_pose_msg.pose.position.y = y_utm
        current_pose_msg.pose.position.z = 0

        # Yaw angle [radians]. Origin is horizontal axis. Counter clockwise
        # Offsetting by pi/2 compared to what is provided by the GPS, because the GPS track angle origin is the vertical axis
        yaw_rad = (math.pi / 2) - track_angle_deg * math.pi / 180

        # Convert to quaternions
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)
        current_pose_msg.pose.orientation.x = quaternion[0]
        current_pose_msg.pose.orientation.y = quaternion[1]
        current_pose_msg.pose.orientation.z = quaternion[2]
        current_pose_msg.pose.orientation.w = quaternion[3]

        """ Build the /current_velocity [TwistStamped] message """
        speed_mps = speed_knots * 0.5144  # m/s

        vel_msg = TwistStamped()
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.header.frame_id = "car"
        vel_msg.twist.linear.x = speed_mps

        """ Publish /tf """
        # Publish the tf between 'world' and 'car' for visualization purposes.
        # It's easier to make the camera in foxglove follow a frame than another kind of object
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (x_utm, y_utm, 0), quaternion, rospy.Time.now(), "car", "world"
        )

        """ Publish topics """
        self.pub_pose.publish(current_pose_msg)
        self.pub_velocity.publish(vel_msg)

        # computed the predicted pose
        dt = 0.1  # s
        future_pose_msg = PoseStamped()
        future_pose_msg.header.stamp = (
            rospy.Time.now()
        )  # TODO figure out if this causes a problem
        future_pose_msg.header.frame_id = "world"

        # model 1
        # future_pose_msg.pose.position.x = x_utm + speed_mps * math.cos(yaw_rad) * dt
        # future_pose_msg.pose.position.y = y_utm + speed_mps * math.sin(yaw_rad) * dt
        # future_pose_msg.pose.position.z = 0
        # future_pose_msg.pose.orientation.x = quaternion[0]
        # future_pose_msg.pose.orientation.y = quaternion[1]
        # future_pose_msg.pose.orientation.z = quaternion[2]
        # future_pose_msg.pose.orientation.w = quaternion[3]

        # model 2, see the file future_state_computer.py for explanations
        if self._last_arduino_logging_msg is not None and (
            time.time() - self._last_t_arduino_logging_msg < 0.2
        ):
            steering_diff = self._last_arduino_logging_msg.steering_cmd_final - 98
            if steering_diff != 0:
                R = 15 / steering_diff
            else:
                R = 10000  # big number
        else:
            R = 10000  # big number

        future_pose_msg.pose.position.x = (
            x_utm
            + np.cos(yaw_rad + np.pi / 2) * R * (np.cos(speed_mps / -R * dt) - 1)
            - np.sin(yaw_rad + np.pi / 2) * R * np.sin(speed_mps / -R * dt)
        )
        future_pose_msg.pose.position.y = (
            y_utm
            + np.sin(yaw_rad + np.pi / 2) * R * (np.cos(speed_mps / -R * dt) - 1)
            + np.cos(yaw_rad + np.pi / 2) * R * np.sin(speed_mps / -R * dt)
        )
        future_pose_msg.pose.position.z = 0
        future_pose_msg.pose.orientation.x = quaternion[0]
        future_pose_msg.pose.orientation.y = quaternion[1]
        future_pose_msg.pose.orientation.z = quaternion[2]
        future_pose_msg.pose.orientation.w = quaternion[3]

        self.future_pub_pose.publish(future_pose_msg)


if __name__ == "__main__":
    vehicle_state_publisher = VehicleStatePublisher()
    rospy.spin()
