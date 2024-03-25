#!/usr/bin/python3
# TODO: Add documentation and diagrams

from datetime import datetime, timezone
import math
import numpy as np
import rospy
import tf
import time
import utm

from dynamic_reconfigure.server import Server
from dynamic_reconfigure_pkg.cfg import vehicle_simConfig
from geometry_msgs.msg import PoseStamped, TwistStamped
from geometry_utils_pkg.geometry_utils import (
    compute_steering_angle_from_curvature,
    State,
)
from nmea_msgs.msg import Gprmc
from self_racing_car_msgs.msg import ArduinoLogging
from std_msgs.msg import Float64, Float32
from vehicle_sim import LateralControlModel, LongitudinalControlModel, VehicleSim

MPS_TO_KNOTS = 1.94384


class Sim:
    def __init__(self):

        # Constants
        rate = rospy.get_param("~rate", 10.0)
        self.time_step_secs = rospy.get_param(
            "~time_step_secs", 0.1
        )  # Ideally calculate that from rate above

        # Subscribers
        rospy.Subscriber(
            "initialpose",
            PoseStamped,
            self.clicked_point_callback,
        )
        rospy.Subscriber(
            "throttle_pwm_cmd",
            Float32,
            self.throttle_pwm_callback,
        )
        rospy.Subscriber(
            "target_curvature",
            Float64,
            self.target_curvature_callback,
        )

        # Publishers
        self.sim_pose_pub = rospy.Publisher(
            "current_pose_sim",
            PoseStamped,
            queue_size=10,
        )
        self.sim_twist_pub = rospy.Publisher(
            "current_velocity_sim",
            TwistStamped,
            queue_size=10,
        )
        self.rmc_pub = rospy.Publisher("gps_info", Gprmc, queue_size=10)
        self.arduino_log_pub = rospy.Publisher(
            "arduino_logging", ArduinoLogging, queue_size=10
        )

        # Dynamic reconfigure server
        self.dynamic_reconfigure_server = Server(
            vehicle_simConfig,
            self.dynamic_reconfigure_callback,
        )

        # Initialization
        longitudinal_model = LongitudinalControlModel(
            tau_delay=1,
            noise=0,
        )

        lateral_model = LateralControlModel(
            wheelbase=self.wheelbase,
        )

        self.simulated_vehicle = VehicleSim(
            lateral_model=lateral_model,
            longitudinal_model=longitudinal_model,
        )

        self.state_initialized = False
        self.target_curvature = float(1 / self.max_radius)
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(rate)
        self.throttle_pwm_cmd = None

    def dynamic_reconfigure_callback(self, config, level):
        self.wheelbase = config["wheelbase"]
        self.tau_delay = config["tau_delay"]
        self.noise = config["noise"]
        self.max_radius = config["max_radius"]
        self.timeout_pwm_msg_before_stop_secs = config[
            "timeout_pwm_msg_before_stop_secs"
        ]
        self.set_arduino_log_engage = config["set_arduino_log_engage"]

        try:
            self.simulated_vehicle.lateral_model.update_wheelbase(config["wheelbase"])
            self.simulated_vehicle.longitudinal_model.update_tau_delay(
                config["tau_delay"]
            )
            self.simulated_vehicle.longitudinal_model.update_noise(config["noise"])
        except AttributeError:
            rospy.logwarn(
                "Sim vehicle object doesn't exist yet. Can't update its wheelbase."
            )

        return config

    def clicked_point_callback(self, msg: PoseStamped):
        """
        Callback function to initialize the position and velocity of simulated vehicle when a clicked_point topic is published
        """

        rospy.logwarn("Clicked point callback")

        # Convert the coordinates from the map to the world frame.
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                "world", "map", rospy.Time(0)
            )

            orientation_list = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                orientation_list
            )

            # Initialize the position and orientation using the position of the clicked point
            self.simulated_vehicle.current_state.x = msg.pose.position.x + trans[0]
            self.simulated_vehicle.current_state.y = msg.pose.position.y + trans[1]
            self.simulated_vehicle.current_state.angle = yaw

            # Initialize the speed to zero
            self.simulated_vehicle.current_state.vx = 0

            self.state_initialized = True

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.logwarn("Error during coordinate transform")

    def throttle_pwm_callback(self, msg: Float32):
        """
        Throttle pwm callback, generated by the longitudinal controller
        """
        self.throttle_pwm_cmd = msg.data
        self.time_last_throttle_pwm_msg = time.time()

    def target_curvature_callback(self, msg: Float64):
        """
        Target curvature callback, generated by the rest of the AV stack
        """
        self.target_curvature = msg.data

    def loop(self):
        """
        Main loop of the vehicle simulator
        """

        while not rospy.is_shutdown():

            # If initial state has not been initialized
            if not self.state_initialized:
                rospy.loginfo("Sim has not started.")

            else:

                # Compute steering input
                steering_input = compute_steering_angle_from_curvature(
                    self.target_curvature,
                    self.simulated_vehicle.lateral_model.wheelbase,
                )

                # Predict next position given steering input
                self.simulated_vehicle.update_position(
                    steering_input, self.time_step_secs
                )

                # Check that throttle command is still being published
                if self.throttle_pwm_cmd is not None and (
                    time.time() - self.time_last_throttle_pwm_msg
                    < self.timeout_pwm_msg_before_stop_secs
                ):

                    # Predict next velocity based on longitudinal control model
                    self.simulated_vehicle.update_speed(
                        self.throttle_pwm_cmd, self.time_step_secs
                    )
                else:
                    # stop the vehicle
                    # Instead of calling this function, we could also set throttle_pwm_cmd to something that stops the vehicle
                    rospy.logwarn(
                        f"Throttle is None or throttle_pwm_cmd not received for at least {self.timeout_pwm_msg_before_stop_secs} seconds."
                    )
                    self.simulated_vehicle.stop()

                # Publish current simulated state for debugging
                self.publish_sim_state(self.simulated_vehicle.current_state)

                # Publish Gprmc topic
                self.publish_nmea_sentence(self.simulated_vehicle.current_state)

                # Convert position and velocity to NMEA sentence
                # sentence = self.nmea.create_sentence_string_from_state(self.simulated_vehicle.current_state)

                # Publish sentence on serial
                # self.serial_output.write(sentence)

                # Publish engage mode from simulated arduino
                self.publish_arduino_logging()

            self.rate.sleep()

    def publish_sim_state(self, current_state: State) -> None:
        """
        Function to publish the vehicle's current position and velocity
        Argument:
            - state [State]: Current state of vehicle sim
        """

        """ Current position """
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "world"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = current_state.x
        pose_msg.pose.position.y = current_state.y

        quaternion = tf.transformations.quaternion_from_euler(0, 0, current_state.angle)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        self.sim_pose_pub.publish(pose_msg)

        """ Current velocity """
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = "world"
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.twist.linear.x = current_state.vx

        self.sim_twist_pub.publish(twist_msg)

    def publish_nmea_sentence(self, current_state: State):
        """
        Function to publish the topic Gprmc topic
        Argument:
            - state [State]: Current state of vehicle sim
        """

        # Compute latitude and longitude from UTM coordinates
        zone_number = 10
        zone_letter = "S"
        latitude, longitude = utm.to_latlon(
            current_state.x, current_state.y, zone_number, zone_letter
        )

        # Current time
        now = datetime.now(timezone.utc)
        utc_seconds = now.second + now.microsecond / 1000000
        date = "%s-%s-%s" % (now.year, now.month, now.day)

        # Track angle
        track_angle_deg = ((math.pi / 2) - current_state.angle) * 180 / np.pi

        # Speed
        speed_knots = current_state.vx * MPS_TO_KNOTS

        msg = Gprmc()
        msg.header.stamp = rospy.Time.now()
        msg.utc_seconds = utc_seconds
        msg.lat = latitude
        msg.lon = longitude
        msg.lat_dir = "N"
        msg.lon_dir = "W"
        msg.speed = speed_knots
        msg.track = track_angle_deg
        msg.date = date
        msg.mag_var = -1
        msg.mag_var_direction = ""
        msg.mode_indicator = "A"

        self.rmc_pub.publish(msg)

    def publish_arduino_logging(self):
        """
        Function to publish on the topic /arduino_logging, to engage the speed controller
        """
        arduino_logging_msg = ArduinoLogging()

        arduino_logging_msg.engaged_mode = self.set_arduino_log_engage

        self.arduino_log_pub.publish(arduino_logging_msg)


if __name__ == "__main__":
    try:
        rospy.init_node("Sim")
        sim = Sim()
        sim.loop()
    except rospy.ROSInterruptException:
        sim.ser.close()
        pass
