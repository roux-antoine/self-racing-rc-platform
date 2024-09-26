#!/usr/bin/python3

from enum import Enum
import numpy as np
import rospy
import time

from controllers_pkg.pid import PID
from geometry_msgs.msg import TwistStamped
from self_racing_car_msgs.msg import ArduinoLogging, LongitudinalControllerDebugInfo
from std_msgs.msg import Float32

from dynamic_reconfigure.server import Server

from dynamic_reconfigure_pkg.cfg import (
    longitudinal_controllerConfig,
)


class LongitudinalControlMode(Enum):
    ConstantPwmOutput = 0
    PID = 1


class LongitudinalController:
    def __init__(self):

        # Constants
        rate = rospy.get_param("~rate", 10.0)

        # Subscribers
        rospy.Subscriber(
            "target_velocity",
            TwistStamped,
            self.target_velocity_callback,
        )
        rospy.Subscriber(
            "current_velocity",
            TwistStamped,
            self.current_velocity_callback,
        )
        rospy.Subscriber(
            "arduino_logging",
            ArduinoLogging,
            self.firmware_log_callback,
        )

        # Publishers
        self.throttle_cmd_pub = rospy.Publisher(
            "throttle_pwm_cmd",
            Float32,
            queue_size=10,
        )

        self.pub_debug_pid = rospy.Publisher(
            "pid_debug", LongitudinalControllerDebugInfo, queue_size=10
        )

        # Variables
        self.desired_velocity = 0
        self.current_velocity = 0
        self.anti_windup_enabled = False
        self.pid_controller = PID()
        self.speed_controller_enabled = False
        self.previous_t = time.time()
        self.rate = rospy.Rate(rate)
        self.time_last_arduino_msg = time.time()
        self.startup_mode_enabled = True
        self.controller_previously_disabled = True

        self.dynamic_reconfigure_server = Server(
            longitudinal_controllerConfig,
            self.dynamic_reconfigure_callback,
        )

    def dynamic_reconfigure_callback(self, config, level):
        self.throttle_idle_autonomous_pwm = config["throttle_idle_autonomous_pwm"]
        self.throttle_max_autonomous_pwm = config["throttle_max_autonomous_pwm"]
        self.throttle_min_autonomous_pwm = config["throttle_min_autonomous_pwm"]
        self.constant_pwm_output = config["constant_pwm_output"]
        self.timeout_engage_msg_before_stop_secs = config[
            "timeout_engage_msg_before_stop_secs"
        ]
        self.t_ramp_sec = config["t_ramp_sec"]

        if config["longitudinal_control_mode"] == LongitudinalControlMode.PID.value:
            self.longitudinal_control_mode = LongitudinalControlMode.PID
        elif (
            config["longitudinal_control_mode"]
            == LongitudinalControlMode.ConstantPwmOutput.value
        ):
            self.longitudinal_control_mode = LongitudinalControlMode.ConstantPwmOutput
        else:
            raise ValueError("Invalid value for longitudinal_control_mode.")
        self.pid_controller.kp = config["kp"]
        self.pid_controller.ki = config["ki"]
        self.pid_controller.kd = config["kd"]

        return config

    def current_velocity_callback(self, msg: TwistStamped):
        """
        Callback function for current velocity message
        """
        self.current_velocity = msg.twist.linear.x

    def firmware_log_callback(self, msg: ArduinoLogging):
        """
        Callback of log message from low level controller.
        """
        self.speed_controller_enabled = msg.engaged_mode
        self.time_last_arduino_msg = time.time()

    def target_velocity_callback(self, msg: TwistStamped):
        """
        Callback function for target velocity message. Main function of the controller.
        """
        self.desired_velocity = msg.twist.linear.x

    def loop(self):
        """
        Main loop of the longitudinal controller
        """

        while not rospy.is_shutdown():

            duration_since_last_message = time.time() - self.time_last_arduino_msg

            if self.longitudinal_control_mode == LongitudinalControlMode.PID:

                # Speed controller ENABLED
                if (
                    self.speed_controller_enabled
                    and duration_since_last_message
                    < self.timeout_engage_msg_before_stop_secs
                ):
                    rospy.loginfo("-------------")
                    rospy.loginfo("Controller ON")

                    # Soft start mechanism

                    # TODO: Change some variables' names
                    # TODO: We could also not trigger the PID in startup mode (?)
                    # Check if we have just enabled the controller
                    if self.controller_previously_disabled:
                        self.controller_previously_disabled = False
                        self.startup_mode_enabled = True
                        self.t_controller_enabled = time.time()
                        self.speed_when_controller_enabled = self.current_velocity

                    if self.startup_mode_enabled:

                        t_elapsed_since_controller_enabled = (
                            time.time() - self.t_controller_enabled
                        )

                        rampRate = (
                            self.desired_velocity - self.speed_when_controller_enabled
                        ) / self.t_ramp_sec
                        target_speed_ramp = (
                            self.speed_when_controller_enabled
                            + rampRate * t_elapsed_since_controller_enabled
                        )

                        target_velocity = target_speed_ramp

                        # Check if we still need to be in startup mode
                        # Exit startup mode if the ramp time is elapsed or if we the ramp speed is higher than target speed
                        if (
                            t_elapsed_since_controller_enabled > self.t_ramp_sec
                            or target_speed_ramp > self.desired_velocity
                        ):
                            self.startup_mode_enabled = False
                            target_velocity = self.desired_velocity

                    # Not in startup mode
                    else:
                        target_velocity = self.desired_velocity

                    # Compute sample time
                    dt = time.time() - self.previous_t

                    # Compute velocity error
                    error = target_velocity - self.current_velocity
                    rospy.loginfo(f"Error: {error}")

                    # Feedforward term
                    ff = self.compute_feedforward_term(target_velocity)

                    # Call the PID controller
                    throttle_diff, p, i, d = self.pid_controller.update(
                        error, self.anti_windup_enabled, dt
                    )
                    rospy.loginfo(f"Throttle diff: {throttle_diff}")

                    # Add the controller's output value to the feedforward term
                    throttle_value = throttle_diff + ff

                    #  --- Anti windup strategy ---

                    # Check if throttle output is already saturating the actuator
                    if (
                        throttle_value < self.throttle_min_autonomous_pwm
                        or throttle_value > self.throttle_max_autonomous_pwm
                    ):
                        throttle_saturating = True

                        if throttle_value < self.throttle_min_autonomous_pwm:
                            throttle_value = self.throttle_min_autonomous_pwm

                        elif throttle_value > self.throttle_max_autonomous_pwm:
                            throttle_value = self.throttle_max_autonomous_pwm

                    else:
                        throttle_saturating = False

                    # Check if controller is trying to saturate the actuator even more
                    if np.sign(error) == np.sign(throttle_diff):
                        controller_saturating = True
                    else:
                        controller_saturating = False

                    # If both above are true, then stop integrating.
                    if throttle_saturating and controller_saturating:
                        self.anti_windup_enabled = True
                    else:
                        self.anti_windup_enabled = False

                    rospy.loginfo(f"Anti windup: {self.anti_windup_enabled}")

                    #  Publish result
                    self.publish_throttle_cmd(throttle_value)
                    self.publish_debug_pid(
                        throttle_saturating,
                        controller_saturating,
                        p,
                        i,
                        d,
                        error,
                        ff,
                        self.startup_mode_enabled,
                        target_velocity,
                    )

                # Speed controller DISABLED
                else:
                    # We are in PID mode but the controller is disabled because the engage button
                    # is not pressed (or value not received since for too long)
                    rospy.loginfo("Controller OFF")

                    # Variable to know if
                    self.controller_previously_disabled = True

                    # Publish the throttle IDLE value
                    self.publish_throttle_cmd(self.throttle_idle_autonomous_pwm)
                    # Reset the integral component
                    self.pid_controller.reset()

                    # Publish debug but with all values at zero
                    self.publish_debug_pid(
                        False,
                        False,
                        0,
                        0,
                        0,
                        0,
                        0,
                        False,
                        0,
                    )

            elif (
                self.longitudinal_control_mode
                == LongitudinalControlMode.ConstantPwmOutput
            ):

                if (
                    self.speed_controller_enabled
                    and duration_since_last_message
                    < self.timeout_engage_msg_before_stop_secs
                ):

                    self.publish_throttle_cmd(self.constant_pwm_output)

                else:
                    # Publish the throttle IDLE value
                    self.publish_throttle_cmd(self.throttle_idle_autonomous_pwm)

            # Reset the previous time to avoid having a large integral value when we engage again
            self.previous_t = time.time()

            self.rate.sleep()

    def publish_throttle_cmd(self, throttle_value: float):
        """
        Function to publish on throttle topic
        """

        throttle_msg = Float32()
        throttle_msg.data = throttle_value

        self.throttle_cmd_pub.publish(throttle_msg)

    def publish_debug_pid(
        self,
        throttle_saturating,
        controller_saturating,
        p,
        i,
        d,
        error,
        ff,
        startup_mode_enabled,
        target_velocity,
    ):
        """
        Function to publish debug info regarding speed controller
        """

        debug_msg = LongitudinalControllerDebugInfo()
        debug_msg.throttle_saturating = throttle_saturating
        debug_msg.controller_saturating = controller_saturating
        debug_msg.p = p
        debug_msg.i = i
        debug_msg.d = d
        debug_msg.error = error
        debug_msg.feedforward = ff
        debug_msg.startup_mode_enabled = startup_mode_enabled
        debug_msg.target_velocity = target_velocity

        self.pub_debug_pid.publish(debug_msg)

    def compute_feedforward_term(self, desired_velocity):
        """
        Function to correlate desired velocity (m/s) to required throttle pwm,
        given a simple linear approximation
        """

        # ff = 1.92 * desired_velocity + 94.1
        ff = 1.4 * desired_velocity + 94.1

        return ff


if __name__ == "__main__":
    try:
        rospy.init_node("longitudinal_controller")
        longitudinal_controller = LongitudinalController()
        longitudinal_controller.loop()
    except rospy.ROSInterruptException:
        pass
